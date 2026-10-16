"""Static-scene collision queries using the same URDF as Gazebo.

PyBullet runs in DIRECT mode (no window, no time stepping). Mesh collision
shapes are convex hulls: conservative around concave details. Self pairs on
one rigid assembly or directly adjoining assemblies are excluded explicitly.
Only the rigid robot base may contact the named floor; all moving links and
the target object participate in environment checks.
"""
from pathlib import Path
import itertools
import math
import tempfile
import xml.etree.ElementTree as ET

import numpy as np
import pybullet as p
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from ament_index_python.packages import get_package_share_directory

JOINT_NAMES = [f'panda_joint{i}' for i in range(1, 8)]


def _prepare(xml, destination):
    root = ET.fromstring(xml)
    for tag in ('gazebo', 'ros2_control', 'transmission'):
        for element in list(root.findall(tag)):
            root.remove(element)
    for link in root.findall('link'):
        for visual in list(link.findall('visual')):
            link.remove(visual)
        if link.find('inertial') is None:
            inertial = ET.SubElement(link, 'inertial')
            ET.SubElement(inertial, 'mass', value='0')
            ET.SubElement(inertial, 'inertia', ixx='1', iyy='1', izz='1', ixy='0', ixz='0', iyz='0')
        for mesh in link.findall('./collision/geometry/mesh'):
            filename = mesh.get('filename', '')
            if filename.startswith('package://'):
                package, relative = filename[len('package://'):].split('/', 1)
                filename = str(Path(get_package_share_directory(package)) / relative)
                mesh.set('filename', filename)
            if not Path(filename).is_file():
                raise ValueError(f'Collision mesh missing: {filename}')
    ET.indent(root, space='  ')
    ET.ElementTree(root).write(destination, encoding='utf-8', xml_declaration=True)
    return root


class CollisionChecker:
    def __init__(self, robot_xml, world_urdf, clearance=0.008):
        if not math.isfinite(clearance) or clearance < 0:
            raise ValueError('clearance must be a finite nonnegative distance')
        self.clearance = clearance
        self.last_reason = ''
        self.allow_finger_target = False
        self.finger_width = .04
        self.client = p.connect(p.DIRECT)
        self.temp = tempfile.TemporaryDirectory(prefix='panda_collision_')
        try:
            robot_file = str(Path(self.temp.name) / 'robot.urdf')
            world_file = str(Path(self.temp.name) / 'world.urdf')
            root = _prepare(robot_xml, robot_file)
            _prepare(Path(world_urdf).read_text(), world_file)
            self.robot = p.loadURDF(robot_file, useFixedBase=True, physicsClientId=self.client)
            self.world = p.loadURDF(world_file, useFixedBase=True, physicsClientId=self.client)
            self.robot_links, self.joints = self._indices(self.robot)
            self.world_links, _ = self._indices(self.world)
            self.world_names = {i: name for name, i in self.world_links.items()}
            target_root = ET.parse(world_urdf).getroot()
            target_link = target_root.find("link[@name='target_cube']")
            target_size = [float(x) for x in target_link.find('./collision/geometry/box').get('size').split()]
            shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[v/2 for v in target_size], physicsClientId=self.client)
            target_joint = next(j for j in target_root.findall('joint')
                                if j.find('child').get('link') == 'target_cube')
            if target_joint.find('parent').get('link') != 'obstacle_world_root':
                raise ValueError('Target must attach directly to the scene root')
            origin = target_joint.find('origin')
            target_position = [float(x) for x in origin.get('xyz','0 0 0').split()]
            target_rotation = p.getQuaternionFromEuler([float(x) for x in origin.get('rpy','0 0 0').split()])
            self.target_body = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=shape,
                                                 basePosition=target_position, baseOrientation=target_rotation,
                                                 physicsClientId=self.client)
            self.attachment = None
            self.support_contacts = set()
            self.robot_names = {i: name for name, i in self.robot_links.items()}
            if any(name not in self.joints for name in JOINT_NAMES):
                raise ValueError('Expected seven Panda joints')
            self.joint_indices = [self.joints[name] for name in JOINT_NAMES]
            limits = {j.get('name'): j.find('limit') for j in root.findall('joint')}
            self.lower = np.array([float(limits[n].get('lower')) for n in JOINT_NAMES])
            self.upper = np.array([float(limits[n].get('upper')) for n in JOINT_NAMES])
            self.velocity = np.array([float(limits[n].get('velocity')) for n in JOINT_NAMES])
            if (not np.all(np.isfinite([self.lower, self.upper, self.velocity]))
                    or np.any(self.lower >= self.upper) or np.any(self.velocity <= 0)):
                raise ValueError('Invalid robot joint limits')
            # Rigid assemblies are connected components of fixed joints.
            names = [link.get('name') for link in root.findall('link')]
            groups = {name: name for name in names}
            def group(name):
                while groups[name] != name:
                    name = groups[name]
                return name
            for joint in root.findall('joint'):
                if joint.get('type') == 'fixed':
                    a, b = joint.find('parent').get('link'), joint.find('child').get('link')
                    groups[group(b)] = group(a)
            adjacent = set()
            for joint in root.findall('joint'):
                a, b = joint.find('parent').get('link'), joint.find('child').get('link')
                adjacent.add(frozenset((group(a), group(b))))
            collision_links = [link.get('name') for link in root.findall('link') if link.find('collision') is not None]
            self.self_pairs = set()
            for a, b in itertools.combinations(collision_links, 2):
                if group(a) != group(b) and frozenset((group(a), group(b))) not in adjacent:
                    self.self_pairs.add(tuple(sorted((self.robot_links[a], self.robot_links[b]))))
            child_names = {j.find('child').get('link') for j in root.findall('joint')}
            base = next(name for name in names if name not in child_names)
            self.base_links = {index for name, index in self.robot_links.items() if group(name) == group(base)}
            self.tcp = self.robot_links['panda_hand_tcp']
            self.collision_links = collision_links
            for name in collision_links:
                if not p.getCollisionShapeData(self.robot, self.robot_links[name], physicsClientId=self.client):
                    raise ValueError(f'Collision geometry failed to load for {name}')
        except Exception:
            self.close()
            raise

    def _indices(self, body):
        links = {p.getBodyInfo(body, physicsClientId=self.client)[0].decode(): -1}
        joints = {}
        for i in range(p.getNumJoints(body, physicsClientId=self.client)):
            info = p.getJointInfo(body, i, physicsClientId=self.client)
            joints[info[1].decode()] = i
            links[info[12].decode()] = i
        return links, joints

    def set_configuration(self, q):
        q = np.asarray(q, dtype=float)
        if q.shape != (7,) or not np.all(np.isfinite(q)):
            raise ValueError('Expected seven finite joint positions')
        for name in ('panda_finger_joint1', 'panda_finger_joint2'):
            if name in self.joints:
                p.resetJointState(self.robot, self.joints[name], self.finger_width, physicsClientId=self.client)
        for index, angle in zip(self.joint_indices, q):
            p.resetJointState(self.robot, index, float(angle), physicsClientId=self.client)
        if self.attachment is not None:
            state = p.getLinkState(self.robot, self.tcp, computeForwardKinematics=True, physicsClientId=self.client)
            position, orientation = p.multiplyTransforms(state[4], state[5], *self.attachment)
            p.resetBasePositionAndOrientation(self.target_body, position, orientation, physicsClientId=self.client)
        return q

    def target_pose(self, position, orientation):
        self.attachment = None
        p.resetBasePositionAndOrientation(self.target_body, position, orientation, physicsClientId=self.client)

    def attach_payload(self, captured_q, captured_position, captured_orientation):
        self.attachment = None
        self.set_configuration(captured_q)
        state = p.getLinkState(self.robot, self.tcp, computeForwardKinematics=True, physicsClientId=self.client)
        inverse = p.invertTransform(state[4], state[5])
        self.attachment = p.multiplyTransforms(*inverse, captured_position, captured_orientation)

    def is_state_valid(self, q):
        q = self.set_configuration(q)
        if np.any(q < self.lower) or np.any(q > self.upper):
            self.last_reason = 'joint limit'
            return False
        for point in p.getClosestPoints(self.robot, self.robot, distance=0.002, physicsClientId=self.client):
            a, b = point[3], point[4]
            if tuple(sorted((a, b))) in self.self_pairs:
                self.last_reason = f'self: {self.robot_names[a]} / {self.robot_names[b]}'
                return False
        for point in p.getClosestPoints(self.robot, self.world, distance=self.clearance, physicsClientId=self.client):
            a, b = point[3], point[4]
            if self.world_names.get(b) == 'target_cube':
                continue
            if (self.allow_finger_target and self.world_names.get(b) == 'target_cube'
                    and self.robot_names[a] in ('panda_leftfinger', 'panda_rightfinger')):
                continue
            if a in self.base_links and self.world_names.get(b) == 'floor':
                continue
            self.last_reason = f'environment: {self.robot_names[a]} / {self.world_names.get(b)} ({point[8]:.4f} m)'
            return False
        for point in p.getClosestPoints(self.robot, self.target_body, distance=self.clearance, physicsClientId=self.client):
            if self.allow_finger_target and self.robot_names[point[3]] in ('panda_leftfinger', 'panda_rightfinger'):
                continue
            self.last_reason = f'target: {self.robot_names[point[3]]} ({point[8]:.4f} m)'
            return False
        if self.attachment is not None:
            for point in p.getClosestPoints(self.target_body, self.world, distance=self.clearance, physicsClientId=self.client):
                name = self.world_names.get(point[4])
                if name == 'target_cube':
                    continue
                if name in self.support_contacts and point[8] >= -.002:
                    continue
                self.last_reason = f'payload: target_cube / {name} ({point[8]:.4f} m)'
                return False
        self.last_reason = ''
        return True

    def tcp_pose(self, q):
        self.set_configuration(q)
        state = p.getLinkState(self.robot, self.tcp, computeForwardKinematics=True, physicsClientId=self.client)
        return np.array(state[4]), np.array(state[5])

    def local_ik(self, position, seed, quaternion):
        position = np.asarray(position, dtype=float)
        seed = np.clip(np.asarray(seed, dtype=float), self.lower+1e-6, self.upper-1e-6)
        target_rotation = Rotation.from_quat(quaternion)
        def residual(q):
            xyz, orientation = self.tcp_pose(q)
            angle = (target_rotation.inv()*Rotation.from_quat(orientation)).as_rotvec()
            return np.r_[10*(xyz-position), angle, .0001*(q-seed)]
        def jacobian(q):
            value = residual(q)
            jac = np.zeros((len(value),7))
            for i in range(7):
                h = 1e-4 if q[i]+1e-4 < self.upper[i] else -1e-4
                changed = q.copy(); changed[i] += h
                jac[:,i] = (residual(changed)-value)/h
            return jac
        result = least_squares(residual,seed,jac=jacobian,
                               bounds=(self.lower+1e-7,self.upper-1e-7),
                               max_nfev=150,ftol=1e-8,xtol=1e-8,gtol=1e-8)
        actual, orientation = self.tcp_pose(result.x)
        if (np.linalg.norm(actual-position)>.003
                or abs(float(np.dot(orientation,quaternion))) < math.cos(.03/2)):
            raise ValueError('连续逆解未收敛到目标')
        if not self.is_state_valid(result.x):
            raise ValueError('连续逆解碰撞：'+self.last_reason)
        return result.x.tolist()

    def inverse_kinematics(self, position, seed, quaternion=None, attempts=40):
        """Return a valid IK solution; verify actual FK residual and collisions."""
        position = np.asarray(position, dtype=float)
        if position.shape != (3,) or not np.all(np.isfinite(position)):
            raise ValueError('Target position must have three finite values')
        if quaternion is not None:
            quaternion = np.asarray(quaternion, dtype=float)
            norm = np.linalg.norm(quaternion)
            if quaternion.shape != (4,) or not np.all(np.isfinite(quaternion)) or norm < 1e-9:
                raise ValueError('Invalid target quaternion')
            quaternion = quaternion / norm
        if quaternion is not None:
            try:
                return self.local_ik(position,seed,quaternion)
            except ValueError:
                pass
        rng = np.random.default_rng(7)
        best, best_distance = None, float("inf")
        for attempt in range(attempts):
            rest = np.array(seed) if attempt == 0 else rng.uniform(self.lower, self.upper)
            self.set_configuration(rest)
            extra = 2 if 'panda_finger_joint1' in self.joints else 0
            kwargs = dict(bodyUniqueId=self.robot, endEffectorLinkIndex=self.tcp,
                          targetPosition=position.tolist(), lowerLimits=self.lower.tolist()+[0.]*extra,
                          upperLimits=self.upper.tolist()+[.04]*extra, jointRanges=(self.upper-self.lower).tolist()+[.04]*extra,
                          restPoses=rest.tolist()+[.04]*extra, maxNumIterations=250, residualThreshold=1e-5,
                          physicsClientId=self.client)
            if quaternion is not None:
                kwargs['targetOrientation'] = quaternion.tolist()
            q = np.asarray(p.calculateInverseKinematics(**kwargs)[:7])
            if not self.is_state_valid(q):
                continue
            actual, orientation = self.tcp_pose(q)
            if np.linalg.norm(actual-position) > 0.005:
                continue
            if quaternion is not None and abs(float(np.dot(orientation, quaternion))) < math.cos(0.05/2):
                continue
            distance = float(np.linalg.norm(q-np.asarray(seed)))
            if distance < best_distance:
                best, best_distance = q.tolist(), distance
            if attempt == 0 and distance < .8:
                return best
        if best is not None:
            return best
        raise ValueError('No collision-free IK solution within position/orientation tolerance')

    def close(self):
        if getattr(self, 'client', -1) >= 0:
            p.disconnect(physicsClientId=self.client)
            self.client = -1
        if hasattr(self, 'temp'):
            self.temp.cleanup()
