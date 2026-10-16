"""Contact-verified physical pick and place; no weld or teleportation."""
import math
import time
import xml.etree.ElementTree as ET
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from std_srvs.srv import Trigger
from ros_gz_interfaces.msg import Contacts
from tf2_msgs.msg import TFMessage
from rclpy.qos import qos_profile_sensor_data


class GraspTask:
    def __init__(self, planner):
        self.node = planner
        self.active = False
        self.stage = ''
        self.q = self.dq = None
        self.contacts = {}
        self.target_state = None
        self.target_time = -1000.
        self.arm_done = False
        self.failure = None
        self.payload = None
        self.width_pub = planner.create_publisher(Float64, '/gripper_goal', 10)
        planner.create_subscription(JointState, '/joint_states', self.state, qos_profile_sensor_data)
        planner.create_subscription(Contacts, '/target_contacts', self.contact, qos_profile_sensor_data)
        planner.create_subscription(TFMessage, '/world/empty/dynamic_pose/info', self.poses, qos_profile_sensor_data)
        planner.create_service(Trigger, '/pick_and_place', self.pick_and_place)
        planner.create_service(Trigger, '/grasp_target', self.pick_and_place)
        planner.create_service(Trigger, '/open_gripper', lambda req, res: self.manual(.04, res))
        planner.create_service(Trigger, '/close_gripper', lambda req, res: self.manual(0., res))
        planner.create_timer(.05, self.poll)

    def state(self, msg):
        values = {name: i for i, name in enumerate(msg.name)}
        try:
            names = ['panda_finger_joint1', 'panda_finger_joint2']
            self.q = [msg.position[values[name]] for name in names]
            self.dq = [msg.velocity[values[name]] for name in names]
        except (KeyError, IndexError):
            pass

    def poses(self, msg):
        for transform in msg.transforms:
            if transform.child_frame_id == 'target_cube':
                t, q = transform.transform.translation, transform.transform.rotation
                values = [t.x,t.y,t.z,q.x,q.y,q.z,q.w]
                if all(math.isfinite(v) for v in values):
                    self.target_state = (values[:3],values[3:])
                    self.target_time = self.node.sim_time

    def contact(self, msg):
        for contact in msg.contacts:
            names = contact.collision1.name + ' ' + contact.collision2.name
            if 'target_cube' in names:
                for finger in ('panda_leftfinger','panda_rightfinger'):
                    if finger in names:
                        self.contacts[finger] = self.node.sim_time

    def both_contacts(self):
        return all(self.node.sim_time-self.contacts.get(n,-1000.) < .25
                   for n in ('panda_leftfinger','panda_rightfinger'))

    def manual(self, width, response):
        if self.active or self.node.busy or self.payload is not None:
            response.success,response.message = False,'任务或携物状态下不允许手动操作夹爪'
        elif self.q is None or self.width_pub.get_subscription_count()!=1:
            response.success,response.message = False,'夹爪控制未准备好'
        else:
            self.width_pub.publish(Float64(data=width))
            response.success,response.message = True,'已发送开口指令；不代表抓取成功'
        return response

    def pick_and_place(self, request, response):
        try:
            if self.active or self.node.busy or self.payload is not None:
                raise ValueError('任务正在执行或仍处于携物状态')
            if self.q is None or time.monotonic()-self.node.feedback_time>.5:
                raise ValueError('请先启动控制并播放仿真')
            if self.target_state is None or self.node.sim_time-self.target_time>.5:
                raise ValueError('尚未收到 Gazebo 红色目标的实际位置')
            if self.width_pub.get_subscription_count()!=1:
                raise ValueError('夹爪控制未连接')
            root=ET.parse(self.node.get_parameter('world_urdf').value).getroot()
            joint=next(j for j in root.findall('joint') if j.find('child').get('link')=='place_pedestal')
            origin=[float(x) for x in joint.find('origin').get('xyz').split()]
            box=root.find("link[@name='place_pedestal']/collision/geometry/box")
            self.place=[origin[0],origin[1],origin[2]+float(box.get('size').split()[2])/2+.03]
            self.pick=self.target_state[0].copy()
            if np.linalg.norm(np.array(self.pick[:2])-self.place[:2])<.06:
                raise ValueError('目标已经在放置台附近，请重置场景后再试')
            self.active=True
            self.failure=None
            self.stage='opening'
            self.started=self.node.sim_time
            self.wall_started=time.monotonic()
            self.contacts.clear()
            self.width_pub.publish(Float64(data=.04))
            self.node.status('pick_opening: 张开夹爪，准备拾取')
            response.success,response.message=True,'拾取放置任务已提交；查看 /planner_status'
        except Exception as exc:
            response.success,response.message=False,str(exc)
        return response

    def on_planner_status(self,text):
        if not self.active:return
        if text.startswith('succeeded:'):self.arm_done=True
        elif text.startswith(('failed:','rejected:')):self.failure=text

    def move(self,stage,position,cartesian=False):
        self.stage=stage
        self.arm_done=False
        self.node.request('grasp_descent' if cartesian else 'grasp_pose',
                          (position,[1.,0.,0.,0.]),internal=True)

    def poll(self):
        if not self.active:return
        try:
            if self.failure:raise ValueError(self.failure)
            if self.stage == 'homing':
                if time.monotonic()-self.home_started>60:
                    raise ValueError('归位超时，请检查仿真和控制反馈')
            elif time.monotonic()-self.wall_started>150:
                raise ValueError('流程超时，请检查是否暂停仿真')
            if self.node.sim_time-self.target_time>.5:raise ValueError('Gazebo 目标位置反馈中断')
            if self.stage=='opening':
                if self.q and min(self.q)>=.038:
                    self.move('approaching',[self.pick[0],self.pick[1],self.pick[2]+.13])
                elif self.node.sim_time-self.started>5:raise ValueError('张开夹爪失败')
            elif self.stage=='approaching' and self.arm_done:
                self.move('descending',[self.pick[0],self.pick[1],self.pick[2]+.005],True)
            elif self.stage=='descending' and self.arm_done:
                self.stage='closing';self.started=self.node.sim_time;self.contacts.clear()
                self.width_pub.publish(Float64(data=0.))
                self.node.status('pick_closing: 闭合并等待双指接触')
            elif self.stage=='closing':
                elapsed=self.node.sim_time-self.started
                if elapsed>2.2 and self.both_contacts() and self.q and min(self.q)>.015 and max(self.q)<.038:
                    self.payload=(self.node.current_q.copy(),self.target_state[0].copy(),
                                  self.target_state[1].copy(),float(np.mean(self.q)))
                    self.move('lifting',[self.pick[0],self.pick[1],self.pick[2]+.105],True)
                elif elapsed>5:raise ValueError('未确认双指夹住目标，停止任务')
            elif self.stage=='lifting' and self.arm_done:
                if self.target_state[0][2]<self.pick[2]+.055:
                    raise ValueError('实际方块没有被提起，不能继续搬运')
                if not self.both_contacts():raise ValueError('提起后目标失去双指接触')
                self.node.status('pick_lifted: 已确认方块实际抬起，规划携物搬运路径')
                self.move('transferring',[self.place[0],self.place[1],self.place[2]+.13])
            elif self.stage=='transferring' and self.arm_done:
                if not self.both_contacts():raise ValueError('搬运后目标失去双指接触')
                self.move('lowering',[self.place[0],self.place[1],self.place[2]+.005],True)
            elif self.stage=='lowering' and self.arm_done:
                if np.linalg.norm(np.array(self.target_state[0][:2])-self.place[:2])>.04:
                    raise ValueError('实际方块未到达放置位置')
                self.stage='releasing';self.started=self.node.sim_time
                self.width_pub.publish(Float64(data=.04))
                self.node.status('place_releasing: 目标已下降，张开夹爪')
            elif self.stage=='releasing':
                if self.q and min(self.q)>=.038 and self.node.sim_time-self.started>1:
                    actual=self.target_state[0]
                    if np.linalg.norm(np.array(actual)-self.place)>.025:
                        raise ValueError('目标没有落到放置台预期位置')
                    self.payload=None
                    self.move('retreating',[self.place[0],self.place[1],self.place[2]+.15],True)
                elif self.node.sim_time-self.started>5:raise ValueError('松开夹爪失败')
            elif self.stage=='retreating' and self.arm_done:
                if np.linalg.norm(np.array(self.target_state[0])-self.place)>.025:
                    raise ValueError('退回后物体未稳定在放置位置')
                self.stage='homing'
                self.arm_done=False
                self.home_started=time.monotonic()
                self.node.status('returning_home: 放置完成，正在用 RRT 规划回初始关节姿态')
                self.node.request('joint', self.node.get_parameter('home_joint_positions').value, internal=True)
            elif self.stage=='homing' and self.arm_done:
                if np.linalg.norm(np.array(self.target_state[0])-self.place)>.025:
                    raise ValueError('归位后检查发现物体偏离放置位置')
                self.active=False
                self.node.status('pick_place_succeeded: 方块已放置并松开，机械臂已避障归位，夹爪保持张开')
        except Exception as exc:
            self.active=False
            # If physically holding the object, keep it closed and retain payload
            # state. Do not silently drop it or permit unrelated plans.
            if self.stage == 'homing':
                self.node.status('pick_place_home_failed: 放置已完成，但归位失败：'+str(exc)+'；保持当前姿态')
            else:
                self.node.status('pick_place_failed: '+str(exc)+'；保持当前夹爪指令')
