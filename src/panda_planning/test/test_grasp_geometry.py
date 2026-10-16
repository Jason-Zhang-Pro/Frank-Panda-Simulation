"""Validate finger kinematics, gravity and carried-object collision policy."""
import numpy as np
import xacro
from ament_index_python.packages import get_package_share_directory
from panda_planning.collision_checker import CollisionChecker
from panda_planning.control_node import GravityModel,Q_TARGET


def test_grasp_geometry():
    xml=xacro.process_file(get_package_share_directory('franka_description')+'/robots/panda/panda_sim.urdf.xacro').toxml()
    model=GravityModel(xml)
    q=Q_TARGET.copy()
    gravity,_=model.evaluate(q,[.02,.03])
    gradient=[]
    for i in range(7):
        plus,minus=q.copy(),q.copy();plus[i]+=1e-6;minus[i]-=1e-6
        gradient.append((model.evaluate(plus,[.02,.03])[1]-model.evaluate(minus,[.02,.03])[1])/2e-6)
    assert np.allclose(gravity,gradient,atol=1e-6)
    import xml.etree.ElementTree as ET
    world=get_package_share_directory('panda_planning')+'/worlds/world.urdf'
    scene=ET.parse(world).getroot()
    target=[float(v) for v in scene.find("joint[@name='target_cube_fixed']/origin").get('xyz').split()]
    grasp_xyz=[target[0],target[1],target[2]+.005]
    lift_xyz=[target[0],target[1],target[2]+.105]
    c=CollisionChecker(xml,world)
    try:
        c.allow_finger_target=True
        grasp=c.local_ik(grasp_xyz,q,[1,0,0,0])
        c.finger_width=.03
        c.attach_payload(grasp,target,[0,0,0,1])
        assert not c.is_state_valid(grasp)
        assert c.last_reason.startswith('payload:'),c.last_reason
        c.support_contacts={'target_pedestal'}
        assert c.is_state_valid(grasp),c.last_reason
        lifted=c.local_ik(lift_xyz,grasp,[1,0,0,0])
        c.support_contacts=set()
        assert c.is_state_valid(lifted),c.last_reason
    finally:
        c.close()
