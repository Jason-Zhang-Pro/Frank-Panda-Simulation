"""Offline integration checks against the installed Panda model and scene."""

def test_pipeline():
    import xacro
    import numpy as np
    from ament_index_python.packages import get_package_share_directory
    from panda_planning.collision_checker import CollisionChecker
    from panda_planning.path_planner import JointRRT,RRTConfig
    from panda_planning.trajectory_planner import TrajectoryPlanner
    from panda_planning.control_node import TimedTrajectory
    share=get_package_share_directory('panda_planning')
    xml=xacro.process_file(get_package_share_directory('franka_description')+'/robots/panda/panda_sim.urdf.xacro').toxml()
    import xml.etree.ElementTree as ET
    scene=ET.parse(share+'/worlds/world.urdf').getroot()
    target=scene.find("joint[@name='target_cube_fixed']/origin")
    target_xyz=[float(v) for v in target.get('xyz').split()]
    target_xyz[2]+=.13
    c=CollisionChecker(xml,share+'/worlds/world.urdf')
    q=[0,-.785398,0,-2.356194,0,1.570796,.785398]
    assert c.is_state_valid(q),c.last_reason
    end=c.inverse_kinematics(target_xyz,q)
    path=JointRRT(list(zip(c.lower,c.upper)),c.is_state_valid,RRTConfig(seed=10,timeout=10)).plan(q,end)
    assert path
    msg,samples=TrajectoryPlanner(c.velocity*.2,[.6]*7).build(path,c)
    tr=TimedTrajectory(msg,q,c.lower,c.upper,c.velocity)
    tr.sample(0)
    last=msg.points[-1].time_from_start
    seconds=last.sec+last.nanosec*1e-9
    for t in np.linspace(0,seconds,250):
     ref,vel,done=tr.sample(t)
     assert c.is_state_valid(ref),c.last_reason
     assert np.all(np.abs(vel)<=c.velocity*.2+1e-6)
    assert np.linalg.norm(c.tcp_pose(end)[0]-target_xyz)<.005
    # Invalid states and real self/environment collisions must be rejected.
    assert not c.is_state_valid([0]*7)
    rng=np.random.default_rng(3)
    reasons=set()
    for _ in range(500):
     if not c.is_state_valid(rng.uniform(c.lower,c.upper)):
      reasons.add(c.last_reason.split(':')[0])
    assert 'self' in reasons and 'environment' in reasons,reasons
    # Analytical acceleration bound, finite difference independently confirms it.
    tr=TimedTrajectory(msg,q,c.lower,c.upper,c.velocity);tr.sample(0)
    times=np.linspace(0,seconds,5000)
    v=np.array([tr.sample(t)[1] for t in times])
    a=np.diff(v,axis=0)/np.diff(times)[:,None]
    assert np.max(np.abs(a))<=.6001,np.max(np.abs(a))
    print('PASS: real Panda FK/IK, self/environment collisions, RRT, sampled trajectory collision, velocity, acceleration and controller compatibility')
    print('PATH',len(path),'samples',len(samples),'duration',seconds,'target',end)
    c.close()
