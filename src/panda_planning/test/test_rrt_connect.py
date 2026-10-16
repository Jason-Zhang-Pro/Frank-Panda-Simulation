"""Bidirectional path orientation and full-edge collision regression tests."""
import math
import pytest
from panda_planning.path_planner import JointRRTConnect,RRTConfig


@pytest.mark.parametrize('seed', range(6))
def test_detour_and_path_direction(seed):
    valid=lambda q: q[0]**2+q[1]**2>.3**2
    planner=JointRRTConnect([(-1,1),(-1,1)],valid,RRTConfig(seed=seed,timeout=2))
    path=planner.plan([-.8,0],[.8,0])
    assert planner.last_status=='success'
    assert path[0]==[-.8,0.] and path[-1]==[.8,0.]
    assert len(path)>2
    for a,b in zip(path,path[1:]):
        assert math.dist(a,b)<=planner.config.step_size+1e-10
        for i in range(101):
            assert valid([x+(y-x)*i/100 for x,y in zip(a,b)])


def test_failure_reasons_and_endpoints():
    blocked=JointRRTConnect([(-1,1)],lambda q:abs(q[0])>.2,RRTConfig(seed=0,max_iterations=20))
    assert blocked.plan([-.8],[.8]) is None
    assert blocked.last_status=='iteration_limit'
    with pytest.raises(ValueError):blocked.plan([0],[.8])
    timeout=JointRRTConnect([(-1,1)],lambda q:True,RRTConfig(timeout=1e-12))
    assert timeout.plan([-.8],[.8]) is None and timeout.last_status=='timeout'
    direct=JointRRTConnect([(-1,1)],lambda q:True)
    assert direct.plan([0],[0])==[[0.]]
    assert direct.plan([0],[.5])==[[0.],[.5]]
