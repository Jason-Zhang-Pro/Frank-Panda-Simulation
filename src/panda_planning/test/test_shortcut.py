"""Shortcut preserves endpoints and cannot cut through an obstacle."""
import math
import pytest
from panda_planning.path_planner import JointRRTConnect,RRTConfig


def length(path):
    return sum(math.dist(a,b) for a,b in zip(path,path[1:]))


def test_free_path_collapses_without_mutating_input():
    p=JointRRTConnect([(-2,2)]*2,lambda q:True,RRTConfig(seed=1))
    original=[[-1.,0.],[-.5,.4],[0.,.6],[1.,0.]]
    result=p.shortcut(original)
    assert result==[original[0],original[-1]]
    assert len(original)==4
    assert length(result)<=length(original)


def test_obstacle_cannot_be_shortcut():
    valid=lambda q:q[0]**2+q[1]**2>.25**2
    p=JointRRTConnect([(-1,1)]*2,valid,RRTConfig(seed=3))
    path=[[-.8,0.],[-.8,.6],[.8,.6],[.8,0.]]
    result=p.shortcut(path)
    assert result[0]==path[0] and result[-1]==path[-1]
    assert 2<len(result)<len(path)
    assert length(result)<length(path)
    for a,b in zip(result,result[1:]):
        for i in range(501):
            assert valid([x+(y-x)*i/500 for x,y in zip(a,b)])


def test_budget_and_invalid_arguments():
    p=JointRRTConnect([(-1,1)],lambda q:True)
    path=[[-1.],[0.],[1.]]
    assert p.shortcut(path,attempts=0)==path
    assert p.shortcut(path,timeout=1e-12)==path
    assert p.shortcut([[0.]])==[[0.]]
    with pytest.raises(ValueError):p.shortcut([])
    with pytest.raises(ValueError):p.shortcut([[2.]])
    with pytest.raises(ValueError):p.shortcut(path,timeout=0)
