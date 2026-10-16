"""Return home is a planned stage; success requires measured arrival."""
import time
from types import SimpleNamespace
from panda_planning.grasp_task import GraspTask

HOME = [0., -.785398, 0., -2.356194, 0., 1.570796, .785398]


class FakeNode:
    def __init__(self):
        self.sim_time = 100.
        self.calls = []
        self.messages = []

    def get_parameter(self, name):
        assert name == 'home_joint_positions'
        return SimpleNamespace(value=HOME.copy())

    def request(self, kind, goal, internal=False):
        self.calls.append((kind, goal, internal))

    def status(self, text):
        self.messages.append(text)


def task_after_retreat():
    task = GraspTask.__new__(GraspTask)
    task.node = FakeNode()
    task.active = True
    task.stage = 'retreating'
    task.arm_done = True
    task.failure = None
    task.wall_started = time.monotonic()
    task.target_time = 100.
    task.target_state = ([.32,-.55,.35],[0.,0.,0.,1.])
    task.place = [.32,-.55,.35]
    task.payload = None
    return task


def test_home_is_planned_before_success():
    task = task_after_retreat()
    task.poll()
    assert task.stage == 'homing' and task.active and not task.arm_done
    assert task.node.calls == [('joint', HOME, True)]
    assert not any(s.startswith('pick_place_succeeded:') for s in task.node.messages)
    task.poll()
    assert task.active, 'Do not complete merely because planning was requested'
    task.arm_done = True
    task.poll()
    assert not task.active
    assert task.node.messages[-1].startswith('pick_place_succeeded:')


def test_home_failure_is_distinguished_from_place_failure():
    task = task_after_retreat()
    task.poll()
    task.failure = 'failed: RRT 搜索超时或未找到路径'
    task.poll()
    assert not task.active
    assert task.node.messages[-1].startswith('pick_place_home_failed:')
    assert not any(s.startswith('pick_place_succeeded:') for s in task.node.messages)
