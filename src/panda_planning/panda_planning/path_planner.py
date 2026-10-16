#!/usr/bin/env python3
"""关节空间 RRT 路径规划模块，供 planner_node.py 导入。

输入：起点、终点、关节限位，以及真实的状态碰撞检测函数。
输出：关节路径 list[list[float]]；搜索超时/未找到路径返回 None。
不负责逆运动学、轨迹时间分配或发布 ROS 指令。

is_state_valid(q) 必须检查整条机械臂的自碰撞和环境碰撞。
边按 resolution（关节空间欧氏距离，rad）离散检查，不是连续碰撞证明。
"""
import math
import random
import time
from dataclasses import dataclass
from typing import Callable, List, Optional, Sequence, Tuple

Configuration = Tuple[float, ...]


@dataclass(frozen=True)
class RRTConfig:
    step_size: float = 0.15
    resolution: float = 0.02
    goal_bias: float = 0.15
    max_iterations: int = 5000
    timeout: float = 5.0
    seed: Optional[int] = None

    def __post_init__(self):
        for name in ('step_size', 'resolution', 'timeout'):
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f'{name} 必须是有限正数')
        if not math.isfinite(self.goal_bias) or not 0 <= self.goal_bias <= 1:
            raise ValueError('goal_bias 必须在 [0, 1] 内')
        if (isinstance(self.max_iterations, bool)
                or not isinstance(self.max_iterations, int)
                or self.max_iterations < 1):
            raise ValueError('max_iterations 必须是正整数')


class JointRRT:
    """使用有界关节空间中的欧氏距离，不处理连续旋转关节的角度回绕。"""

    def __init__(
        self,
        joint_limits: Sequence[Sequence[float]],
        is_state_valid: Callable[[Configuration], bool],
        config: Optional[RRTConfig] = None,
    ):
        if not callable(is_state_valid):
            raise ValueError('必须提供真实的 is_state_valid 碰撞检测函数')
        limits = []
        for pair in joint_limits:
            if len(pair) != 2:
                raise ValueError('每个关节限位必须是 (lower, upper)')
            lower, upper = map(float, pair)
            if not math.isfinite(lower) or not math.isfinite(upper) or lower >= upper:
                raise ValueError('关节限位必须有限且 lower < upper')
            limits.append((lower, upper))
        if not limits:
            raise ValueError('至少需要一个关节')
        self.limits = tuple(limits)
        self.is_state_valid = is_state_valid
        self.config = config or RRTConfig()
        self.random = random.Random(self.config.seed)

    def _configuration(self, values: Sequence[float]) -> Configuration:
        result = tuple(float(value) for value in values)
        if len(result) != len(self.limits) or not all(map(math.isfinite, result)):
            raise ValueError('关节位置维数错误或包含非有限数值')
        return result

    def _valid(self, q: Configuration) -> bool:
        return (all(lower <= value <= upper
                    for value, (lower, upper) in zip(q, self.limits))
                and bool(self.is_state_valid(q)))

    @staticmethod
    def _distance(a: Configuration, b: Configuration) -> float:
        return math.sqrt(sum((x - y)**2 for x, y in zip(a, b)))

    def _edge_valid(self, a: Configuration, b: Configuration, deadline: float) -> bool:
        steps = max(1, math.ceil(self._distance(a, b) / self.config.resolution))
        for i in range(steps + 1):
            if time.monotonic() >= deadline:
                raise TimeoutError
            fraction = i / steps
            q = tuple(x + fraction * (y - x) for x, y in zip(a, b))
            if not self._valid(q):
                return False
        return True

    @staticmethod
    def _recover(nodes, parents, index) -> List[List[float]]:
        path = []
        while index is not None:
            path.append(list(nodes[index]))
            index = parents[index]
        return list(reversed(path))

    def shortcut(self, path, attempts=150, timeout=2.0):
        """Remove intermediate vertices only after full discretized edge checks.

        Input must be an already collision-checked path (e.g. plan output).
        Keeps endpoints, adds no waypoints and never lengthens the joint path.
        Timeout preserves all previously validated improvements.
        """
        if isinstance(attempts, bool) or not isinstance(attempts, int) or attempts < 0:
            raise ValueError('shortcut attempts 必须为非负整数')
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError('shortcut timeout 必须为有限正数')
        result = [self._configuration(q) for q in path]
        if not result:
            raise ValueError('shortcut 路径不能为空')
        if not all(self._valid(q) for q in result):
            raise ValueError('shortcut 输入含非法状态')
        deadline = time.monotonic()+timeout
        for attempt in range(attempts):
            if len(result) < 3 or time.monotonic() >= deadline:
                break
            if attempt == 0:
                i,j = 0,len(result)-1
            else:
                i,j = sorted(self.random.sample(range(len(result)),2))
            if j-i < 2:
                continue
            before = sum(self._distance(a,b) for a,b in zip(result[i:j],result[i+1:j+1]))
            if self._distance(result[i],result[j]) > before+1e-12:
                continue
            try:
                valid = self._edge_valid(result[i],result[j],deadline)
            except TimeoutError:
                break
            if valid:
                result[i+1:j] = []
        return [list(q) for q in result]

    def plan(self, start: Sequence[float], goal: Sequence[float]) -> Optional[List[List[float]]]:
        """非法起终点抛出 ValueError；搜索失败返回 None；检测器异常向上传递。"""
        start, goal = self._configuration(start), self._configuration(goal)
        if not self._valid(start):
            raise ValueError('起点超出限位或处于碰撞状态')
        if not self._valid(goal):
            raise ValueError('终点超出限位或处于碰撞状态')
        if start == goal:
            return [list(start)]
        deadline = time.monotonic() + self.config.timeout
        try:
            if self._edge_valid(start, goal, deadline):
                return [list(start), list(goal)]
            nodes, parents = [start], [None]
            for _ in range(self.config.max_iterations):
                if time.monotonic() >= deadline:
                    return None
                sample = (goal if self.random.random() < self.config.goal_bias else
                          tuple(self.random.uniform(lo, hi) for lo, hi in self.limits))
                nearest = min(range(len(nodes)),
                              key=lambda i: self._distance(nodes[i], sample))
                distance = self._distance(nodes[nearest], sample)
                if distance <= 1e-12:
                    continue
                scale = min(1., self.config.step_size / distance)
                candidate = tuple(x + scale * (y - x)
                                  for x, y in zip(nodes[nearest], sample))
                if not self._edge_valid(nodes[nearest], candidate, deadline):
                    continue
                nodes.append(candidate)
                parents.append(nearest)
                index = len(nodes) - 1
                if self._distance(candidate, goal) <= self.config.step_size:
                    if self._edge_valid(candidate, goal, deadline):
                        path = self._recover(nodes, parents, index)
                        if candidate != goal:
                            path.append(list(goal))
                        return path
            return None
        except TimeoutError:
            return None


class JointRRTConnect(JointRRT):
    """Bidirectional RRT-Connect: EXTEND once, CONNECT greedily, swap trees.

    Uses the same bounded joint metric, discretized edge checks and inputs
    as JointRRT. It adds no hand-authored waypoints or path smoothing.
    """

    def __init__(self, joint_limits, is_state_valid, config=None):
        super().__init__(joint_limits, is_state_valid, config)
        self.last_status = 'not_started'

    def _extend(self, nodes, parents, target, deadline):
        if time.monotonic() >= deadline:
            raise TimeoutError
        nearest = min(range(len(nodes)),
                      key=lambda i: self._distance(nodes[i], target))
        distance = self._distance(nodes[nearest], target)
        if distance <= 1e-12:
            return 'reached', nearest
        if distance <= self.config.step_size:
            candidate = target
        else:
            scale = self.config.step_size / distance
            candidate = tuple(x+scale*(y-x) for x,y in zip(nodes[nearest],target))
        if not self._edge_valid(nodes[nearest], candidate, deadline):
            return 'trapped', nearest
        nodes.append(candidate)
        parents.append(nearest)
        return ('reached' if candidate == target else 'advanced'), len(nodes)-1

    def plan(self, start, goal):
        start, goal = self._configuration(start), self._configuration(goal)
        self.last_status = 'invalid_endpoint'
        if not self._valid(start):
            raise ValueError('起点超出限位或处于碰撞状态')
        if not self._valid(goal):
            raise ValueError('终点超出限位或处于碰撞状态')
        if start == goal:
            self.last_status = 'success'
            return [list(start)]
        deadline = time.monotonic()+self.config.timeout
        try:
            if self._edge_valid(start,goal,deadline):
                self.last_status = 'success'
                return [list(start),list(goal)]
            a, pa = [start], [None]
            b, pb = [goal], [None]
            a_from_start = True
            for _ in range(self.config.max_iterations):
                if time.monotonic() >= deadline:
                    raise TimeoutError
                sample = (b[0] if self.random.random() < self.config.goal_bias else
                          tuple(self.random.uniform(lo,hi) for lo,hi in self.limits))
                status, ia = self._extend(a,pa,sample,deadline)
                if status != 'trapped':
                    while True:
                        connected, ib = self._extend(b,pb,a[ia],deadline)
                        if connected != 'advanced':
                            break
                    if connected == 'reached':
                        left, right = self._recover(a,pa,ia), self._recover(b,pb,ib)
                        if not a_from_start:
                            left, right = right, left
                        self.last_status = 'success'
                        return left+right[-2::-1]
                a,b = b,a
                pa,pb = pb,pa
                a_from_start = not a_from_start
            self.last_status = 'iteration_limit'
            return None
        except TimeoutError:
            self.last_status = 'timeout'
            return None
