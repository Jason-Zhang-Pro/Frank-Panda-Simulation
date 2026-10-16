#!/usr/bin/env python3
"""Activate controllers using bounded paused steps, then start simulation."""
import argparse
import math
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from controller_manager_msgs.srv import ListControllers, SwitchController
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

NAMES = [f'panda_joint{i}' for i in range(1, 8)]
HOME = [0., -.785398, 0., -2.356194, 0., 1.570796, .785398]
CONTROLLERS = ['joint_state_broadcaster', 'panda_effort_controller', 'panda_gripper_controller']


def world_control(world, request):
    result = subprocess.run([
        'ign', 'service', '-s', f'/world/{world}/control',
        '--reqtype', 'ignition.msgs.WorldControl',
        '--reptype', 'ignition.msgs.Boolean', '--timeout', '2000',
        '--req', request,
    ], capture_output=True, text=True, timeout=4)
    if result.returncode != 0 or 'data: true' not in result.stdout:
        raise RuntimeError('Gazebo control request failed: ' + result.stdout + result.stderr)


class Startup(Node):
    def __init__(self):
        super().__init__('panda_startup')
        self.q = self.dq = None
        self.torque = None
        self.torque_time = 0.
        self.create_subscription(JointState, '/joint_states', self.state, qos_profile_sensor_data)
        self.create_subscription(Float64MultiArray, '/panda_effort_controller/commands', self.command, 10)
        self.list_client = self.create_client(ListControllers, '/controller_manager/list_controllers')
        self.switch_client = self.create_client(SwitchController, '/controller_manager/switch_controller')

    def state(self, msg):
        indices = {name: i for i, name in enumerate(msg.name)}
        try:
            q = [msg.position[indices[name]] for name in NAMES]
            dq = [msg.velocity[indices[name]] for name in NAMES]
            if all(math.isfinite(x) for x in q + dq):
                self.q, self.dq = q, dq
        except (KeyError, IndexError):
            pass

    def command(self, msg):
        if len(msg.data) == 7 and all(math.isfinite(x) for x in msg.data):
            self.torque = list(msg.data)
            self.torque_time = time.monotonic()

    def spin_for(self, seconds):
        deadline = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=min(.01, max(0., deadline-time.monotonic())))

    def wait_future(self, future, timeout=10.):
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=.02)
        if not future.done():
            raise RuntimeError('ROS service timed out')
        return future.result()

    def run(self, world, auto_play):
        world_control(world, 'pause: true')
        if not self.list_client.wait_for_service(timeout_sec=15.):
            raise RuntimeError('controller_manager unavailable')
        response = self.wait_future(self.list_client.call_async(ListControllers.Request()))
        states = {c.name: c.state for c in response.controller}
        if any(states.get(name) != 'inactive' for name in CONTROLLERS):
            raise RuntimeError(f'Expected freshly configured inactive controllers; got {states}')
        deadline = time.monotonic() + 15.
        while self.count_publishers('/panda_effort_controller/commands') == 0 and time.monotonic() < deadline:
            self.spin_for(.05)
        if self.count_publishers('/panda_effort_controller/commands') != 1:
            raise RuntimeError('Exactly one torque control script must be running')
        if not self.switch_client.wait_for_service(timeout_sec=10.):
            raise RuntimeError('switch_controller unavailable')
        request = SwitchController.Request()
        request.activate_controllers = CONTROLLERS
        request.strictness = SwitchController.Request.STRICT
        request.timeout.sec = 30
        future = self.switch_client.call_async(request)
        self.get_logger().info('自动激活控制器，保持暂停并逐步推进仿真…')
        # Each request advances only 1 ms in this world's 1 ms physics configuration.
        for _ in range(100):
            self.spin_for(.02)
            if future.done():
                break
            world_control(world, 'pause: true, multi_step: 1')
        if not future.done() or not future.result().ok:
            raise RuntimeError('Controller activation failed or exceeded 100 paused steps')
        self.torque = None
        for _ in range(100):
            world_control(world, 'pause: true, multi_step: 1')
            self.spin_for(.04)
            if self.q is not None and max(abs(a-b) for a,b in zip(self.q, HOME)) > .05:
                raise RuntimeError('Panda moved away from initial posture during startup')
            if (self.q is not None and self.torque is not None
                    and time.monotonic()-self.torque_time < .15
                    and max(abs(t) for t in self.torque) > .01
                    and max(abs(v) for v in self.dq) < .25):
                break
        else:
            raise RuntimeError('No valid feedback and nonzero holding torque received')
        response = self.wait_future(self.list_client.call_async(ListControllers.Request()))
        states = {c.name: c.state for c in response.controller}
        if any(states.get(name) != 'active' for name in CONTROLLERS):
            raise RuntimeError(f'Controllers not active: {states}')
        if auto_play:
            world_control(world, 'pause: false')
            self.get_logger().info('启动完成：三个控制器已激活，反馈和保持力矩正常，仿真已播放。')
        else:
            self.get_logger().info('启动完成：三个控制器已激活，仿真保持暂停，可点击播放。')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--world', default='empty')
    parser.add_argument('--auto-play', choices=['true', 'false'], default='true')
    args = parser.parse_args()
    rclpy.init()
    node = Startup()
    try:
        node.run(args.world, args.auto_play == 'true')
    except Exception as exc:
        node.get_logger().error(f'启动失败，停止自动播放：{exc}')
        try:
            world_control(args.world, 'pause: true')
        except Exception:
            pass
        raise
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
