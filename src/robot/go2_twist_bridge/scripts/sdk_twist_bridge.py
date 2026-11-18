#!/usr/bin/env python3
"""Forward cmd_vel through official SportClient with localization and timeout gates."""
import math
import signal
import socket
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.sport.sport_client import SportClient


class SdkVelocityBridge(Node):
    def __init__(self, client, **kwargs):
        super().__init__('unitree_sdk_cmd_vel_bridge', **kwargs)
        self.client = client
        self.require_localization = self.declare_parameter(
            'require_localization_valid', False).value
        self.localization_timeout = self.declare_parameter(
            'localization_timeout_sec', 1.0).value
        self.command_timeout = self.declare_parameter('cmd_vel_timeout_sec', 0.5).value
        self.stop_on_gate_close = self.declare_parameter('stop_on_gate_close', True).value
        self.max_linear = self.declare_parameter('max_linear_x', 0.5).value
        self.max_angular = self.declare_parameter('max_angular_z', 0.3).value
        states = self.declare_parameter(
            'required_localization_states', 'IN_MAP_TRACKING,GLOBAL_TRACKING').value
        self.allowed_states = {state.strip() for state in states.split(',') if state.strip()}
        valid_topic = self.declare_parameter(
            'localization_valid_topic', '/localization/valid').value
        state_topic = self.declare_parameter(
            'localization_state_topic', '/localization/state').value
        if not all(math.isfinite(v) and v > 0.0 for v in (
                self.localization_timeout, self.command_timeout,
                self.max_linear, self.max_angular)):
            raise ValueError('Timeouts and velocity limits must be finite and positive')
        self.valid = False
        self.state = ''
        self.valid_received = None
        self.state_received = None
        self.gate_open = None
        self.command = (0.0, 0.0, 0.0)
        self.received = 0.0
        self.needs_stop = False
        self.calls = 0
        self.report = 0.0
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.receive, 10)
        if self.require_localization:
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.valid_subscription = self.create_subscription(Bool, valid_topic, self.receive_valid, qos)
            self.state_subscription = self.create_subscription(String, state_topic, self.receive_state, qos)
        self.timer = self.create_timer(0.05, self.tick)
        self.get_logger().info(
            'Official SportClient ready. Idle: no requests. '
            f'Limits: |vx|<={self.max_linear}, vy=0, |wz|<={self.max_angular}; '
            f'timeout={self.command_timeout}s; stop=StopMove(1003); '
            f'localization gate={self.require_localization}.')

    def receive_valid(self, msg):
        self.valid = msg.data
        self.valid_received = time.monotonic()

    def receive_state(self, msg):
        self.state = msg.data
        self.state_received = time.monotonic()

    def localization_ready(self, now):
        if not self.require_localization:
            return True
        return (self.valid and self.state in self.allowed_states
                and self.valid_received is not None and self.state_received is not None
                and now - self.valid_received <= self.localization_timeout
                and now - self.state_received <= self.localization_timeout)

    def receive(self, msg):
        values = (msg.linear.x, msg.linear.y, msg.angular.z)
        unused = (msg.linear.z, msg.angular.x, msg.angular.y)
        if (not all(math.isfinite(v) for v in values + unused)
                or values[1] != 0.0 or any(v != 0.0 for v in unused)):
            self.command = (0.0, 0.0, 0.0)
            raise RuntimeError('Non-finite or unsupported cmd_vel components; stopping bridge')
        self.command = (
            max(-self.max_linear, min(self.max_linear, values[0])), 0.0,
            max(-self.max_angular, min(self.max_angular, values[2])))
        self.received = time.monotonic()

    def stop(self):
        if not self.needs_stop:
            return
        # Retry a failed stop, but never repeatedly send posture commands while idle.
        for _ in range(3):
            code = self.client.StopMove()
            self.get_logger().info(f'StopMove returned {code}')
            if code == 0:
                self.needs_stop = False
                return
        raise RuntimeError('StopMove failed; use the physical remote to stop the robot')

    def tick(self):
        now = time.monotonic()
        ready = self.localization_ready(now)
        if ready != self.gate_open:
            self.gate_open = ready
            self.get_logger().info('Localization gate ' + ('opened' if ready else 'closed'))
        if not ready:
            if self.stop_on_gate_close:
                self.stop()
            # A reopened gate must receive a fresh command before moving again.
            self.received = 0.0
            self.command = (0.0, 0.0, 0.0)
            return
        if now - self.received > self.command_timeout or not any(self.command):
            self.stop()
            return
        self.needs_stop = True
        code = self.client.Move(*self.command)
        self.calls += 1
        if code != 0:
            raise RuntimeError(f'Official SportClient.Move returned {code}; stopping test')
        if now >= self.report:
            self.get_logger().info(
                f'Move{self.command}: send_code={code}, calls={self.calls} '
                '(official Move uses no-reply mode)')
            self.report = now + 1.0


def main():
    import os
    interface = os.environ.get('GO2_NETWORK_INTERFACE')
    if not interface:
        raise SystemExit('Set GO2_NETWORK_INTERFACE to the robot Ethernet interface first')
    interfaces = [name for _, name in socket.if_nameindex()]
    if interface not in interfaces:
        raise SystemExit(
            f'GO2 network interface {interface!r} is missing. '
            f'Available interfaces: {", ".join(interfaces)}. '
            'Reconnect the robot USB Ethernet adapter or set GO2_NETWORK_INTERFACE '
            'to its current name, then source scripts/setup_unitree_official.sh again. '
            'No robot requests were sent.')
    if os.environ.get('ROS_LOCALHOST_ONLY') == '1':
        raise SystemExit(
            'ROS_LOCALHOST_ONLY=1 blocks robot network discovery. '
            'Run: unset ROS_LOCALHOST_ONLY; then source scripts/setup_unitree_official.sh again.')
    domain = int(os.environ.get('ROS_DOMAIN_ID', '0'))
    rclpy.init()
    # Discovery only: no robot requests are sent during this check.
    probe = rclpy.create_node('unitree_sdk_bridge_startup_check', use_global_arguments=False)
    deadline = time.monotonic() + 1.5
    while time.monotonic() < deadline:
        rclpy.spin_once(probe, timeout_sec=0.05)
    names = probe.get_node_names()
    probe.destroy_node()
    if 'unitree_sdk_cmd_vel_bridge' in names:
        rclpy.shutdown()
        raise SystemExit('Another velocity bridge is running. Stop it and Nav2 before this test.')
    ChannelFactoryInitialize(domain, interface)
    client = SportClient()
    client.SetTimeout(0.2)
    client.Init()
    node = SdkVelocityBridge(client)
    interrupted = False

    def interrupt(_signum, _frame):
        nonlocal interrupted
        interrupted = True

    signal.signal(signal.SIGINT, interrupt)
    signal.signal(signal.SIGTERM, interrupt)
    status = 0
    try:
        while not interrupted and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
    except Exception as exc:
        node.get_logger().error(str(exc))
        status = 1
    finally:
        try:
            node.stop()
        except Exception as exc:
            node.get_logger().error(str(exc))
            status = 1
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return status


if __name__ == '__main__':
    raise SystemExit(main())
