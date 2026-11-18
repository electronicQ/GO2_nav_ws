#!/usr/bin/env python3
"""Publish a bounded +0.5 m/s Twist test; run the velocity bridge separately."""
import argparse
import math
import os
import signal
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
from unitree_api.msg import Request, Response
from unitree_go.msg import SportModeState


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=3.0,
                        help='Forward duration in seconds (default: 3).')
    parser.add_argument('--topic', default='/cmd_vel',
                        help='Twist output topic; use a diagnostic topic for bench verification.')
    args = parser.parse_args()
    if not math.isfinite(args.duration) or not 0 < args.duration <= 30:
        parser.error('--duration must be greater than zero and at most 30 seconds')

    rclpy.init()
    node = rclpy.create_node(f'cmd_vel_forward_test_{os.getpid()}')
    publisher = node.create_publisher(Twist, args.topic, 10)
    stopped = False
    sent = False
    requests = 0
    responses = {}
    speed = None

    def interrupt(_sig, _frame):
        nonlocal stopped
        stopped = True

    signal.signal(signal.SIGINT, interrupt)
    signal.signal(signal.SIGTERM, interrupt)

    def request(msg):
        nonlocal requests
        if msg.header.identity.api_id == 1008:
            requests += 1

    def response(msg):
        if msg.header.identity.api_id == 1008:
            code = msg.header.status.code
            responses[code] = responses.get(code, 0) + 1

    def state(msg):
        nonlocal speed
        speed = float(msg.velocity[0])

    subscriptions = [
        node.create_subscription(Request, '/api/sport/request', request, qos_profile_sensor_data),
        node.create_subscription(Response, '/api/sport/response', response, qos_profile_sensor_data),
        node.create_subscription(SportModeState, '/sportmodestate', state, qos_profile_sensor_data),
    ]

    def check_publishers():
        others = [info.node_namespace.rstrip('/') + '/' + info.node_name
                  for info in node.get_publishers_info_by_topic(args.topic)
                  if info.node_name != node.get_name() or
                  info.node_namespace != node.get_namespace()]
        if others:
            raise RuntimeError(f'Other publishers on {args.topic}: {others}; stop Nav2/smoother first')

    status = 0
    try:
        print(f'Waiting for a subscriber on {args.topic}; no velocity sent yet.', flush=True)
        deadline = time.monotonic() + 5.0
        while not stopped and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            # Allow discovery to settle before checking the graph.
            if time.monotonic() >= deadline - 4.0 and publisher.get_subscription_count():
                break
        if stopped:
            return 130
        if not publisher.get_subscription_count():
            raise RuntimeError('No subscriber discovered; start the control bridge in the same DDS environment')
        check_publishers()
        command = Twist()
        command.linear.x = 0.5
        print(f'Publishing {args.topic}: vx=0.5 m/s, vy=0, wz=0, '
              f'20 Hz for {args.duration:g} s. Ctrl+C sends zero.', flush=True)
        start = time.monotonic()
        tick = start
        report = start
        while not stopped and time.monotonic() - start < args.duration:
            now = time.monotonic()
            if now >= report:
                check_publishers()
                speed_text = 'unavailable' if speed is None else f'{speed:.3f} m/s'
                print(f't={now-start:.1f}s Move requests={requests}, '
                      f'response codes={responses}, robot vx={speed_text}', flush=True)
                report = now + 1.0
            publisher.publish(command)
            sent = True
            tick += 0.05
            while not stopped and time.monotonic() < tick:
                rclpy.spin_once(node, timeout_sec=min(0.01, max(0.0, tick-time.monotonic())))
    except Exception as exc:
        print(f'Test stopped: {exc}', flush=True)
        status = 1
    finally:
        if sent and rclpy.ok():
            # Keep the zero command alive longer than the bridge's 50 ms timer.
            zero = Twist()
            deadline = time.monotonic() + 1.0
            while time.monotonic() < deadline:
                publisher.publish(zero)
                rclpy.spin_once(node, timeout_sec=0.05)
            print(f'Zero velocity sent for 1 s. Move requests={requests}; '
                  f'response codes={responses}; robot vx={speed}', flush=True)
        node.destroy_node()
        rclpy.shutdown()
    return 130 if stopped and status == 0 else status


if __name__ == '__main__':
    raise SystemExit(main())
