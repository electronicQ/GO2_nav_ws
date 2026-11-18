"""State-machine checks using a fake client; no SDK transport or robot calls."""
import importlib.util
from pathlib import Path

import pytest
import rclpy
from geometry_msgs.msg import Twist
from rclpy.parameter import Parameter
from std_msgs.msg import Bool, String

spec = importlib.util.spec_from_file_location(
    'sdk_bridge', Path(__file__).parents[1] / 'scripts/sdk_twist_bridge.py')
bridge_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(bridge_module)


class FakeClient:
    def __init__(self):
        self.moves = []
        self.stops = 0

    def Move(self, *values):
        self.moves.append(values)
        return 0

    def StopMove(self):
        self.stops += 1
        return 0


@pytest.fixture
def bridge():
    rclpy.init()
    client = FakeClient()
    node = bridge_module.SdkVelocityBridge(client, parameter_overrides=[
        Parameter('require_localization_valid', value=True)])
    yield node, client
    node.destroy_node()
    rclpy.shutdown()


def command(node, speed=0.5):
    msg = Twist()
    msg.linear.x = speed
    node.receive(msg)


def localized(node):
    node.receive_valid(Bool(data=True))
    node.receive_state(String(data='GLOBAL_TRACKING'))


def test_initialization_and_idle_do_not_send_requests(bridge):
    node, client = bridge
    node.tick()
    command(node)
    node.tick()
    assert client.moves == [] and client.stops == 0
    localized(node)
    node.tick()
    assert client.moves == []  # command from a closed gate was discarded
    command(node)
    node.tick()
    assert client.moves == [(0.5, 0.0, 0.0)]


@pytest.mark.parametrize('failure', ['invalid', 'lost_state', 'valid_stale', 'state_stale'])
def test_localization_gate_stops_and_requires_fresh_command(bridge, failure):
    node, client = bridge
    localized(node)
    command(node)
    node.tick()
    if failure == 'invalid':
        node.receive_valid(Bool(data=False))
    elif failure == 'lost_state':
        node.receive_state(String(data='LOCAL_LOST'))
    elif failure == 'valid_stale':
        node.valid_received -= 2
    else:
        node.state_received -= 2
    node.tick()
    node.tick()
    assert client.stops == 1
    localized(node)
    node.tick()
    assert len(client.moves) == 1
    command(node)
    node.tick()
    assert len(client.moves) == 2


def test_command_timeout_stops_once(bridge):
    node, client = bridge
    localized(node)
    command(node)
    node.tick()
    node.received -= 1
    node.tick()
    node.tick()
    assert len(client.moves) == 1 and client.stops == 1


def test_zero_and_limits(bridge):
    node, client = bridge
    localized(node)
    msg = Twist()
    msg.linear.x, msg.angular.z = 0.8, -0.6
    node.receive(msg)
    node.tick()
    assert client.moves == [(0.5, 0.0, -0.3)]
    command(node, 0.0)
    node.tick()
    node.tick()
    assert client.stops == 1


def test_non_finite_command_is_rejected(bridge):
    node, client = bridge
    with pytest.raises(RuntimeError):
        command(node, float('nan'))
    assert client.moves == []
