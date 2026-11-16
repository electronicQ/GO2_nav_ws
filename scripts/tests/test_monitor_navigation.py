"""Timing attribution and Linux parsing tests for the read-only monitor."""
import importlib.util
from pathlib import Path
import unittest
import struct

spec = importlib.util.spec_from_file_location('monitor_navigation', Path(__file__).resolve().parents[1] / 'monitor_navigation.py')
monitor = importlib.util.module_from_spec(spec)
spec.loader.exec_module(monitor)


class MonitorTest(unittest.TestCase):
    def test_idle_and_new_goal_silence_is_not_attributed_to_active_control(self):
        stream = monitor.Stream()
        stream.receive(1, segment=1)
        stream.receive(1.067, segment=1)
        stream.receive(10, segment=None)  # waiting for a goal
        stream.receive(20, segment=2)
        stream.receive(20.067, segment=2)
        summary = stream.summary(22)
        self.assertEqual(summary['active_follow_path_intervals'], 2)
        self.assertAlmostEqual(summary['active_follow_path_max_gap_sec'], .067)
        self.assertEqual(summary['active_follow_path_gaps_over_200ms'], 0)
        self.assertAlmostEqual(summary['max_gap_sec_including_idle'], 10)
        self.assertAlmostEqual(summary['last_receive_age_sec_at_stop'], 1.933)

    def test_active_stall_and_zero_commands_remain_visible(self):
        stream = monitor.Stream()
        stream.receive(1, segment=1, value=[.2, 0, 0], nonzero=True)
        gap, active = stream.receive(4, segment=1, value=[0, 0, 0])
        self.assertEqual(gap, 3)
        self.assertTrue(active)
        summary = stream.summary(9)
        self.assertEqual(summary['received'], 2)
        self.assertEqual(summary['nonzero_received'], 1)
        self.assertEqual(summary['active_follow_path_gaps_over_200ms'], 1)
        self.assertEqual(summary['last_receive_age_sec_at_stop'], 5)
        self.assertEqual(summary['last_value'], [0, 0, 0])

    def test_proc_comm_parentheses_do_not_shift_cpu_or_memory_fields(self):
        # Fields 3..24, with a comm containing both spaces and a closing parenthesis.
        fields = ['S'] + ['0'] * 21
        fields[11], fields[12], fields[17], fields[19], fields[21] = '123', '45', '7', '98765', '200'
        stat = monitor.parse_proc_stat('42 (worker ) name) ' + ' '.join(fields))
        self.assertEqual(stat['ticks'], 168)
        self.assertEqual(stat['threads'], 7)
        self.assertEqual(stat['start'], 98765)
        self.assertEqual(stat['rss_pages'], 200)

    def test_raw_tf_headers_match_ros_serialization_with_variable_frame_lengths(self):
        try:
            from rclpy.serialization import serialize_message
            from tf2_msgs.msg import TFMessage
            from geometry_msgs.msg import TransformStamped
        except ImportError:
            self.skipTest('ROS Python messages unavailable')
        transforms = []
        for parent, child in [('odom', 'imu_link'), ('a', 'bb'), ('long_frame_name', 'child')]:
            t = TransformStamped()
            t.header.frame_id, t.child_frame_id = parent, child
            t.header.stamp.sec, t.header.stamp.nanosec = 123, 456
            transforms.append(t)
        raw = serialize_message(TFMessage(transforms=transforms))
        self.assertEqual(monitor.cdr_tf_headers(raw),
            [(t.header.frame_id, t.child_frame_id, 123.000000456) for t in transforms])
        with self.assertRaises((ValueError, struct.error)):
            monitor.cdr_tf_headers(raw[:20])

    def test_raw_odometry_timestamp_matches_ros_serialization(self):
        try:
            from rclpy.serialization import serialize_message
            from nav_msgs.msg import Odometry
        except ImportError:
            self.skipTest('ROS Python messages unavailable')
        msg = Odometry()
        msg.header.stamp.sec, msg.header.stamp.nanosec = 456, 789
        self.assertAlmostEqual(monitor.cdr_header_stamp(serialize_message(msg)), 456.000000789)


if __name__ == '__main__':
    unittest.main()
