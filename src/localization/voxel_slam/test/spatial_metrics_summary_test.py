"""Do not double-count periodically repeated loader counters as I/O events."""
import csv
import importlib.util
from pathlib import Path
import tempfile
import unittest


path = Path(__file__).resolve().parents[1] / 'scripts/summarize_spatial_metrics.py'
spec = importlib.util.spec_from_file_location('spatial_summary', path)
summary = importlib.util.module_from_spec(spec)
spec.loader.exec_module(summary)


class SpatialMetricsSummaryTest(unittest.TestCase):
    def fixture(self, directory, duplicate=False):
        row = dict(state='IN_MAP_TRACKING', valid='1', core_ready='1', query_points='10',
                   missing_block_points='2', schedule_ms='1', match_ms='2', frame_compute_ms='3',
                   live_block_bytes='100', pinned_block_bytes='0', reserved_bytes='0',
                   peak_accounted_bytes='120', rss_bytes='1000', read_bytes='200')
        with (directory / 'frames.csv').open('w', newline='') as stream:
            writer = csv.DictWriter(stream, fieldnames=list(row))
            writer.writeheader()
            writer.writerows([row, row, row])
        with (directory / 'block_events.csv').open('w', newline='') as stream:
            writer = csv.DictWriter(stream, fieldnames=['event', 'request_id', 'read_bytes',
                                                       'service_ms', 'decode_ms'])
            writer.writeheader()
            writer.writerow(dict(event='admitted', request_id='1', read_bytes='0',
                                 service_ms='0', decode_ms='0'))
            terminal = dict(event='published', request_id='1', read_bytes='200',
                            service_ms='2', decode_ms='1')
            writer.writerow(terminal)
            if duplicate: writer.writerow(terminal)

    def test_repeated_frame_counters_do_not_multiply_bytes(self):
        with tempfile.TemporaryDirectory() as temp:
            directory = Path(temp)
            self.fixture(directory)
            result = summary.summarize(directory)
            self.assertEqual(result['logical_file_read_bytes'], 200)
            self.assertEqual(result['completed_load_attempts'], 1)
            self.assertEqual(result['recorded_map_phase_frames'], 3)
            self.assertEqual(result['unfinished_admitted_requests'], 0)
            self.assertAlmostEqual(result['online_missing_block_point_fraction'], .2)
            self.assertEqual(result['frame_compute_ms']['p99'], 3)

    def test_duplicate_terminal_ids_are_rejected(self):
        with tempfile.TemporaryDirectory() as temp:
            directory = Path(temp)
            self.fixture(directory, duplicate=True)
            with self.assertRaisesRegex(ValueError, 'Duplicate terminal'):
                summary.summarize(directory)


if __name__ == '__main__':
    unittest.main()
