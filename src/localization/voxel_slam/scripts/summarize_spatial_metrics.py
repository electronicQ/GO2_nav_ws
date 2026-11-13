"""Summarize phase-1 map-supply measurements, without treating them as ground truth."""
import argparse
import csv
import json
from pathlib import Path


def distribution(values):
    ordered = sorted(values)
    if not ordered:
        return None
    def quantile(p):
        index = (len(ordered) - 1) * p
        low = int(index)
        high = min(low + 1, len(ordered) - 1)
        return ordered[low] + (ordered[high] - ordered[low]) * (index - low)
    return dict(mean=sum(ordered) / len(ordered), p50=quantile(.5),
                p95=quantile(.95), p99=quantile(.99), maximum=ordered[-1])


def summarize(directory):
    with (directory / 'frames.csv').open(newline='') as stream:
        frames = list(csv.DictReader(stream))
    with (directory / 'block_events.csv').open(newline='') as stream:
        events = list(csv.DictReader(stream))
    terminal = [r for r in events if r['event'] in
                ('published', 'failed', 'discarded', 'cancelled_active')]
    ids = [r['request_id'] for r in terminal]
    if len(ids) != len(set(ids)):
        raise ValueError('Duplicate terminal request IDs; cannot sum reads safely')
    admitted = {r['request_id'] for r in events if r['event'] == 'admitted'}
    queries = sum(int(r['query_points']) for r in frames)
    missing = sum(int(r['missing_block_points']) for r in frames)
    result = dict(
        recorded_map_phase_frames=len(frames),
        valid_recorded_frames=sum(r['valid'] == '1' for r in frames),
        core_not_ready_frames=sum(r['core_ready'] != '1' for r in frames),
        # This depends on the online estimate; it is not reference demand coverage.
        online_missing_block_point_fraction=missing / queries if queries else None,
        completed_load_attempts=len(terminal),
        unfinished_admitted_requests=len(admitted - set(ids)),
        failed_load_attempts=sum(r['event'] == 'failed' for r in terminal),
        logical_file_read_bytes=sum(int(r['read_bytes']) for r in terminal),
        states=list(dict.fromkeys(r['state'] for r in frames)))
    for key in ('schedule_ms', 'match_ms', 'frame_compute_ms', 'live_block_bytes',
                'pinned_block_bytes', 'reserved_bytes', 'peak_accounted_bytes', 'rss_bytes'):
        result[key] = distribution([float(r[key]) for r in frames])
    result['source_service_ms'] = distribution([float(r['service_ms']) for r in terminal])
    result['decode_ms'] = distribution([float(r['decode_ms']) for r in terminal])
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    args = parser.parse_args()
    print(json.dumps(summarize(args.directory), indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
