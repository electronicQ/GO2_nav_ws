"""Repeat map-loaded idle SIGINT shutdown checks in an isolated ROS domain."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import time

from ament_index_python.packages import get_package_prefix, get_package_share_directory

from run_spatial_map_probe import stop


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--attempts', type=int, default=3)
    args = parser.parse_args()
    if not 1 <= args.attempts <= 20:
        parser.error('attempts must be between 1 and 20')
    args.map = args.map.resolve()
    args.output = args.output.resolve()
    if not (args.map / 'manifest.yaml').is_file() or args.map in args.output.parents:
        parser.error('Need an existing map and an output outside the map')
    args.output.mkdir(parents=True, exist_ok=False)
    os.environ['ROS_DOMAIN_ID'] = '98'
    os.environ['ROS_LOG_DIR'] = str(args.output / 'ros_logs')
    share = Path(get_package_share_directory('voxel_slam'))
    executable = Path(get_package_prefix('voxel_slam')) / 'lib/voxel_slam/voxelslam'
    results = []
    try:
        for attempt in range(args.attempts):
            log = args.output / f'attempt_{attempt + 1}.log'
            with log.open('w') as stream:
                process = subprocess.Popen([
                    str(executable), '--ros-args', '--params-file',
                    str(share / 'config/ros2_localization_mid360.yaml'),
                    '-p', 'Localization.map_path:=' + str(args.map),
                    '-p', 'Localization.visualization.publish_map_cloud:=false'],
                    stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
                try:
                    deadline = time.monotonic() + 60
                    while 'Loaded localization map package with' not in log.read_text():
                        if process.poll() is not None:
                            raise RuntimeError('Node exited before map initialization')
                        if time.monotonic() >= deadline:
                            raise TimeoutError('Map load timed out')
                        time.sleep(.1)
                    time.sleep(.2)
                finally:
                    started = time.monotonic()
                    stop(process)
                    elapsed = time.monotonic() - started
                result = dict(attempt=attempt + 1, exit_code=process.returncode,
                              shutdown_seconds=elapsed)
                results.append(result)
                print(result, flush=True)
                if process.returncode != 0:
                    raise RuntimeError('SIGINT shutdown was not clean')
    finally:
        (args.output / 'result.json').write_text(json.dumps(results, indent=2) + '\n')


if __name__ == '__main__':
    main()
