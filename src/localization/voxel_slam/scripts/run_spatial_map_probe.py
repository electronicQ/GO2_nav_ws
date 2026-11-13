"""Bounded isolated replay of the unchanged dual-sphere map-loading baseline."""
import argparse
import hashlib
import json
import os
import platform
from pathlib import Path
import signal
import subprocess
import time


def digest_file(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def stop(process):
    if process is None or process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=10)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map', required=True, type=Path)
    parser.add_argument('--bag', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--seconds', type=float, default=60)
    parser.add_argument('--memory-mb', type=float, default=0.0)
    parser.add_argument('--io-rate-mb', type=float, default=0.0)
    parser.add_argument('--io-latency-ms', type=float, default=0.0)
    parser.add_argument('--domain-id', type=int, default=96)
    parser.add_argument('--config', type=Path)
    parser.add_argument('--hash-bag', action='store_true')
    args = parser.parse_args()
    import math
    if (not 0 <= args.domain_id <= 101 or not math.isfinite(args.seconds) or
            args.seconds <= 0 or args.seconds > 3600 or
            any(not math.isfinite(v) or v < 0 for v in
                (args.memory_mb, args.io_rate_mb, args.io_latency_ms))):
        parser.error('Invalid domain, duration or resource limit')
    args.map, args.bag, args.output = args.map.resolve(), args.bag.resolve(), args.output.resolve()
    if args.map in args.output.parents or args.bag in args.output.parents:
        parser.error('Experiment output must not be inside the read-only map or input bag')
    if not (args.map / 'manifest.yaml').is_file() or not (args.bag / 'metadata.yaml').is_file():
        parser.error('Map and bag must contain their manifests')
    args.output.mkdir(parents=True, exist_ok=False)
    os.environ['ROS_DOMAIN_ID'] = str(args.domain_id)
    os.environ['ROS_LOG_DIR'] = str(args.output / 'ros_logs')

    from ament_index_python.packages import get_package_prefix, get_package_share_directory
    import rclpy
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from rosbag2_interfaces.srv import Resume
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import PointCloud2
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Bool, String
    import yaml
    from summarize_spatial_metrics import summarize

    share = Path(get_package_share_directory('voxel_slam'))
    config = args.config or share / 'config/ros2_localization_mid360.yaml'
    params = yaml.safe_load(config.read_text())
    values = params['/**']['ros__parameters']
    values['General'].update(mode='localization', is_save_map=0, map_export_path='')
    values['Localization']['map_path'] = str(args.map)
    values['Localization']['visualization'] = dict(publish_map_cloud=False, leaf_size=.5)
    values['Localization']['spatial_map'].update(
        metrics_directory=str(args.output / 'metrics'),
        max_block_memory_mb=args.memory_mb, io_rate_mb_per_sec=args.io_rate_mb,
        io_fixed_latency_ms=args.io_latency_ms)
    effective_config = args.output / 'config.yaml'
    effective_config.write_text(yaml.safe_dump(params, sort_keys=False))
    before = {str(p.relative_to(args.map)): digest_file(p)
              for p in sorted(args.map.rglob('*')) if p.is_file()}
    repo = Path(__file__).resolve().parents[2]
    revision = subprocess.check_output(['git', '-C', str(repo), 'rev-parse', 'HEAD'], text=True).strip()
    patch = subprocess.check_output(['git', '-C', str(repo), 'diff', 'HEAD', '--binary'])
    untracked = subprocess.check_output(
        ['git', '-C', str(repo), 'ls-files', '--others', '--exclude-standard'], text=True).splitlines()
    (args.output / 'source_changes.patch').write_bytes(patch)
    for name in untracked:
        source = repo / name
        if source.is_file():
            target = args.output / 'source_untracked' / name
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(source.read_bytes())
    bag_files = {str(p.relative_to(args.bag)): dict(size=p.stat().st_size,
                 sha256=digest_file(p) if args.hash_bag or p.suffix == '.yaml' else None)
                 for p in sorted(args.bag.iterdir()) if p.is_file()}
    manifest = dict(git_commit=revision, dirty_patch_sha256=hashlib.sha256(patch).hexdigest(),
                    untracked_source_sha256={p: digest_file(repo / p) for p in untracked
                                             if (repo / p).is_file()},
                    map_files_sha256=before, bag_files=bag_files,
                    config_sha256=digest_file(effective_config),
                    domain=args.domain_id, seconds=args.seconds,
                    platform=platform.platform(), cpu_count=os.cpu_count(),
                    policy='existing_core_first_distance_sorted_dual_sphere',
                    cache_mode='uncontrolled_os_cache',
                    limits=dict(memory_mib=args.memory_mb, io_mib_per_sec=args.io_rate_mb,
                                fixed_latency_ms=args.io_latency_ms),
                    full_bag_hash=args.hash_bag)
    (args.output / 'run_manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    rclpy.init()
    node = rclpy.create_node('spatial_map_probe_observer')
    result = dict(scans=0, valid=False, states=[])
    trajectory = (args.output / 'trajectory_imu.tum').open('w')
    state_events = (args.output / 'observer_events.jsonl').open('w')
    playback_started = None
    latest_bag_time = None

    def record_event(kind, value):
        now = time.monotonic()
        state_events.write(json.dumps(dict(kind=kind, value=value, steady_sec=now,
                           playback_elapsed_sec=now - playback_started if playback_started is not None else None,
                           latest_clock_sec=latest_bag_time)) + '\n')
        state_events.flush()

    def on_clock(msg):
        nonlocal latest_bag_time
        latest_bag_time = msg.clock.sec + msg.clock.nanosec * 1e-9

    def on_scan(msg):
        if msg.width * msg.height: result['scans'] += 1

    def on_state(msg):
        if not result['states'] or result['states'][-1] != msg.data:
            result['states'].append(msg.data)
            record_event('state', msg.data)

    def on_valid(msg):
        if msg.data != result['valid']:
            record_event('valid', msg.data)
        result['valid'] = msg.data

    def on_odom(msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        trajectory.write(f'{stamp:.9f} {p.x:.12g} {p.y:.12g} {p.z:.12g} '
                         f'{q.x:.12g} {q.y:.12g} {q.z:.12g} {q.w:.12g}\n')

    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
    subscriptions = [node.create_subscription(PointCloud2, '/map_scan', on_scan, 10),
                     node.create_subscription(String, '/localization/state', on_state, qos),
                     node.create_subscription(Bool, '/localization/valid', on_valid, qos),
                     node.create_subscription(Odometry, '/LIO/odom_imu', on_odom, 10000),
                     node.create_subscription(Clock, '/clock', on_clock,
                         QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))]
    slam = player = None
    with (args.output / 'slam.log').open('w') as slog, (args.output / 'play.log').open('w') as plog:
        try:
            executable = Path(get_package_prefix('voxel_slam')) / 'lib/voxel_slam/voxelslam'
            slam = subprocess.Popen([str(executable), '--ros-args', '--params-file', str(effective_config)],
                                    stdout=slog, stderr=subprocess.STDOUT, start_new_session=True)
            deadline = time.monotonic() + 60
            while 'Loaded localization map package with' not in (args.output / 'slam.log').read_text():
                if slam.poll() is not None: raise RuntimeError('SLAM exited during map loading')
                if time.monotonic() >= deadline: raise TimeoutError('Map load timed out')
                time.sleep(.2)
            player = subprocess.Popen(['ros2', 'bag', 'play', str(args.bag), '--clock',
                                       '--start-paused', '--topics', '/livox/lidar', '/livox/imu'],
                                      stdout=plog, stderr=subprocess.STDOUT, start_new_session=True)
            client = node.create_client(Resume, '/rosbag2_player/resume')
            if not client.wait_for_service(timeout_sec=15): raise TimeoutError('No bag resume service')
            time.sleep(2)
            playback_started = time.monotonic()
            record_event('resume_requested', True)
            future = client.call_async(Resume.Request())
            rclpy.spin_until_future_complete(node, future, timeout_sec=5)
            if not future.done() or future.exception(): raise RuntimeError('Bag resume failed')
            deadline = time.monotonic() + args.seconds
            heartbeat = time.monotonic() + 20
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.01)
                if slam.poll() is not None: raise RuntimeError('SLAM exited during replay')
                if player.poll() is not None: raise RuntimeError('Bag ended before requested duration')
                if time.monotonic() >= heartbeat:
                    print('Progress:', result, flush=True)
                    heartbeat += 20
            result['runtime_completed'] = True
        finally:
            record_event('observation_end', result['valid'])
            stop(player)
            stop(slam)
            result['slam_exit_code'] = slam.returncode if slam else None
            result['clean_shutdown'] = result['slam_exit_code'] == 0
            trajectory.close()
            state_events.close()
            result['map_unchanged'] = before == {
                str(p.relative_to(args.map)): digest_file(p)
                for p in sorted(args.map.rglob('*')) if p.is_file()}
            if (args.output / 'metrics/frames.csv').exists():
                result['metrics'] = summarize(args.output / 'metrics')
            (args.output / 'result.json').write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
            node.destroy_node()
            rclpy.shutdown()
    print(json.dumps(result, indent=2), flush=True)
    if not result['map_unchanged']: raise RuntimeError('Read-only map changed')


if __name__ == '__main__':
    main()
