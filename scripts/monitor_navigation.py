#!/usr/bin/env python3
"""Read-only Linux/ROS 2 navigation telemetry. No command/action publishers."""
import argparse
import csv
import datetime as dt
import json
import math
import os
from pathlib import Path
import platform
import resource
import select
import shutil
import signal
import struct
import subprocess
import sys
import tarfile
import threading
import time

TARGETS = ('voxelslam', 'rviz2', 'controller_server', 'velocity_smoother',
           'planner_server', 'bt_navigator', 'behavior_server', 'twist_bridge',
           'rslidar_sdk_node', 'wit_ros2_imu', 'lifecycle_manager')


def read(path):
    try:
        return Path(path).read_text()
    except (OSError, UnicodeError):
        return ''


def number(path, scale=1):
    try:
        return float(read(path).strip()) / scale
    except ValueError:
        return None


def key_values(text):
    out = {}
    for line in text.splitlines():
        parts = line.replace(':', ' ').split()
        if len(parts) >= 2:
            try:
                out[parts[0]] = int(parts[1])
            except ValueError:
                pass
    return out


def parse_proc_stat(text):
    """Keep spaces/parentheses in Linux comm from shifting numeric fields."""
    end = text.rfind(')')
    if end < 0:
        raise ValueError('incomplete process stat')
    v = text[end + 1:].split()
    return {'ticks': int(v[11]) + int(v[12]), 'start': int(v[19]),
            'threads': int(v[17]), 'rss_pages': int(v[21]), 'state': v[0]}


def cdr_header_stamp(data):
    if len(data) < 12 or data[:2] not in (b'\x00\x00', b'\x00\x01'):
        raise ValueError('unsupported CDR header')
    endian = '<' if data[1] else '>'
    seconds, nanoseconds = struct.unpack_from(endian + 'iI', data, 4)
    return seconds + nanoseconds * 1e-9


def cdr_tf_headers(data):
    """Read TF headers without allocating Vector3/Quaternion objects for every TF.

    CDR alignment is relative to the payload after its four-byte encapsulation.
    Unknown encodings fall back to the installed ROS deserializer at the call site.
    """
    if len(data) < 8 or data[:2] not in (b'\x00\x00', b'\x00\x01'):
        raise ValueError('unsupported TF CDR header')
    endian = '<' if data[1] else '>'
    offset = 4
    def align(size):
        nonlocal offset
        offset += (-(offset - 4)) % size
    def uint():
        nonlocal offset
        align(4)
        value = struct.unpack_from(endian + 'I', data, offset)[0]
        offset += 4
        return value
    def string():
        nonlocal offset
        length = uint()
        if length < 1 or offset + length > len(data):
            raise ValueError('truncated CDR string')
        value = data[offset:offset + length - 1].decode('utf-8')
        offset += length
        return value
    count = uint()
    if count > len(data) // 64:
        raise ValueError('invalid TF sequence length')
    headers = []
    for _ in range(count):
        align(4)
        sec, nsec = struct.unpack_from(endian + 'iI', data, offset)
        offset += 8
        parent, child = string(), string()
        align(8)
        offset += 7 * 8  # translation + quaternion
        if offset > len(data):
            raise ValueError('truncated TF transform')
        headers.append((parent, child, sec + nsec * 1e-9))
    return headers


class Csv:
    def __init__(self, directory, name, fields):
        self.file = (directory / name).open('w', newline='')
        self.writer = csv.DictWriter(self.file, fieldnames=fields)
        self.writer.writeheader()

    def row(self, **values):
        self.writer.writerow(values)

    def flush(self):
        self.file.flush()

    def close(self):
        self.file.close()


class Stream:
    def __init__(self):
        self.count = self.window_count = 0
        self.last = self.previous_segment = None
        self.max_gap = self.active_max_gap = 0.
        self.active_intervals = self.active_over_200ms = 0
        self.active_sum = 0.
        self.active_window_max = self.window_max = 0.
        self.nonzero_count = 0
        self.last_value = None
        self.stamp_age = None

    def receive(self, now, segment=None, value=None, stamp_age=None, nonzero=False):
        gap = None if self.last is None else now - self.last
        active = segment is not None and segment == self.previous_segment
        if gap is not None:
            self.max_gap = max(self.max_gap, gap)
            self.window_max = max(self.window_max, gap)
            if active:
                self.active_max_gap = max(self.active_max_gap, gap)
                self.active_window_max = max(self.active_window_max, gap)
                self.active_intervals += 1
                self.active_sum += gap
                self.active_over_200ms += gap > .2
        self.last = now
        self.previous_segment = segment
        self.count += 1
        self.window_count += 1
        self.nonzero_count += bool(nonzero)
        if value is not None:
            self.last_value = value
        self.stamp_age = stamp_age
        return gap, active

    def summary(self, now=None):
        return {'received': self.count, 'nonzero_received': self.nonzero_count,
                'max_gap_sec_including_idle': self.max_gap,
                'active_follow_path_intervals': self.active_intervals,
                'active_follow_path_mean_interval_sec':
                    self.active_sum / self.active_intervals if self.active_intervals else None,
                'active_follow_path_max_gap_sec': self.active_max_gap,
                'active_follow_path_gaps_over_200ms': self.active_over_200ms,
                'last_value': self.last_value,
                'last_receive_age_sec_at_stop': now-self.last if now is not None and self.last is not None else None}


class SystemSampler:
    def __init__(self, directory, started, interval, top_threads):
        self.directory, self.started = directory, started
        self.interval, self.top_threads = interval, top_threads
        self.stop = threading.Event()
        self.previous = {}
        self.last_process = {}
        self.previous_disk = {}
        self.last_swap = None
        self.hz = os.sysconf('SC_CLK_TCK')
        self.page_kb = os.sysconf('SC_PAGE_SIZE') / 1024
        self.started_processes = {}
        self.errors = []
        self.cpu_seconds = 0.
        self.peak_cpu_busy = 0.
        self.min_available_kb = None
        self.peak_temperature_c = None
        self.tables = {
            'cpu': Csv(directory, 'cpu.csv', ['elapsed_s', 'cpu', 'busy_pct', 'iowait_pct', 'steal_pct']),
            'system': Csv(directory, 'system.csv', ['elapsed_s', 'mem_available_kb', 'mem_total_kb',
                'swap_used_kb', 'swap_in_pages_per_s', 'swap_out_pages_per_s', 'load1',
                'processes_running', 'processes_blocked', 'pressure_cpu', 'pressure_memory', 'pressure_io']),
            'process': Csv(directory, 'processes.csv', ['elapsed_s', 'pid', 'start_ticks', 'name',
                'cpu_pct_one_core', 'rss_kb', 'threads', 'state', 'read_bytes_per_s', 'write_bytes_per_s',
                'voluntary_switches_per_s', 'involuntary_switches_per_s']),
            'thread': Csv(directory, 'threads.csv', ['elapsed_s', 'pid', 'tid', 'name', 'cpu_pct_one_core', 'state']),
            'sensors': Csv(directory, 'sensors.csv', ['elapsed_s', 'sensor', 'value', 'unit']),
            'disks': Csv(directory, 'disks.csv', ['elapsed_s', 'device', 'read_kb_per_s', 'write_kb_per_s', 'busy_pct']),
        }
        self.temp_paths = sorted(Path('/sys/class/thermal').glob('thermal_zone*/temp'))
        self.temp_paths += sorted(Path('/sys/class/hwmon').glob('hwmon*/temp*_input'))
        self.freq_paths = sorted(Path('/sys/devices/system/cpu/cpufreq').glob('policy*/scaling_cur_freq'))
        self.throttle_paths = sorted(Path('/sys/devices/system/cpu').glob('cpu*/thermal_throttle/*_throttle_count'))
        self.worker = threading.Thread(target=self.run, name='performance_system_sampler', daemon=True)

    def sample(self):
        now = time.monotonic()
        elapsed = now - self.started
        stat = read('/proc/stat')
        for line in stat.splitlines():
            parts = line.split()
            if not parts or not parts[0].startswith('cpu'):
                continue
            values = list(map(int, parts[1:9]))
            old = self.previous.get(parts[0])
            self.previous[parts[0]] = values
            if old:
                delta = [a - b for a, b in zip(values, old)]
                total = sum(delta)
                if total > 0:
                    busy = 100 * (total - delta[3] - delta[4]) / total
                    self.peak_cpu_busy = max(self.peak_cpu_busy, busy)
                    self.tables['cpu'].row(elapsed_s=elapsed, cpu=parts[0], busy_pct=busy,
                                          iowait_pct=100 * delta[4] / total, steal_pct=100 * delta[7] / total)
        mem = key_values(read('/proc/meminfo'))
        vm = key_values(read('/proc/vmstat'))
        swap = [vm.get('pswpin', 0), vm.get('pswpout', 0)]
        swap_rates = [None, None]
        if self.last_swap:
            old_time, old = self.last_swap
            swap_rates = [(a - b) / (now - old_time) for a, b in zip(swap, old)]
        self.last_swap = (now, swap)
        available = mem.get('MemAvailable')
        if available is not None:
            self.min_available_kb = available if self.min_available_kb is None else min(self.min_available_kb, available)
        fields = key_values(stat)
        self.tables['system'].row(elapsed_s=elapsed, mem_available_kb=available,
            mem_total_kb=mem.get('MemTotal'), swap_used_kb=mem.get('SwapTotal', 0) - mem.get('SwapFree', 0),
            swap_in_pages_per_s=swap_rates[0], swap_out_pages_per_s=swap_rates[1], load1=os.getloadavg()[0],
            processes_running=fields.get('procs_running'), processes_blocked=fields.get('procs_blocked'),
            pressure_cpu=read('/proc/pressure/cpu').strip().replace('\n', '; '),
            pressure_memory=read('/proc/pressure/memory').strip().replace('\n', '; '),
            pressure_io=read('/proc/pressure/io').strip().replace('\n', '; '))
        for path in self.temp_paths:
            val = number(path, 1000)
            if val is not None and -30 < val < 150:
                sensor = str(path) + ':' + read(path.parent / 'type').strip() + read(path.parent / 'name').strip()
                self.tables['sensors'].row(elapsed_s=elapsed, sensor=sensor, value=val, unit='C')
                self.peak_temperature_c = val if self.peak_temperature_c is None else max(self.peak_temperature_c, val)
        for path in self.freq_paths + self.throttle_paths:
            val = number(path, 1000 if path in self.freq_paths else 1)
            if val is not None:
                self.tables['sensors'].row(elapsed_s=elapsed, sensor=str(path), value=val,
                                          unit='MHz' if path in self.freq_paths else 'count')
        self.sample_processes(now, elapsed)
        for line in read('/proc/diskstats').splitlines():
            v = line.split()
            if len(v) < 14 or v[2].startswith(('loop', 'ram')):
                continue
            dev = v[2]
            # Disk and partition rows are kept separately; do not sum them.
            current = [int(v[5]), int(v[9]), int(v[12])]
            old = self.previous_disk.get(dev)
            self.previous_disk[dev] = (now, current)
            if old:
                delta_t = now - old[0]
                d = [a - b for a, b in zip(current, old[1])]
                self.tables['disks'].row(elapsed_s=elapsed, device=dev, read_kb_per_s=d[0] / 2 / delta_t,
                    write_kb_per_s=d[1] / 2 / delta_t, busy_pct=d[2] / (delta_t * 10))
        for table in self.tables.values():
            table.flush()

    def sample_processes(self, now, elapsed):
        current_keys = set()
        for proc in Path('/proc').iterdir():
            if not proc.name.isdigit():
                continue
            try:
                name = read(proc / 'comm').strip()
                command = (proc / 'cmdline').read_bytes().decode(errors='replace').replace('\0', ' ').strip()
                # Match executable/script names, not incidental strings in shell commands.
                argv = command.split()
                candidates = [Path(arg).name for arg in argv[:2]]
                if not any(any(target in candidate for target in TARGETS) for candidate in candidates):
                    continue
                st = parse_proc_stat(read(proc / 'stat'))
                io = key_values(read(proc / 'io'))
                ctx = key_values(read(proc / 'status'))
                values = [st['ticks'], io.get('read_bytes'), io.get('write_bytes'),
                          ctx.get('voluntary_ctxt_switches'), ctx.get('nonvoluntary_ctxt_switches')]
                key = (int(proc.name), st['start'])
                current_keys.add(key)
                self.started_processes[str(key)] = {'pid': int(proc.name), 'name': name, 'command': command}
                old = self.last_process.get(key)
                rates = [None] * 5
                if old:
                    delta_t = now - old[0]
                    rates = [(a - b) / delta_t if a is not None and b is not None else None
                             for a, b in zip(values, old[1])]
                    rates[0] *= 100 / self.hz
                self.last_process[key] = (now, values)
                self.tables['process'].row(elapsed_s=elapsed, pid=proc.name, start_ticks=st['start'], name=name,
                    cpu_pct_one_core=rates[0], rss_kb=st['rss_pages'] * self.page_kb, threads=st['threads'],
                    state=st['state'], read_bytes_per_s=rates[1], write_bytes_per_s=rates[2],
                    voluntary_switches_per_s=rates[3], involuntary_switches_per_s=rates[4])
                threads = []
                for task in (proc / 'task').iterdir():
                    try:
                        ts = parse_proc_stat(read(task / 'stat'))
                        tk = (int(proc.name), int(task.name), ts['start'])
                        current_keys.add(tk)
                        old_thread = self.last_process.get(tk)
                        self.last_process[tk] = (now, [ts['ticks']])
                        if old_thread:
                            cpu = (ts['ticks'] - old_thread[1][0]) * 100 / self.hz / (now - old_thread[0])
                            threads.append((cpu, task.name, task, ts['state']))
                    except (OSError, ValueError, IndexError):
                        pass
                for cpu, tid, task, state in sorted(threads, reverse=True)[:self.top_threads]:
                    self.tables['thread'].row(elapsed_s=elapsed, pid=proc.name, tid=tid,
                                              name=read(task / 'comm').strip(), cpu_pct_one_core=cpu, state=state)
            except (OSError, ValueError, IndexError):
                continue
        self.last_process = {key: value for key, value in self.last_process.items() if key in current_keys}

    def run(self):
        cpu_start = time.thread_time()
        while not self.stop.is_set():
            before = time.monotonic()
            try:
                self.sample()
            except Exception as exc:
                self.errors.append(str(exc))
            self.stop.wait(max(0, self.interval - (time.monotonic() - before)))
        self.cpu_seconds = time.thread_time() - cpu_start
        for table in self.tables.values():
            table.close()


def command_output(command):
    try:
        result = subprocess.run(command, capture_output=True, text=True, timeout=4)
        return result.stdout + result.stderr
    except (OSError, subprocess.TimeoutExpired) as exc:
        return str(exc)


def collect_logs(directory, started_wall, process_ids=()):
    root = Path(os.environ.get('ROS_LOG_DIR', Path(os.environ.get('ROS_HOME', Path.home() / '.ros')) / 'log'))
    target = directory / 'ros_logs'
    target.mkdir()
    copied = 0
    if root.is_dir():
        for path in sorted(root.glob('*.log'), key=lambda p: p.stat().st_mtime, reverse=True):
            if copied >= 24:
                break
            try:
                running_pid = any('_' + str(pid) + '_' in path.name for pid in process_ids)
                if (path.stat().st_mtime < started_wall - 2 and not running_pid) or not any(t[:12] in path.name for t in TARGETS):
                    continue
                with path.open('rb') as f:
                    size = path.stat().st_size
                    f.seek(max(0, size - 192 * 1024))
                    data = f.read(192 * 1024)
                (target / path.name).write_bytes(data)
                copied += 1
            except OSError:
                pass
    if root.is_dir():
        for path in sorted(root.glob('*/launch.log'), key=lambda p: p.stat().st_mtime, reverse=True)[:3]:
            try:
                with path.open('rb') as f:
                    f.seek(max(0, path.stat().st_size - 192 * 1024))
                    (target / (path.parent.name + '_launch.log')).write_bytes(f.read(192 * 1024))
                copied += 1
            except OSError:
                pass
    return copied


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=0, help='seconds; default: until Ctrl+C')
    parser.add_argument('--interval', type=float, default=1, help='system sampling interval, minimum 0.5s')
    parser.add_argument('--mode', choices=['live', 'bag'], default='live', help='bag matches launch localhost/FastDDS settings')
    parser.add_argument('--system-only', action='store_true', help='skip ROS subscriptions')
    parser.add_argument('--output', type=Path, default=Path.home() / 'go2_perf_logs', help='parent directory for reports')
    parser.add_argument('--top-threads', type=int, default=8, help='highest CPU threads per process per sample')
    args = parser.parse_args()
    if not math.isfinite(args.duration) or args.duration < 0 or not math.isfinite(args.interval) or args.interval < .5 or args.top_threads < 1:
        parser.error('duration must be >=0; interval >=0.5; top-threads >=1')
    if args.mode == 'bag':
        os.environ['RMW_IMPLEMENTATION'] = 'rmw_fastrtps_cpp'
        os.environ['ROS_LOCALHOST_ONLY'] = '1'
        os.environ.pop('CYCLONEDDS_URI', None)
    started_wall = time.time()
    timestamp = dt.datetime.now().astimezone().strftime('%Y%m%d_%H%M%S_%f')
    directory = args.output.expanduser().resolve() / ('navigation_' + timestamp)
    directory.mkdir(parents=True)
    workspace = Path(__file__).resolve().parents[1]
    metadata = {'started_at': dt.datetime.now().astimezone().isoformat(), 'hostname': platform.node(),
        'platform': platform.platform(), 'cpu_count': os.cpu_count(), 'arguments': vars(args).copy(),
        'environment': {key: os.environ.get(key) for key in ['ROS_DOMAIN_ID', 'RMW_IMPLEMENTATION', 'ROS_LOCALHOST_ONLY',
            'GO2_NETWORK_INTERFACE', 'GO2_NAV_WORKSPACE', 'CYCLONEDDS_URI', 'DISPLAY', 'WAYLAND_DISPLAY']},
        'git_commit': command_output(['git', '-C', str(workspace), 'rev-parse', 'HEAD']).strip(),
        'lscpu': command_output(['lscpu']), 'network_addresses': command_output(['ip', '-br', 'address']),
        'graphics': command_output(['glxinfo', '-B']) if os.environ.get('DISPLAY') else 'No DISPLAY; renderer not queried.',
        'notes': ['CPU process/thread percentages use one core=100%. Per-core data is in cpu.csv.',
            'ROS gaps are observer receipt times; communication loss and observer delays can also cause gaps.',
            'Only small telemetry/status topics are subscribed. No PointCloud2 subscription or motion publishing.',
            'ROS log files contain at most the last 192 KiB each; older/unchanged logs are excluded.']}
    (directory / 'metadata.json').write_text(json.dumps(metadata, indent=2, default=str))
    events_file = (directory / 'events.jsonl').open('w')
    def event(kind, **values):
        events_file.write(json.dumps({'elapsed_s': time.monotonic() - started, 'kind': kind, **values}, default=str) + '\n')
        events_file.flush()
    started = time.monotonic()
    metadata['sample_time_origin_unix'] = time.time()
    metadata['monitor_pid'] = os.getpid()
    (directory / 'metadata.json').write_text(json.dumps(metadata, indent=2, default=str))
    sampler = SystemSampler(directory, started, args.interval, args.top_threads)
    streams = {}
    node = None
    executor = None
    ros_error = None
    active = False
    segment = 0
    executing_ids = set()
    subscriptions = []
    clients = []
    pending = {}
    lifecycle_states = {}
    topic_table = Csv(directory, 'topics.csv', ['elapsed_s', 'topic', 'received_window', 'rate_hz',
        'last_receive_age_s', 'max_gap_window_s', 'active_follow_path_max_gap_window_s',
        'source_stamp_age_s', 'follow_path_executing', 'segment', 'last_value'])
    try:
        if not args.system_only:
            import rclpy
            from rclpy.qos import qos_profile_sensor_data
            from geometry_msgs.msg import Twist
            from nav_msgs.msg import Odometry
            from sensor_msgs.msg import Imu
            from std_msgs.msg import String, Bool, Float64
            from tf2_msgs.msg import TFMessage
            from rosgraph_msgs.msg import Clock
            from action_msgs.msg import GoalStatusArray
            from lifecycle_msgs.srv import GetState
            from rclpy.parameter import Parameter
            from rclpy.serialization import deserialize_message
            from rclpy.executors import SingleThreadedExecutor
            rclpy.init()
            node = rclpy.create_node('go2_navigation_performance_monitor_' + str(os.getpid()),
                parameter_overrides=[Parameter('use_sim_time', value=args.mode == 'bag')])
            names = ['/cmd_vel_nav', '/cmd_vel', '/LIO/odom_imu', '/imu/data_raw',
                'tf:odom->imu_link', 'tf:map->odom', '/localization/state', '/localization/valid',
                '/localization/correction_age', '/clock']
            streams = {name: Stream() for name in names}
            def receive(name, msg=None, stamp=None, value=None, nonzero=False, stamp_seconds=None):
                now = time.monotonic()
                age = None
                if stamp is not None or stamp_seconds is not None:
                    seconds = stamp_seconds if stamp_seconds is not None else stamp.sec + stamp.nanosec * 1e-9
                    ros_now = node.get_clock().now().nanoseconds / 1e9
                    age = ros_now - seconds if seconds else None
                stream = streams[name]
                old_value = stream.last_value
                gap, was_active = stream.receive(now, segment if active else None, value, age, nonzero)
                if value is not None and value != old_value and name in ('/localization/state', '/localization/valid'):
                    event('state_change', topic=name, value=value)
                threshold = .1 if name.startswith(('tf:', '/LIO/', '/imu/')) else (1. if name.startswith('/localization/') else .2)
                if gap is not None and gap > threshold:
                    event('receive_gap', topic=name, gap_sec=gap, continuously_executing=was_active)
            for topic in ['/cmd_vel_nav', '/cmd_vel']:
                def velocity(msg, name=topic):
                    receive(name, value=[msg.linear.x, msg.linear.y, msg.angular.z],
                            nonzero=abs(msg.linear.x)+abs(msg.linear.y)+abs(msg.angular.z)>1e-6)
                subscriptions.append(node.create_subscription(Twist, topic, velocity, qos_profile_sensor_data))
            for topic, cls in [('/LIO/odom_imu', Odometry), ('/imu/data_raw', Imu)]:
                def stamped_raw(data, name=topic, message_type=cls):
                    try:
                        seconds = cdr_header_stamp(data)
                    except (ValueError, struct.error):
                        stamp = deserialize_message(data, message_type).header.stamp
                        seconds = stamp.sec + stamp.nanosec*1e-9
                    receive(name, stamp_seconds=seconds)
                subscriptions.append(node.create_subscription(cls, topic, stamped_raw, qos_profile_sensor_data, raw=True))
            for topic, cls in [('/localization/state', String), ('/localization/valid', Bool), ('/localization/correction_age', Float64)]:
                subscriptions.append(node.create_subscription(cls, topic,
                    lambda msg, name=topic: receive(name, value=msg.data), 10))
            def transforms(data):
                try:
                    headers = cdr_tf_headers(data)
                except (ValueError, UnicodeError, struct.error):
                    msg = deserialize_message(data, TFMessage)
                    headers = [(t.header.frame_id, t.child_frame_id,
                        t.header.stamp.sec+t.header.stamp.nanosec*1e-9) for t in msg.transforms]
                for parent, child, seconds in headers:
                    name = 'tf:' + parent + '->' + child
                    if name in streams:
                        receive(name, stamp_seconds=seconds)
            subscriptions.append(node.create_subscription(TFMessage, '/tf', transforms, qos_profile_sensor_data, raw=True))
            subscriptions.append(node.create_subscription(Clock, '/clock', lambda msg: receive('/clock', value=msg.clock.sec+msg.clock.nanosec*1e-9), qos_profile_sensor_data))
            def action_status(msg, topic):
                nonlocal active, segment, executing_ids
                current_ids = {bytes(s.goal_info.goal_id.uuid).hex() for s in msg.status_list if s.status == 2}
                if topic == '/follow_path':
                    next_active = bool(current_ids)
                    # A changed action id splits timing statistics even if a cancel/finish update was missed.
                    if next_active and (not active or current_ids != executing_ids):
                        segment += 1
                    active = next_active
                    executing_ids = current_ids
                event('action_status', action=topic, statuses=[{'id': bytes(s.goal_info.goal_id.uuid).hex(), 'status': s.status} for s in msg.status_list])
            for topic in ['/follow_path', '/navigate_to_pose']:
                subscriptions.append(node.create_subscription(GoalStatusArray, topic+'/_action/status',
                    lambda msg, name=topic: action_status(msg, name), 10))
            for name in ['/controller_server', '/velocity_smoother', '/local_costmap/local_costmap', '/bt_navigator']:
                clients.append((name, node.create_client(GetState, name+'/get_state')))
            executor = SingleThreadedExecutor()
            executor.add_node(node)
            # Record actual node clock behavior as well as the shell's environment.
            event('ros_started', monitor_use_sim_time=node.get_parameter('use_sim_time').value)
        else:
            event('system_only')
    except (ImportError, RuntimeError) as exc:
        ros_error = str(exc)
        print('ROS monitoring unavailable; recording system data only:', exc, flush=True)
        event('ros_error', error=ros_error)
        if node:
            node.destroy_node()
            node = None
    stopping = False
    def stop_handler(signum, frame):
        nonlocal stopping
        stopping = True
    old_signals = {sig: signal.signal(sig, stop_handler) for sig in (signal.SIGINT, signal.SIGTERM)}
    sampler.worker.start()
    print('Recording to:', directory, flush=True)
    print('Press ENTER to mark a stall. Ctrl+C stops, summarizes and creates a .tar.gz file.', flush=True)
    last_flush = last_graph = started
    monitor_error = None
    try:
        while not stopping and (not args.duration or time.monotonic() - started < args.duration):
            if node:
                executor.spin_once(timeout_sec=.05)
            else:
                time.sleep(.05)
            now = time.monotonic()
            if sys.stdin.isatty() and select.select([sys.stdin], [], [], 0)[0]:
                sys.stdin.readline()
                event('user_mark')
                print('Marked at %.1fs' % (now - started), flush=True)
            if now - last_flush >= args.interval:
                for name, stream in streams.items():
                    topic_table.row(elapsed_s=now-started, topic=name, received_window=stream.window_count,
                        rate_hz=stream.window_count/(now-last_flush),
                        last_receive_age_s=now-stream.last if stream.last is not None else None,
                        max_gap_window_s=stream.window_max, active_follow_path_max_gap_window_s=stream.active_window_max,
                        source_stamp_age_s=stream.stamp_age, follow_path_executing=active, segment=segment,
                        last_value=json.dumps(stream.last_value))
                    stream.window_count = 0
                    stream.window_max = stream.active_window_max = 0.
                topic_table.flush()
                last_flush = now
            if now-last_graph >= 10:
                if node:
                    for name in ['/cmd_vel_nav', '/cmd_vel', '/LIO/odom_imu', '/tf']:
                        infos = node.get_publishers_info_by_topic(name)
                        event('topic_graph', topic=name, publishers=[{'name': p.node_name, 'namespace': p.node_namespace,
                            'type': p.topic_type, 'qos': str(p.qos_profile)} for p in infos],
                            subscribers=[p.node_name for p in node.get_subscriptions_info_by_topic(name)])
                    for name, client in clients:
                        if name in pending and not pending[name].done():
                            continue
                        if client.service_is_ready():
                            future = client.call_async(GetState.Request())
                            pending[name] = future
                            def done(f, name=name):
                                try:
                                    state = f.result().current_state.label
                                    lifecycle_states[name] = state
                                    event('lifecycle_state', node=name, state=state)
                                except Exception as exc:
                                    event('lifecycle_error', node=name, error=str(exc))
                            future.add_done_callback(done)
                    rates = {name: streams[name].count for name in ['/cmd_vel_nav', '/cmd_vel']}
                    print('%.0fs follow_path=%s localization=%s received=%s' %
                          (now-started, active, streams['/localization/state'].last_value, rates), flush=True)
                    if not any(s.count for s in streams.values()):
                        print('No ROS messages: check ROS_DOMAIN_ID, DDS/network, and --mode bag for bag tests.', flush=True)
                last_graph = now
    except Exception as exc:
        monitor_error = str(exc)
        event('monitor_error', error=monitor_error)
    finally:
        sampler.stop.set()
        sampler.worker.join(timeout=10)
        for sig, old in old_signals.items():
            signal.signal(sig, old)
        if executor:
            executor.shutdown()
        if node:
            node.destroy_node()
        if not args.system_only:
            try:
                if rclpy.ok():
                    rclpy.shutdown()
            except (NameError, RuntimeError):
                pass
        topic_table.close()
        events_file.close()
    duration = time.monotonic()-started
    usage = resource.getrusage(resource.RUSAGE_SELF)
    summary = {'duration_sec': duration, 'ros_enabled': node is not None, 'ros_error': ros_error,
        'monitor_error': monitor_error, 'streams': {name: s.summary(time.monotonic()) for name, s in streams.items()},
        'follow_path_executing_at_stop': active,
        'lifecycle_states': lifecycle_states,
        'system': {'peak_cpu_busy_pct_any_core_or_total': sampler.peak_cpu_busy,
            'min_mem_available_kb': sampler.min_available_kb, 'peak_sensor_temperature_c': sampler.peak_temperature_c,
            'sampler_errors': sampler.errors, 'sampler_cpu_seconds': sampler.cpu_seconds, 'processes_seen': list(sampler.started_processes.values())},
        'monitor_process_cpu_seconds': usage.ru_utime+usage.ru_stime,
        'monitor_cpu_pct_one_core_approx': (usage.ru_utime+usage.ru_stime)/duration*100,
        'ros_log_tails_copied': collect_logs(directory, started_wall,
            [p['pid'] for p in sampler.started_processes.values()]),
        'interpretation': ['Use topics.csv last_receive_age_s to identify ongoing silence, not just completed receive gaps.',
            'active_follow_path_* covers consecutive receives in the same observed executing action segment.',
            'No messages is not zero velocity. No publisher/incorrect DDS/idle goals also cause silence.',
            'Message timestamps use ROS time; CPU/events elapsed_s use monotonic time. metadata.json provides wall start.',
            'CPU saturation or high temperature alone does not prove causation; correlate with topic gaps and ROS log times.']}
    (directory/'summary.json').write_text(json.dumps(summary, indent=2, default=str))
    with tarfile.open(str(directory)+'.tar.gz', 'w:gz') as archive:
        archive.add(directory, arcname=directory.name)
    print('Finished. Send this file:', str(directory)+'.tar.gz', flush=True)
    if monitor_error:
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
