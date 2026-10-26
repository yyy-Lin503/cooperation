"""GUI-owned ROS startup and teardown. Radar and motion algorithms stay elsewhere.

stdin/stdout carry newline-delimited JSON commands/events. Child output is
wrapped as log events; children never inherit the GUI command pipe.
"""
from pathlib import Path
import argparse
import json
import os
import queue
import signal
import subprocess
import sys
import threading
import time
import uuid

from .gui_processes import OwnedProcess, atomic_json, process_identity, process_table
from .gui_runtime_support import acquire_topic_lock, validate_settings
from .runtime_support import verify_loaded_package, shm_key, runtime_paths


class RosBackend:
    def __init__(self, root, settings):
        import rclpy
        import sysv_ipc
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        from nav_msgs.msg import Odometry
        from geometry_msgs.msg import Twist
        self.rclpy, self.ipc, self.twist = rclpy, sysv_ipc, Twist
        self.root, self.settings = root, settings
        self.node = Node('radar_gui_runtime_' + uuid.uuid4().hex[:10])
        self.last_odom = None
        self.subscription = self.node.create_subscription(Odometry, settings['odom_topic'], self.receive,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                       durability=DurabilityPolicy.VOLATILE))
        self.zero_publisher = None
        self.topic = self.node.resolve_topic_name(settings['cmd_vel_topic'])

    def receive(self, _message):
        self.last_odom = time.monotonic()

    def spin(self):
        self.rclpy.spin_once(self.node, timeout_sec=0)

    def chassis_present(self):
        matches = [name for name, _ in self.node.get_node_names_and_namespaces()
                   if name == self.settings['chassis_node']]
        if len(matches) > 1:
            raise RuntimeError('Multiple configured chassis nodes found; refusing ambiguous reuse')
        return bool(matches)

    def chassis_ready(self):
        return (self.chassis_present() and
                self.node.count_subscribers(self.topic) > 0 and self.odom_fresh())

    def odom_fresh(self):
        return self.last_odom is not None and time.monotonic() - self.last_odom <= .5

    def other_publishers(self):
        return [p.node_namespace.rstrip('/') + '/' + p.node_name
                for p in self.node.get_publishers_info_by_topic(self.topic)
                if p.node_name != self.node.get_name()]

    def receiver_state(self):
        from .odom_transport import decode
        memory = None
        try:
            memory = self.ipc.SharedMemory(shm_key(), flags=0)
            first, second = memory.read(32), memory.read(32)
            sample = decode(first, time.time()) if first == second else {}
            if sample.get('valid'):
                pid = sample['writer_pid']
                command = Path('/proc', str(pid), 'cmdline').read_bytes().decode('utf-8', errors='replace')
                own_path = any(token.startswith(str(self.root) + os.sep) for token in command.split('\0'))
                environment = Path('/proc', str(pid), 'environ').read_bytes().decode('utf-8', errors='replace').split('\0')
                own_environment = 'RADAR_PROJECT_ROOT=' + str(self.root) in environment
                if ((own_path or own_environment) and
                    ('radar_odom_receiver' in command or 'my_radar_follower.odom_receiver' in command)):
                    return 'fresh'
                return 'conflict'
        except (OSError, ValueError, self.ipc.Error):
            pass
        finally:
            if memory is not None:
                memory.detach()
        import fcntl
        lock = Path('/tmp', 'radar_odom_writer_' + str(shm_key()) + '.lock').open('a')
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            return 'absent'
        except OSError:
            return 'busy'
        finally:
            lock.close()

    def zero(self):
        if self.zero_publisher is None:
            self.zero_publisher = self.node.create_publisher(self.twist, self.topic, 10)
        self.zero_publisher.publish(self.twist())

    def close(self):
        self.node.destroy_node()


class Runtime:
    def __init__(self, root, folder, settings, backend, emit, factory=OwnedProcess, clock=time.monotonic):
        self.root, self.folder, self.settings = Path(root), Path(folder), settings
        self.backend, self.emit, self.factory, self.clock = backend, emit, factory, clock
        self.folder.mkdir(parents=True, exist_ok=True)
        self.runtime_id = self.folder.name
        self.context = self.folder / 'context.json'
        self.control_status = self.folder / 'control.json'
        self.registry = self.folder / 'owned.json'
        self.owned = {}
        self.external = set()
        self.phase = 'idle'
        self.session = '0'
        self.want_follow = False
        self.stopping = []
        self.stopped_at = None
        self.close_requested = False
        self.failure = ''
        self.deadline = 0
        self.last_heartbeat = clock()
        self.last_snapshot = 0
        self.last_status = 0
        self.last_context_write = 0
        self.registry_value = None
        self.topic_lock = None
        self.done = False
        self.status = {}
        self.write_context()
        self.snapshot()
        self.emit('ready', runtime_id=self.runtime_id)

    def write_context(self):
        atomic_json(self.context, dict(runtime_id=self.runtime_id, session_id=self.session,
            permit=self.want_follow and self.phase == 'following' and not self.stopping,
            lease_monotonic=self.clock()))
        self.last_context_write = self.clock()

    def snapshot(self):
        table = process_table() if any(isinstance(p, OwnedProcess) for p in self.owned.values()) else None
        for child in self.owned.values():
            if isinstance(child, OwnedProcess):child.refresh(table)
            else:child.refresh()
        value = dict(manager=process_identity(os.getpid()),
            owned={name: sorted(child.identities, key=lambda p:p['pid']) for name, child in self.owned.items()})
        if value != self.registry_value:
            atomic_json(self.registry, value)
            self.registry_value = value

    def state(self, phase=None, message=''):
        self.last_status = self.clock()
        if phase is not None:
            self.phase = phase
        value = dict(phase=self.phase, message=message,
            components={name: ('owned' if name in self.owned else 'external' if name in self.external else 'off')
                        for name in ('chassis', 'odom', 'follow')},
            odom_fresh=self.backend.odom_fresh(), control=self.status)
        self.emit('state', **value)

    def spawn(self, name, argv):
        child = self.factory(name, argv, self.root, self.emit)
        self.owned[name] = child
        # Register the root before a reader thread or another startup stage.
        self.snapshot()
        self.emit('log', component=name, message='Started PID ' + str(child.process.pid))
        return child

    def command(self, message):
        action = message.get('action')
        if action == 'heartbeat':
            self.last_heartbeat = self.clock()
        elif action == 'session':
            session = str(message.get('session_id', '0'))
            if not session.isdecimal():
                raise ValueError('Invalid GUI capture session')
            self.session = str(int(session))
            self.write_context()
        elif action == 'start':
            if self.want_follow or self.stopping or self.close_requested:
                return
            self.failure = ''
            self.status = {}
            self.want_follow = True
            self.deadline = self.clock() + 1  # Let DDS graph discovery settle.
            self.state('discovering', 'Checking ROS graph')
        elif action == 'stop_follow':
            self.want_follow = False
            self.write_context()
            self.request_stop(['follow'])
        elif action == 'shutdown':
            self.shutdown()
        else:
            raise ValueError('Unknown GUI runtime action')

    def shutdown(self, error=''):
        self.want_follow = False
        self.close_requested = True
        self.failure = error or self.failure
        try:
            self.write_context()
        except OSError as exc:
            self.emit('log', component='runtime', message='Cannot revoke context file: ' + str(exc))
        self.request_stop(['follow', 'odom', 'chassis'])

    def request_stop(self, names):
        for name in names:
            if name in self.owned and name not in self.stopping:
                self.stopping.append(name)
        self.state('stopping', self.failure)
        if not self.stopping:
            self.finish_stop()

    def finish_stop(self):
        if self.topic_lock is not None:
            self.topic_lock.close()
            self.topic_lock = None
        self.state('error' if self.failure else 'idle', self.failure)
        if self.close_requested:
            self.done = True

    def bridge_argv(self):
        def parameter(name, value):
            return [ '-p', name + ':=' + (json.dumps(value, ensure_ascii=False) if isinstance(value, str) else str(value).lower()) ]
        argv = ['ros2', 'run', 'my_radar_follower', 'txt_radar_bridge', '--ros-args', '--params-file',
                str(self.root / 'ros2_ws/src/my_radar_follower/config/bridge.yaml')]
        for name, value in dict(track_dir=str(runtime_paths(self.root)['track']),
            cmd_vel_topic=self.settings['cmd_vel_topic'],
            max_linear_speed_mps=self.settings['max_linear_speed_mps'],
            max_angular_speed_radps=self.settings['max_angular_speed_radps'],
            gui_managed_mode=True, gui_context_file=str(self.context),
            gui_runtime_id=self.runtime_id, status_output=str(self.control_status)).items():
            argv.extend(parameter(name, value))
        return argv

    def tick(self):
        now = self.clock()
        if self.close_requested:
            try:
                self.backend.spin()
            except Exception:
                pass  # ROS context failure must not prevent OS process cleanup.
        else:
            self.backend.spin()
        if now - self.last_heartbeat > 2.5 and not self.close_requested:
            self.shutdown('GUI heartbeat lost')
        if now - self.last_snapshot >= .2:
            try:
                self.snapshot()
            except OSError as exc:
                self.shutdown('Ownership registry write failed: ' + str(exc))
            self.last_snapshot = now
        if now - self.last_status >= .5:
            self.state(message=self.failure)
        # Process exits are lifecycle failures, not odom freshness gates.
        for name, child in list(self.owned.items()):
            if name not in self.stopping and child.process.poll() is not None:
                self.shutdown(name + ' process exited: ' + str(child.process.returncode))
                break
            if self.phase == 'following' and name not in self.stopping and getattr(child, 'unexpected_exits', []):
                self.shutdown(name + ' child process exited; manual restart required')
                break
        if self.stopping:
            name = self.stopping[0]
            child = self.owned[name]
            child.begin_stop(now)
            if name == 'follow':
                try:
                    self.backend.zero()
                    self.status = dict(self.status, linear_mps=0.0, angular_radps=0.0,
                        motion_requested=False, reason='gui_stop', published_time_sec=time.time(),
                        allow_translation=False, allow_yaw=False)
                except Exception as exc:
                    self.shutdown('Zero velocity publication failed: ' + str(exc))
            if child.tick_stop(now):
                # Publish zero for at least 0.3 s after the controller is gone.
                if name == 'follow':
                    if self.stopped_at is None:
                        self.stopped_at = now
                    if now - self.stopped_at < .3:
                        return
                    self.stopped_at = None
                del self.owned[name]
                self.stopping.pop(0)
                self.snapshot()
                if not self.stopping:
                    self.finish_stop()
            return
        if self.close_requested:
            self.done = True
            return
        if not self.want_follow:
            return
        if self.phase == 'discovering' and now >= self.deadline:
            publishers = self.backend.other_publishers()
            if publishers:
                raise RuntimeError('Existing velocity publisher: ' + ', '.join(publishers))
            self.topic_lock = acquire_topic_lock(self.backend.topic, 'runtime')
            # Do not wait for topic discovery to protect against a CLI race.
            probe = acquire_topic_lock(self.backend.topic, 'bridge')
            probe.close()
            if 'chassis' not in self.owned:
                if self.backend.chassis_present():
                    self.external.add('chassis')
                else:
                    self.spawn('chassis', ['ros2', 'launch', self.settings['chassis_package'], self.settings['chassis_launch']])
            self.deadline = now + 30
            self.state('starting_chassis', 'Waiting for chassis and odometry')
        elif self.phase == 'starting_chassis':
            if self.backend.chassis_ready():
                receiver = self.backend.receiver_state()
                if receiver == 'conflict':
                    raise RuntimeError('IPC key belongs to another project receiver')
                if 'odom' not in self.owned:
                    if receiver == 'fresh':
                        self.external.add('odom')
                    elif receiver == 'absent':
                        self.spawn('odom', ['ros2', 'run', 'my_radar_follower', 'radar_odom_receiver',
                            '--ros-args', '-p', 'odom_topic:=' + self.settings['odom_topic']])
                self.deadline = now + 10
                self.state('starting_odom', 'Waiting for this project IPC receiver')
            elif now >= self.deadline:
                raise RuntimeError('Chassis startup timed out: check launch, serial port and odom topic')
        elif self.phase == 'starting_odom':
            receiver = self.backend.receiver_state()
            if receiver == 'conflict':
                raise RuntimeError('IPC writer changed to another project')
            if receiver == 'fresh':
                if 'odom' not in self.owned:
                    self.external.add('odom')
                publishers = self.backend.other_publishers()
                if publishers:
                    raise RuntimeError('Velocity publisher appeared during startup: ' + ', '.join(publishers))
                # A zero-only publisher from a previous stop must not outlive
                # startup: destroy it before the command publisher is created.
                if getattr(self.backend, 'zero_publisher', None) is not None:
                    self.backend.node.destroy_publisher(self.backend.zero_publisher)
                    self.backend.zero_publisher = None
                self.spawn('follow', self.bridge_argv())
                self.deadline = now + 10
                self.state('starting_follow', 'Waiting for controller status')
            elif now >= self.deadline:
                raise RuntimeError('Odom receiver startup timed out or IPC writer is unverified')
        elif self.phase in ('starting_follow', 'following'):
            try:
                status = json.loads(self.control_status.read_text(encoding='utf-8'))
                valid = (status.get('runtime_id') == self.runtime_id and
                         status.get('pid') in [p['pid'] for p in self.owned['follow'].identities] and
                         -.05 <= time.time()-status['published_time_sec'] <= 1.0)
            except (OSError, ValueError, TypeError, KeyError):
                valid = False
            if valid:
                if self.phase == 'starting_follow' and not self.backend.chassis_ready():
                    if now >= self.deadline:
                        raise RuntimeError('Chassis/odometry lost readiness before controller activation')
                    if now - self.last_context_write >= .2:self.write_context()
                    return
                activated = self.phase == 'starting_follow'
                if self.phase == 'starting_follow':
                    for child in self.owned.values():
                        if hasattr(child, 'unexpected_exits'):
                            child.unexpected_exits.clear()
                changed = status != self.status or self.phase != 'following'
                self.status = status
                self.deadline = now + max(0, 1-(time.time()-status['published_time_sec']))
                self.phase = 'following'
                if changed:self.state('following')
                if activated:self.write_context()
            elif now >= self.deadline:
                raise RuntimeError('Controller status expired or not available')
        if now - self.last_context_write >= .2:
            self.write_context()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--runtime-dir', required=True)
    parser.add_argument('--settings', required=True)
    args = parser.parse_args()
    root = verify_loaded_package(__file__)
    settings = validate_settings(json.loads(args.settings))
    os.environ['WHEELTEC_SETUP'] = settings['chassis_setup']
    folder = Path(args.runtime_dir).resolve()
    expected_parent = runtime_paths(root)['track'] / 'gui_runtime'
    if folder.parent != expected_parent.resolve():
        raise ValueError('Runtime folder must belong to this project')
    import rclpy
    rclpy.init(args=[])
    backend = RosBackend(root, settings)
    commands = queue.Queue(maxsize=100)
    logs = queue.Queue(maxsize=1000)

    def emit(kind, **value):
        print(json.dumps(dict(type=kind, **value), ensure_ascii=False), flush=True)

    def reader():
        try:
            for line in sys.stdin:
                try:
                    commands.put(json.loads(line), timeout=.5)
                except (ValueError, queue.Full):
                    pass
        finally:
            commands.put({'action': 'shutdown'})

    def factory(name, argv, cwd, output):
        child = OwnedProcess(name, argv, cwd, output)

        def read_output():
            for line in child.process.stdout:
                try:
                    logs.put_nowait(dict(component=name, message=line.rstrip()))
                except queue.Full:
                    pass
        threading.Thread(target=read_output, daemon=True).start()
        return child

    runtime = Runtime(root, folder, settings, backend, emit, factory)
    threading.Thread(target=reader, daemon=True).start()
    termination = [False]
    def terminate(*_):
        termination[0] = True
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, terminate)
    try:
        while not runtime.done:
            try:
                if termination[0] and not runtime.close_requested:
                    runtime.shutdown()
                for _ in range(100):
                    try:
                        runtime.command(commands.get_nowait())
                    except queue.Empty:
                        break
                for _ in range(100):
                    try:
                        emit('log', **logs.get_nowait())
                    except queue.Empty:
                        break
                runtime.tick()
            except Exception as exc:
                emit('log', component='runtime', message=str(exc))
                runtime.shutdown(str(exc))
            time.sleep(.05)
    finally:
        # Ordinary exceptions and GUI EOF still revoke the bridge lease.
        runtime.want_follow = False
        try:
            runtime.write_context()
        except OSError:
            pass
        backend.close()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
