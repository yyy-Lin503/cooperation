"""Exercise GUI lifecycle orchestration without ROS, hardware or real commands."""
from pathlib import Path
import json
import os
import sys
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'ros2_ws/src/my_radar_follower'))
from my_radar_follower.gui_runtime import Runtime, RosBackend
from my_radar_follower.gui_runtime_support import validate_settings, acquire_topic_lock


class Clock:
    now = 100.0
    def __call__(self): return self.now


class Backend:
    def __init__(self):
        self.topic = '/gui_runtime_test'
        self.present = False
        self.fresh = False
        self.receiver = 'absent'
        self.publishers = []
        self.zeros = 0
    def spin(self): pass
    def chassis_present(self): return self.present
    def chassis_ready(self): return self.present and self.fresh
    def odom_fresh(self): return self.fresh
    def receiver_state(self): return self.receiver
    def other_publishers(self): return self.publishers
    def zero(self): self.zeros += 1


class Child:
    def __init__(self, name, argv, cwd, output, history):
        self.name, self.argv = name, argv
        self.process = SimpleNamespace(pid=1000+len(history), returncode=None)
        self.process.poll = lambda: self.process.returncode
        self.identities = [dict(pid=self.process.pid, start='1')]
        self.history, self.stopping = history, False
        history.append(('spawn', name, argv))
    def refresh(self): pass
    def begin_stop(self, now):
        if not self.stopping:
            self.history.append(('stop', self.name))
            self.stopping = True
            self.process.returncode = 0
    def tick_stop(self, now): return True


class RuntimeTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.clock, self.backend = Clock(), Backend()
        self.history, self.events = [], []
        self.lock_patch = patch('my_radar_follower.gui_runtime.acquire_topic_lock',
                                side_effect=lambda *args: SimpleNamespace(close=lambda: None))
        self.lock_patch.start(); self.addCleanup(self.lock_patch.stop)
        settings = dict(chassis_package='wheeltec', chassis_launch='robot.launch.py',
                        odom_topic='/odom', cmd_vel_topic='/cmd_vel',
                        max_linear_speed_mps=.15, max_angular_speed_radps=.25)
        self.runtime = Runtime(self.root, self.root/'session', settings, self.backend,
            lambda kind, **data: self.events.append(dict(type=kind, **data)),
            lambda *args: Child(*args, self.history), self.clock)

    def tick(self, delta=.1):
        self.clock.now += delta
        self.runtime.command(dict(action='heartbeat'))
        self.runtime.tick()

    def start(self, external=False):
        self.backend.present = external
        self.backend.fresh = external
        self.backend.receiver = 'fresh' if external else 'absent'
        self.runtime.command(dict(action='start'))
        self.tick(1.1)
        self.backend.present = self.backend.fresh = True
        self.tick()
        self.backend.receiver = 'fresh'
        self.tick()
        self.report()
        self.tick()

    def report(self, reason='waiting_for_capture'):
        child = self.runtime.owned['follow']
        self.runtime.control_status.write_text(json.dumps(dict(
            runtime_id=self.runtime.runtime_id, pid=child.process.pid,
            published_time_sec=time.time(), reason=reason)))

    def test_start_order_and_session_wait(self):
        self.start()
        self.assertEqual([x[1] for x in self.history], ['chassis','odom','follow'])
        self.assertEqual(self.runtime.phase, 'following')
        context = json.loads(self.runtime.context.read_text())
        self.assertTrue(context['permit'])
        self.assertEqual(context['session_id'], '0')
        self.runtime.command(dict(action='session', session_id='1234567890123456789'))
        self.assertEqual(json.loads(self.runtime.context.read_text())['session_id'], '1234567890123456789')
        argv = self.runtime.owned['follow'].argv
        self.assertIn('max_linear_speed_mps:=0.15', argv)
        self.assertIn('gui_managed_mode:=true', argv)

    def test_duplicate_start_and_external_reuse(self):
        self.start(external=True)
        self.runtime.command(dict(action='start')); self.tick()
        self.assertEqual(list(self.runtime.owned), ['follow'])
        self.assertEqual(self.runtime.external, {'chassis','odom'})
        self.assertEqual(len(self.history), 1)

    def test_stop_preserves_dependencies_then_can_start_again(self):
        self.start()
        self.runtime.command(dict(action='stop_follow'))
        self.tick(); self.tick(.4)
        self.assertEqual(set(self.runtime.owned), {'chassis','odom'})
        self.assertFalse(json.loads(self.runtime.context.read_text())['permit'])
        self.assertGreater(self.backend.zeros, 0)
        self.runtime.command(dict(action='start')); self.tick(1.1); self.tick(); self.tick(); self.report(); self.tick()
        self.assertEqual(self.runtime.phase, 'following')
        self.assertEqual([x[1] for x in self.history if x[0]=='spawn'], ['chassis','odom','follow','follow'])

    def test_shutdown_order_and_external_processes_not_stopped(self):
        self.start(); self.runtime.shutdown()
        for _ in range(8): self.tick(.4)
        self.assertTrue(self.runtime.done)
        self.assertEqual([x[1] for x in self.history if x[0]=='stop'], ['follow','odom','chassis'])
        self.assertFalse(json.loads(self.runtime.registry.read_text())['owned'])

    def test_external_shutdown_only_stops_owned_follow(self):
        self.start(external=True); self.runtime.shutdown()
        self.tick(); self.tick(.4)
        self.assertTrue(self.runtime.done)
        self.assertEqual([x[1] for x in self.history if x[0]=='stop'], ['follow'])

    def test_conflicting_publisher_is_not_killed_or_sent_zero(self):
        self.backend.publishers=['/another_controller']
        self.runtime.command(dict(action='start'))
        with self.assertRaisesRegex(RuntimeError, 'Existing velocity'):
            self.tick(1.1)
        self.runtime.shutdown('conflict'); self.tick()
        self.assertFalse(self.history)
        self.assertEqual(self.backend.zeros, 0)

    def test_new_publisher_during_startup_is_refused(self):
        self.runtime.command(dict(action='start')); self.tick(1.1)
        self.backend.present=self.backend.fresh=True; self.tick()
        self.backend.receiver='fresh'; self.backend.publishers=['/late_controller']
        with self.assertRaisesRegex(RuntimeError, 'appeared'):
            self.tick()
        self.assertNotIn('follow', self.runtime.owned)

    def test_startup_timeout_and_foreign_ipc(self):
        self.runtime.command(dict(action='start')); self.tick(1.1)
        with self.assertRaisesRegex(RuntimeError, 'Chassis startup timed out'):
            self.tick(31)
        self.assertNotIn('follow', self.runtime.owned)

    def test_foreign_receiver_is_not_overwritten(self):
        self.backend.present=self.backend.fresh=True; self.backend.receiver='conflict'
        self.runtime.command(dict(action='start')); self.tick(1.1)
        with self.assertRaisesRegex(RuntimeError, 'another project'):
            self.tick()
        self.assertNotIn('odom', self.runtime.owned)

    def test_child_exit_shuts_down_without_auto_restart(self):
        self.start(); self.runtime.owned['odom'].process.returncode=7
        self.tick()
        self.assertFalse(self.runtime.want_follow)
        for _ in range(8):self.tick(.4)
        self.assertTrue(self.runtime.done)
        self.assertEqual(len([x for x in self.history if x[0]=='spawn']), 3)

    def test_stale_odom_is_diagnostic_only_while_running(self):
        self.start(); self.backend.fresh=False; self.tick(.6)
        self.assertTrue(self.runtime.want_follow)
        self.assertEqual(self.runtime.phase,'following')
        self.assertFalse(self.events[-1]['odom_fresh'])

    def test_owned_launch_child_exit_also_stops_following(self):
        self.start()
        self.runtime.owned['chassis'].unexpected_exits=[dict(pid=12345,start='1')]
        self.tick()
        self.assertFalse(self.runtime.want_follow)
        self.assertIn('child process exited',self.runtime.failure)

    def test_gui_heartbeat_loss_revokes_control(self):
        self.start(); self.clock.now += 3; self.runtime.tick()
        self.assertFalse(self.runtime.want_follow)
        self.assertFalse(json.loads(self.runtime.context.read_text())['permit'])

    def test_ros_context_failure_cannot_prevent_process_cleanup(self):
        self.start()
        def broken():raise RuntimeError('ROS context unavailable')
        self.backend.spin=broken;self.backend.zero=broken
        self.runtime.shutdown('ROS context unavailable')
        for _ in range(8):self.tick(.4)
        self.assertTrue(self.runtime.done)
        self.assertFalse(self.runtime.owned)
        self.assertIn('Zero velocity publication failed',self.runtime.failure)

    def test_stop_during_startup_does_not_start_controller(self):
        self.runtime.command(dict(action='start')); self.tick(1.1)
        self.runtime.command(dict(action='stop_follow')); self.backend.present=self.backend.fresh=True
        self.tick()
        self.assertEqual(set(self.runtime.owned), {'chassis'})

    def test_readiness_must_hold_at_initial_controller_activation(self):
        self.runtime.command(dict(action='start'));self.tick(1.1)
        self.backend.present=self.backend.fresh=True;self.tick()
        self.backend.receiver='fresh';self.tick();self.report()
        self.backend.fresh=False;self.tick()
        self.assertEqual(self.runtime.phase,'starting_follow')
        self.assertFalse(json.loads(self.runtime.context.read_text())['permit'])
        self.backend.fresh=True;self.tick()
        self.assertEqual(self.runtime.phase,'following')
        self.assertTrue(json.loads(self.runtime.context.read_text())['permit'])


class SettingsTest(unittest.TestCase):
    def test_paths_spaces_and_invalid_values(self):
        with tempfile.TemporaryDirectory() as folder:
            setup=Path(folder)/'setup with spaces.bash';setup.touch()
            settings=dict(chassis_setup=str(setup),chassis_package='wheeltec',chassis_launch='robot.launch.py',
                chassis_node='wheeltec_robot',odom_topic='/odom',cmd_vel_topic='/cmd_vel',
                max_linear_speed_mps=.15,max_angular_speed_radps=.25)
            self.assertEqual(validate_settings(settings)['chassis_setup'],str(setup.resolve()))
            for key,value in [('chassis_package','a;bad'),('cmd_vel_topic','cmd_vel'),
                              ('max_linear_speed_mps',float('nan')),('max_angular_speed_radps',-1)]:
                with self.assertRaises(ValueError):validate_settings(dict(settings,**{key:value}))

    @unittest.skipUnless(sys.platform.startswith('linux'),'Linux topic file lock')
    def test_topic_lock_is_shared_across_project_directories(self):
        topic='/test_'+str(os.getpid())
        with patch.dict(os.environ,{'ROS_DOMAIN_ID':'0'}):first=acquire_topic_lock(topic,'bridge')
        try:
            with patch.dict(os.environ,{'ROS_DOMAIN_ID':'000'}):
                with self.assertRaises(RuntimeError):acquire_topic_lock(topic,'bridge')
        finally:first.close()
        acquire_topic_lock(topic,'bridge').close()


class BackendGraphTest(unittest.TestCase):
    def test_ambiguous_chassis_and_own_zero_publisher(self):
        backend=RosBackend.__new__(RosBackend)
        backend.settings=dict(chassis_node='wheeltec_robot')
        backend.node=SimpleNamespace(get_node_names_and_namespaces=lambda:[('wheeltec_robot','/'),('wheeltec_robot','/other')])
        with self.assertRaisesRegex(RuntimeError,'Multiple'):backend.chassis_present()
        backend.topic='/cmd_vel'
        backend.node=SimpleNamespace(get_name=lambda:'owned_manager',get_publishers_info_by_topic=lambda _: [
            SimpleNamespace(node_name='owned_manager',node_namespace='/'),
            SimpleNamespace(node_name='external_driver',node_namespace='/robot')])
        self.assertEqual(backend.other_publishers(),['/robot/external_driver'])

    @unittest.skipUnless(sys.platform.startswith('linux'),'Inspect real Linux process metadata')
    def test_receiver_reuse_requires_exact_project_not_a_prefix_match(self):
        import subprocess
        from my_radar_follower.odom_transport import pack
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder)/'V8';root.mkdir()
            other=Path(folder)/'V81';other.mkdir()
            backend=RosBackend.__new__(RosBackend);backend.root=root
            for directory,expected in ((root,'fresh'),(other,'conflict')):
                script=directory/'radar_odom_receiver.py';script.write_text('import time;print("ready",flush=True);time.sleep(60)')
                environment=dict(os.environ);environment.pop('RADAR_PROJECT_ROOT',None)
                process=subprocess.Popen([sys.executable,str(script)],env=environment,stdout=subprocess.PIPE,text=True)
                try:
                    self.assertEqual(process.stdout.readline().strip(),'ready')
                    packet=pack(time.time(),0,0,1,process.pid)
                    memory=SimpleNamespace(read=lambda _:packet,detach=lambda:None)
                    backend.ipc=SimpleNamespace(SharedMemory=lambda *a,**k:memory,Error=RuntimeError)
                    self.assertEqual(backend.receiver_state(),expected)
                finally:process.terminate();process.wait(timeout=3);process.stdout.close()


if __name__=='__main__':unittest.main()
