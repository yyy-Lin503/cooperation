"""Exercise record/file/command behaviour with mock ROS messages, no ROS install."""

import ast
from datetime import datetime
import math
import json
import os
from pathlib import Path
import struct
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch


class Message:
    def __init__(self):
        self.linear = SimpleNamespace(x=0.0)
        self.angular = SimpleNamespace(z=0.0)
        self.data = 0


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class MockNode:
    def get_logger(self):
        return SimpleNamespace(info=lambda text: None, error=lambda text: None)


def load_bridge_definitions():
    source = (Path(__file__).resolve().parents[2] / 'ros2_ws' / 'src' / 'my_radar_follower' /
              'my_radar_follower' / 'txt_tracker_bridge.py')
    tree = ast.parse(source.read_text(encoding='utf-8'))
    selected = [n for n in tree.body if isinstance(
        n, (ast.Assign, ast.FunctionDef, ast.ClassDef))]
    namespace = dict(Node=MockNode, Twist=Message, Int8=Message, Path=Path,
                     datetime=datetime, math=math, os=os, time=time, struct=struct, json=json)
    exec(compile(ast.Module(body=selected, type_ignores=[]), str(source), 'exec'), namespace)
    return namespace


BRIDGE = load_bridge_definitions()


def record(frame=1, state='TRACKING', measured=1, lost=0, scale=1.0,
           move=1, yaw=1, mode='realtime', stamp=None, target=1):
    stamp = stamp or datetime.now().isoformat(timespec='milliseconds')
    measurement = ['0.6', '3.0', '3.06', '11.3'] if measured else ['nan'] * 4
    return ' '.join([str(target), '0.6', '3.0', '3.06', '11.3', *measurement,
                     str(measured), '0.9', str(frame), str(lost), '8', stamp, mode,
                     state, str(scale), str(move), str(yaw), '0'])


class BridgeRuntimeTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.path = Path(self.temp.name) / 'track_realtime_test.txt'
        self.node = BRIDGE['TxtRadarBridge'].__new__(BRIDGE['TxtRadarBridge'])
        self.node.target_dir = self.path.parent
        self.node.timeout = 0.5
        self.node.max_linear = 0.8
        self.node.max_angular = 0.8
        self.node.pid = BRIDGE['SimplePID'](2.0)
        self.node.last_key = None
        self.node.last_new_frame_time = None
        self.node.last_command = (0.0, 0.0)
        self.node.stop_reason = None
        self.node.cmd_publisher = Publisher()
        self.node.flag_publisher = Publisher()

    def write(self, line, newline=True):
        self.path.write_text(line + ('\n' if newline else ''), encoding='utf-8')

    def moving(self):
        self.write(record(frame=1))
        self.node.timer_callback()
        self.write(record(frame=2))
        self.node.timer_callback()
        self.assertGreater(self.last().linear.x, 0)

    def last(self):
        return self.node.cmd_publisher.messages[-1]

    def assert_stopped(self):
        self.assertEqual((self.last().linear.x, self.last().angular.z), (0.0, 0.0))

    def test_tracking_pid_signs_and_limits(self):
        self.moving()
        self.assertEqual(self.last().linear.x, 0.8)
        self.assertEqual(self.last().angular.z, -0.8)

    def test_prediction_scale_and_nan_measurements(self):
        self.moving()
        self.write(record(frame=3, state='PREDICTED', measured=0, lost=2, scale=0.4))
        self.node.timer_callback()
        self.assertAlmostEqual(self.last().linear.x, 0.32)
        self.assertAlmostEqual(self.last().angular.z, -0.32)

    def test_stopped_overrides_recent_position(self):
        self.moving()
        self.write(record(frame=3, state='STOPPED', lost=31, scale=0, move=0, yaw=0))
        self.node.timer_callback()
        self.assert_stopped()

    def test_motion_permissions(self):
        self.moving()
        self.write(record(frame=3, move=0, yaw=0))
        self.node.timer_callback()
        self.assert_stopped()

    def test_missing_file_stops(self):
        self.node.last_command = (0.5, 0.1)
        self.node.timer_callback()
        self.assert_stopped()

    def test_invalid_record_stops(self):
        self.moving()
        self.write('1 2 3')
        self.node.timer_callback()
        self.assert_stopped()

    def test_old_timestamp_cannot_be_refreshed_by_file_write(self):
        self.moving()
        stamp = datetime.fromtimestamp(time.time() - 5).isoformat()
        self.write(record(frame=3, stamp=stamp))
        self.node.timer_callback()
        self.assert_stopped()

    def test_duplicate_does_not_advance_pid_or_observation_time(self):
        self.moving()
        last_time = self.node.pid.last_time
        last_new = self.node.last_new_frame_time
        self.node.timer_callback()
        self.assertEqual(self.node.pid.last_time, last_time)
        self.assertEqual(self.node.last_new_frame_time, last_new)
        self.assertEqual(self.last().linear.x, 0.8)

    def test_rewritten_duplicate_expires(self):
        self.moving()
        self.node.last_new_frame_time -= 1.0
        self.write(record(frame=2))
        self.node.timer_callback()
        self.assert_stopped()
        self.assertEqual(self.node.stop_reason, 'repeated_frame_timeout')

    def test_frame_rollback_stops(self):
        self.moving()
        self.write(record(frame=1))
        self.node.timer_callback()
        self.assert_stopped()

    def test_new_file_session_resets_pid(self):
        self.moving()
        self.path = self.path.parent / 'track_realtime_new_session.txt'
        self.write(record(frame=1))
        # Avoid filesystem timestamp tie on slower filesystems.
        os.utime(self.path, (time.time() + 0.01, time.time() + 0.01))
        self.node.timer_callback()
        self.assert_stopped()
        self.assertEqual(self.node.last_key[2], 1)

    def test_partial_final_record_is_not_accepted(self):
        self.write(record(frame=1) + '\n1 2', newline=False)
        line = self.node.read_last_complete_record(self.path)
        self.assertEqual(BRIDGE['parse_tracking_record'](line)['frame'], 1)

    def test_offline_record_stops(self):
        self.moving()
        self.write(record(frame=3, mode='offline'))
        self.node.timer_callback()
        self.assert_stopped()

    def test_future_timestamp_stops(self):
        self.moving()
        stamp = datetime.fromtimestamp(time.time() + 5).isoformat()
        self.write(record(frame=3, stamp=stamp))
        self.node.timer_callback()
        self.assert_stopped()

    def metadata(self, **overrides):
        now = time.time()
        result = dict(schema_version=1, frame=2, target_id=1, session_id="1", selection_epoch="1", uid="1",
                      received_time_sec=now, sample_time_sec=now, last_measurement_time_sec=now,
                      force_stop=False, motion_reason="accepted_measurement")
        result.update(overrides)
        self.path.with_name(self.path.name + ".meta.json").write_text(json.dumps(result))

    def test_stale_sensor_cannot_be_refreshed_by_new_txt(self):
        self.moving()
        self.metadata(received_time_sec=time.time() - 2)
        self.node.timer_callback()
        self.assert_stopped()

    def test_visual_evidence_expires_even_while_radar_is_fresh(self):
        self.moving()
        now = time.time()
        guard = dict(enabled=True, permit=True, captured_time_sec=now-0.3,
                     expires_time_sec=now+0.05, camera_session="4", camera_track="2",
                     radar_session="1", selection_epoch="1")
        self.metadata(vision_guard=guard)
        self.node.timer_callback()
        self.assertGreater(self.last().linear.x, 0)
        with patch.object(time, "time", return_value=now+0.1):
            self.node.timer_callback()
        self.assert_stopped()

    def reject_visual_metadata(self, **overrides):
        self.moving()
        now = time.time()
        guard = dict(enabled=True, permit=True, captured_time_sec=now,
                     expires_time_sec=now+0.2, camera_session="4", camera_track="2",
                     radar_session="1", selection_epoch="1")
        guard.update(overrides)
        self.metadata(vision_guard=guard)
        self.node.timer_callback()
        self.assert_stopped()

    def test_visual_permission_false_stops(self):
        self.reject_visual_metadata(permit=False)

    def test_visual_selection_mismatch_stops(self):
        self.reject_visual_metadata(selection_epoch="2")

    def test_visual_invalid_identity_stops(self):
        self.reject_visual_metadata(camera_track="0")

    def test_visual_nan_expiry_stops(self):
        self.reject_visual_metadata(expires_time_sec=float("nan"))

    def test_metadata_frame_mismatch_stops(self):
        self.moving()
        self.metadata(frame=3)
        self.node.timer_callback()
        self.assert_stopped()

    def test_producer_shutdown_stops_duplicate_immediately(self):
        self.moving()
        self.metadata(force_stop=True, motion_reason="capture_stopped")
        self.node.timer_callback()
        self.assert_stopped()
        self.assertIn("capture_stopped", self.node.last_control_output["reason"])

    def test_old_measurement_is_not_refreshed_by_prediction(self):
        self.moving()
        self.write(record(frame=3, state="PREDICTED", measured=0, lost=12, scale=0.4))
        self.metadata(frame=3, last_measurement_time_sec=time.time() - 1.2)
        self.node.timer_callback()
        self.assert_stopped()
        self.assertIn("measurement age",self.node.last_control_output["reason"])

    def test_atomic_snapshot_wins_over_old_txt(self):
        self.moving()
        self.metadata(frame=3, control_snapshot_version=1, record_line=record(frame=3))
        self.node.timer_callback()
        self.assertEqual(self.node.current_record["frame"],3)
        self.metadata(frame=4, control_snapshot_version=1, record_line=record(frame=4))
        self.node.timer_callback()
        self.assertGreater(self.last().linear.x,0)
        self.assertEqual(self.node.current_record["frame"],4)

    def test_prediction_without_metadata_expires(self):
        self.moving()
        self.write(record(frame=3,state="PREDICTED",measured=0,lost=4,scale=.35))
        self.node.timer_callback()
        self.assert_stopped()

    def test_selection_epoch_resets_controller_with_stable_display_id(self):
        self.moving()
        self.metadata()
        self.node.timer_callback()
        self.write(record(frame=3))
        self.metadata(frame=3, selection_epoch="2")
        self.node.timer_callback()
        self.assert_stopped()

    def test_receive_clock_regression_stops_even_with_recent_txt(self):
        self.moving()
        now = time.time()
        self.metadata(received_time_sec=now)
        self.node.timer_callback()
        self.metadata(received_time_sec=now-0.2)
        self.node.timer_callback()
        self.assert_stopped()
        self.assertIn("backwards", self.node.stop_reason)

    def test_metadata_disappearance_and_time_rollback_stop(self):
        self.moving()
        self.metadata()
        self.node.timer_callback()
        self.path.with_name(self.path.name + ".meta.json").unlink()
        self.node.timer_callback()
        self.assert_stopped()
        self.metadata(received_time_sec=time.time()+1)
        self.node.timer_callback()
        self.assert_stopped()

    def test_controller_does_not_write_shared_memory(self):
        # IPC now belongs exclusively to the independent odom receiver.
        self.assertFalse(hasattr(self.node, 'odom_callback'))


class GuiManagedBridgeTest(unittest.TestCase):
    write = BridgeRuntimeTest.write
    metadata = BridgeRuntimeTest.metadata
    last = BridgeRuntimeTest.last
    assert_stopped = BridgeRuntimeTest.assert_stopped

    def setUp(self):
        BridgeRuntimeTest.setUp(self)
        self.node.gui_managed = True
        self.node.gui_runtime_id = 'this-gui'
        self.context = self.path.parent / 'context.json'
        self.node.gui_context_file = str(self.context)
        self.context_write()

    def context_write(self, **values):
        context = dict(runtime_id='this-gui', permit=True, session_id='1', lease_monotonic=time.monotonic())
        context.update(values)
        self.context.write_text(json.dumps(context))

    def frame(self, number, **metadata):
        self.write(record(frame=number))
        self.metadata(frame=number, **metadata)
        self.node.timer_callback()

    def moving(self):
        self.frame(1);self.frame(2)
        self.assertGreater(self.last().linear.x,0)

    def test_no_capture_waits_even_with_fresh_old_files(self):
        self.context_write(session_id='0')
        self.frame(1)
        self.assert_stopped()
        self.assertEqual(self.node.stop_reason,'waiting_for_capture')

    def test_no_target_is_a_waiting_state(self):
        self.node.timer_callback()
        self.assertEqual(self.node.stop_reason,'waiting_for_target')
        self.assert_stopped()

    def test_only_current_gui_session_can_move(self):
        self.frame(1,session_id='2')
        self.assertEqual(self.node.stop_reason,'waiting_for_target')
        self.moving()

    def test_newer_other_session_cannot_override_active_gui_file(self):
        self.moving()
        original=self.path
        self.path=original.parent/'track_realtime_other.txt'
        self.write(record(frame=8));self.metadata(frame=8,session_id='2')
        os.utime(self.path,(time.time()+.01,time.time()+.01))
        self.path=original
        self.node.timer_callback()
        self.assertGreater(self.last().linear.x,0)
        self.assertEqual(self.node.current_record['frame'],2)

    def test_context_revocation_stops_duplicate_immediately(self):
        self.moving();self.context_write(permit=False)
        self.node.timer_callback();self.assert_stopped()
        self.assertEqual(self.node.stop_reason,'gui_paused')

    def test_runtime_death_or_bad_context_stops(self):
        self.moving();self.context_write(lease_monotonic=time.monotonic()-2)
        self.node.timer_callback();self.assert_stopped()
        self.assertEqual(self.node.stop_reason,'gui_runtime_expired')
        self.context_write(runtime_id='another-gui')
        self.node.timer_callback();self.assert_stopped()
        self.context.unlink();self.node.timer_callback();self.assert_stopped()

    def test_missing_companion_metadata_cannot_use_legacy_fallback(self):
        self.write(record(frame=1));self.node.timer_callback()
        self.assertEqual(self.node.stop_reason,'waiting_for_target')
        self.assert_stopped()

    def test_session_switch_cannot_reuse_previous_command(self):
        self.moving();self.context_write(session_id='2')
        self.node.timer_callback();self.assert_stopped()

    def test_visual_guard_still_overrides_managed_motion(self):
        self.moving()
        self.metadata(vision_guard=dict(enabled=True,permit=False))
        self.node.timer_callback();self.assert_stopped()

    def test_optional_status_is_written_atomically(self):
        import sys
        sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'ros2_ws/src/my_radar_follower'))
        BRIDGE['__package__']='my_radar_follower'
        status=self.path.parent/'control.json'
        self.node.status_output=str(status)
        self.node.resolved_cmd_topic='/cmd_vel'
        self.node.timer_callback()
        data=json.loads(status.read_text())
        self.assertEqual(data['reason'],'waiting_for_target')
        self.assertEqual(data['runtime_id'],'this-gui')
        self.assertEqual(data['pid'],os.getpid())
        self.assertFalse(status.with_name('control.json.tmp').exists())


if __name__ == '__main__':
    unittest.main()
