#!/usr/bin/env python3
"""21-column tracking protocol adapter and proportional motion controller."""

from datetime import datetime
from pathlib import Path
import math
import json
import os
import struct
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Int8

PREDICTED_MAX_LOST_FRAMES = 30


def parse_tracking_record(line):
    """Validate the exact protocol written by appendLockedTargetToTxt."""
    fields = line.split()
    if len(fields) != 21:
        raise ValueError("Expected 21 tracking fields")
    target_id = int(fields[0])
    x, y, distance, angle = (float(v) for v in fields[1:5])
    measured = int(fields[9])
    confidence = float(fields[10])
    frame, lost, stable = (int(v) for v in fields[11:14])
    state = fields[16]
    scale = float(fields[17])
    allow_translation, allow_yaw = (int(v) for v in fields[18:20])
    hits = int(fields[20])
    if not all(math.isfinite(v) for v in (x, y, distance, angle, confidence, scale)):
        raise ValueError("Non-finite tracking state")
    if target_id <= 0 or frame < 0 or min(lost, stable, hits) < 0 or distance < 0:
        raise ValueError("Invalid IDs, frame counters or distance")
    if measured not in (0, 1) or allow_translation not in (0, 1) or allow_yaw not in (0, 1):
        raise ValueError("Invalid boolean motion fields")
    if not 0.0 <= confidence <= 1.0 or not 0.0 <= scale <= 1.0:
        raise ValueError("Invalid confidence or speed scale")
    if state not in ("TRACKING", "PREDICTED", "STOPPED"):
        raise ValueError("Unknown control state")
    if state == "TRACKING" and (not measured or lost != 0):
        raise ValueError("TRACKING requires a current accepted measurement")
    if state == "PREDICTED" and lost > PREDICTED_MAX_LOST_FRAMES:
        raise ValueError("Prediction exceeds the retained 30-frame policy")
    if state == "STOPPED" and (scale != 0 or allow_translation or allow_yaw):
        raise ValueError("STOPPED cannot grant motion")
    if fields[15] not in ("realtime", "offline"):
        raise ValueError("Unknown acquisition mode")
    measurements = tuple(float(v) for v in fields[5:9])
    if measured and not all(math.isfinite(v) for v in measurements):
        raise ValueError("Non-finite accepted measurement")
    return {
        "target_id": target_id, "x": x, "y": y, "range": distance, "angle": angle,
        "has_measurement": bool(measured), "confidence": confidence, "frame": frame,
        "lost_frames": lost, "stable_frames": stable, "timestamp": fields[14],
        "mode": fields[15], "control_state": state, "speed_scale": scale,
        "allow_translation": bool(allow_translation), "allow_yaw": bool(allow_yaw),
        "reacquire_hits": hits,
    }


def validate_frame_metadata(metadata, record, now, timeout):
    """Validate companion metadata without changing the 21-column protocol.

    Freshness applies to sensor input and last actual measurement. V1024-2158
    embeds the 21-column line in this atomic metadata file, preventing mixed frames.
    """
    if metadata.get("schema_version") != 1:
        raise ValueError("Unknown frame metadata schema")
    if metadata.get("force_stop"):
        raise ValueError("Producer stopped: " + str(metadata.get("motion_reason", "stopped")))
    if metadata.get("frame") != record["frame"] or metadata.get("target_id") != record["target_id"]:
        raise ValueError("Frame metadata does not match TXT record")
    for key in ("received_time_sec", "sample_time_sec", "last_measurement_time_sec"):
        value = metadata.get(key)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value < 0:
            raise ValueError("Invalid frame metadata time")
    received = metadata["received_time_sec"]
    age = now - received
    if received <= 0 or age < -0.05 or age > timeout:
        raise ValueError("Sensor frame is stale or from the future")
    last = metadata["last_measurement_time_sec"]
    if last > now + 0.05:
        raise ValueError("Accepted measurement time is from the future")
    if record["has_measurement"] and (last <= 0 or last != metadata["sample_time_sec"]):
        raise ValueError("Accepted measurement timestamp mismatch")
    for key in ("session_id", "selection_epoch", "uid"):
        if int(metadata.get(key, -1)) < 0:
            raise ValueError("Invalid frame metadata identity")
    vision = metadata.get("vision_guard")
    if vision is not None:
        if not isinstance(vision, dict) or type(vision.get("enabled")) is not bool:
            raise ValueError("Invalid visual guard metadata")
        if vision["enabled"]:
            if vision.get("permit") is not True:
                raise ValueError("Visual consistency blocked motion")
            captured, expires = vision.get("captured_time_sec"), vision.get("expires_time_sec")
            for value in (captured, expires):
                if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                    raise ValueError("Invalid visual evidence time")
            if captured <= 0 or now-captured < -0.05 or now-captured > 0.5 or now >= expires:
                raise ValueError("Visual evidence expired")
            if expires > captured+0.55 or expires > received+0.4:
                raise ValueError("Invalid visual evidence expiry")
            if int(vision.get("camera_session", 0)) <= 0 or int(vision.get("camera_track", 0)) <= 0:
                raise ValueError("Invalid visual identity")
            if (int(vision.get("radar_session", -1)) != int(metadata["session_id"]) or
                    int(vision.get("selection_epoch", -1)) != int(metadata["selection_epoch"])):
                raise ValueError("Visual selection does not match radar selection")
    measurement_age = max(0.0, now-last) if last > 0 else None
    if (not record["has_measurement"] and (record["allow_translation"] or record["allow_yaw"])
            and (measurement_age is None or measurement_age > 0.35)):
        raise ValueError("Prediction measurement age exceeded 0.35s")
    return measurement_age


class SimplePID:
    """Proportional controller with angular and distance deadbands (I=D=0)."""

    def __init__(self, distance, angular_p=7.0, linear_p=1.2):
        self.target = (0.0, distance)
        self.kp = (angular_p, linear_p)
        self.reset()

    def reset(self):
        self.last_time = None

    def update(self, angle, distance, now):
        if self.last_time is None:
            self.last_time = now
            return 0.0, 0.0
        self.last_time = now
        angular_error = self.target[0] - angle
        distance_error = self.target[1] - distance
        if abs(angular_error) < 0.1:
            angular_error = 0.0
        if abs(distance_error) < 0.1 or abs(distance_error) > 15.0:
            distance_error = 0.0
        if distance_error > 0 and self.target[1] < 1.2:
            distance_error *= (1.2 / self.target[1]) * 0.7
        return self.kp[0] * angular_error, -self.kp[1] * distance_error


class TxtRadarBridge(Node):
    def __init__(self):
        from .runtime_support import verify_loaded_package
        verify_loaded_package(__file__)
        super().__init__("txt_radar_bridge")
        defaults = {
            "track_dir": os.environ.get("RADAR_TRACK_DIR", str(Path.cwd() / "track_txt")),
            "cmd_vel_topic": "/cmd_vel", "odom_topic": "/odom",
            "timer_period_sec": 0.05, "data_timeout_sec": 0.5,
            "desired_distance_m": 2.0, "angular_kp": 7.0, "linear_kp": 1.2,
            "max_linear_speed_mps": 0.8, "max_angular_speed_radps": 0.8,
            "gui_managed_mode": False, "gui_context_file": "",
            "gui_runtime_id": "", "status_output": "",
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.target_dir = Path(self.get_parameter("track_dir").value).expanduser().resolve()
        self.timeout = float(self.get_parameter("data_timeout_sec").value)
        self.max_linear = float(self.get_parameter("max_linear_speed_mps").value)
        self.max_angular = float(self.get_parameter("max_angular_speed_radps").value)
        period = float(self.get_parameter("timer_period_sec").value)
        desired = float(self.get_parameter("desired_distance_m").value)
        parameter_values = (self.timeout, self.max_linear, self.max_angular, period, desired,
                            float(self.get_parameter("angular_kp").value),
                            float(self.get_parameter("linear_kp").value))
        if not all(math.isfinite(v) for v in parameter_values):
            raise ValueError("Non-finite bridge parameter")
        if min(self.timeout, period, desired) <= 0 or min(self.max_linear, self.max_angular) < 0:
            raise ValueError("Invalid bridge timing/distance/speed parameter")
        self.pid = SimplePID(desired, parameter_values[-2], parameter_values[-1])
        self.gui_managed = bool(self.get_parameter("gui_managed_mode").value)
        self.gui_context_file = str(self.get_parameter("gui_context_file").value)
        self.gui_runtime_id = str(self.get_parameter("gui_runtime_id").value)
        self.status_output = str(self.get_parameter("status_output").value)
        self.resolved_cmd_topic = self.resolve_topic_name(self.get_parameter("cmd_vel_topic").value)
        if self.gui_managed and (not self.gui_context_file or not self.gui_runtime_id or not self.status_output):
            raise ValueError("Managed bridge requires a context, runtime identity and status path")
        # Serialize all new bridges by ROS domain and fully resolved output
        # topic, including command-line bridges from another project folder.
        from .gui_runtime_support import acquire_topic_lock
        self.topic_lock = acquire_topic_lock(self.resolved_cmd_topic, 'bridge')
        self.cmd_publisher = self.create_publisher(
            Twist, self.get_parameter("cmd_vel_topic").value, 10)
        self.flag_publisher = self.create_publisher(Int8, "/radar_follow_flag", 10)
        self.last_key = None
        self.last_new_frame_time = None
        self.last_command = (0.0, 0.0)
        self.shm = None
        self.stop_reason = None
        self.metadata_required_paths = set()
        self.last_control_output = {}
        # Only one bridge may consume this output directory.
        self.target_dir.mkdir(parents=True, exist_ok=True)
        import fcntl
        self.lock_file = (self.target_dir / ".v1_bridge.lock").open("a")
        try:
            fcntl.flock(self.lock_file, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError as exc:
            self.lock_file.close()
            raise RuntimeError("A V1 bridge already owns this track_dir") from exc
        # Odom ingestion is independent of this motion controller. No IPC writer here.
        self.timer = self.create_timer(period, self.timer_callback)
        self.get_logger().info(f"V8 tracking bridge; V4 motion policy unchanged: {self.target_dir}")
        self.stop("waiting_for_record")

    @staticmethod
    def read_last_complete_record(path):
        # Bounded tail; do not consume a partially appended final row.
        with path.open("rb") as stream:
            stream.seek(0, os.SEEK_END)
            size = stream.tell()
            stream.seek(max(0, size - 8192))
            tail = stream.read()
        for row in reversed(tail.split(b"\n")[:-1]):
            text = row.decode("utf-8").strip()
            if not text or text.startswith("target_id") or set(text) == {"-"}:
                continue
            return text
        raise ValueError("No complete tracking record")

    def publish_command(self, linear, angular):
        velocity = Twist()
        velocity.linear.x = float(linear)
        velocity.angular.z = float(angular)
        self.cmd_publisher.publish(velocity)
        self.last_control_output = {
            "linear_mps": float(linear), "angular_radps": float(angular),
            "motion_requested": bool(linear or angular),
            "reason": self.stop_reason or "tracking_command",
            "published_time_sec": time.time(),
            "allow_translation": not self.stop_reason and getattr(self, "current_record", {}).get("allow_translation", False),
            "allow_yaw": not self.stop_reason and getattr(self, "current_record", {}).get("allow_yaw", False),
            "frame": getattr(self, "current_record", {}).get("frame"),
            "last_measurement_age_sec": getattr(self, "last_measurement_age_sec", None),
        }
        status_now = time.monotonic()
        if getattr(self, "status_output", "") and (
                status_now-getattr(self, "last_status_write", 0) >= .2 or
                getattr(self, "last_status_reason", None) != self.last_control_output["reason"]):
            self.last_status_write = status_now
            self.last_status_reason = self.last_control_output["reason"]
            try:
                from .gui_processes import atomic_json
                atomic_json(self.status_output, dict(self.last_control_output,
                    pid=os.getpid(), runtime_id=self.gui_runtime_id,
                    cmd_vel_topic=self.resolved_cmd_topic))
            except OSError as exc:
                self.get_logger().error("Control status write failed: " + str(exc))

    def stop(self, reason):
        changed = reason != self.stop_reason
        self.stop_reason = reason
        self.last_command = (0.0, 0.0)
        self.pid.reset()
        self.publish_command(0.0, 0.0)
        flag = Int8()
        flag.data = 0
        self.flag_publisher.publish(flag)
        if changed:
            self.get_logger().info(f"Stopped: {reason}")

    def timer_callback(self):
        try:
            files = list(self.target_dir.glob("track_realtime_*.txt"))
            if getattr(self, "gui_managed", False):
                try:
                    context = json.loads(Path(self.gui_context_file).read_text(encoding="utf-8"))
                    if context.get("runtime_id") != self.gui_runtime_id or context.get("permit") is not True:
                        self.stop("gui_paused")
                        return
                    lease_age = time.monotonic() - float(context["lease_monotonic"])
                    if not math.isfinite(lease_age) or not -.05 <= lease_age <= 1.5:
                        self.stop("gui_runtime_expired")
                        return
                    expected_session = str(int(context["session_id"]))
                except (OSError, ValueError, KeyError, TypeError):
                    self.stop("gui_context_unavailable")
                    return
                if int(expected_session) <= 0:
                    self.stop("waiting_for_capture")
                    return
                # A newer file from another GUI/session must not override the
                # active GUI's file. Companion metadata is mandatory here.
                matched = []
                for item in files:
                    try:
                        meta = json.loads(item.with_name(item.name + ".meta.json").read_text(encoding="utf-8"))
                        if isinstance(meta, dict) and str(meta.get("session_id")) == expected_session:
                            matched.append(item)
                    except (OSError, ValueError, TypeError):
                        continue
                files = matched
            if not files:
                self.stop("waiting_for_target" if getattr(self, "gui_managed", False) else "no_tracking_file")
                return
            path = max(files, key=lambda item: item.stat().st_mtime_ns)
            metadata_path = path.with_name(path.name + ".meta.json")
            metadata = json.loads(metadata_path.read_text(encoding="utf-8")) if metadata_path.exists() else None
            atomic_snapshot = metadata is not None and metadata.get("control_snapshot_version") == 1
            record = parse_tracking_record(metadata["record_line"] if atomic_snapshot else self.read_last_complete_record(path))
            self.current_record = record
            if metadata is None and not record["has_measurement"] and record["lost_frames"] > 3 and (record["allow_translation"] or record["allow_yaw"]):
                raise ValueError("Prediction without measurement metadata exceeded 3 frames")
            wall_now = time.time()
            monotonic_now = time.monotonic()
            timestamp = datetime.fromisoformat(record["timestamp"]).timestamp()
            age = wall_now - timestamp
            if age < -0.05 or age > self.timeout or wall_now - (metadata_path if atomic_snapshot else path).stat().st_mtime > self.timeout:
                self.stop("stale_tracking_record")
                return
            required = getattr(self, "metadata_required_paths", set())
            metadata_path = path.with_name(path.name + ".meta.json")
            if metadata_path.exists() or str(path) in required:
                required.add(str(path))
                self.metadata_required_paths = required
                if metadata is None:
                    raise ValueError("Required control metadata missing")
                if getattr(self, "gui_managed", False) and str(metadata.get("session_id")) != expected_session:
                    raise ValueError("GUI capture session changed during record read")
                self.last_measurement_age_sec = validate_frame_metadata(metadata, record, wall_now, self.timeout)
                context_key = (str(path), metadata["session_id"], metadata["selection_epoch"])
                previous = getattr(self, "last_metadata_time", None)
                if previous and previous[0] == context_key and metadata["received_time_sec"] < previous[1] - 0.05:
                    raise ValueError("Sensor receive clock moved backwards")
                self.last_metadata_time = (context_key, metadata["received_time_sec"])
                if getattr(self, "control_identity", context_key) != context_key:
                    self.pid.reset()
                self.control_identity = context_key
            if record["mode"] != "realtime":
                self.stop("offline_record")
                return
            if record["control_state"] == "STOPPED":
                self.stop("tracker_stopped")
                return
            key = (str(path), record["target_id"], record["frame"])
            if key == self.last_key:
                if self.last_new_frame_time is None or monotonic_now - self.last_new_frame_time > self.timeout:
                    self.stop("repeated_frame_timeout")
                else:
                    # Re-send the command at control rate without advancing PID
                    # or treating the same frame as another sensor observation.
                    self.publish_command(*self.last_command)
                return
            if self.last_key is not None:
                if key[:2] == self.last_key[:2] and key[2] < self.last_key[2]:
                    self.stop("out_of_order_frame")
                    return
                if key[:2] != self.last_key[:2]:
                    self.pid.reset()
            self.last_key = key
            self.last_new_frame_time = monotonic_now
            distance = math.hypot(record["x"], record["y"])
            angular, linear = self.pid.update(
                math.atan2(record["x"], record["y"]), distance, monotonic_now)
            scale = record["speed_scale"]
            linear = max(-self.max_linear, min(self.max_linear, linear)) * scale
            angular = max(-self.max_angular, min(self.max_angular, angular)) * scale
            if not record["allow_translation"] or distance <= 0.15:
                linear = 0.0
            if not record["allow_yaw"] or distance <= 0.15:
                angular = 0.0
            self.last_command = (linear, angular)
            self.stop_reason = None
            self.publish_command(linear, angular)
            flag = Int8()
            flag.data = 1
            self.flag_publisher.publish(flag)
        except Exception as exc:
            self.stop(f"invalid_or_unreadable_record: {exc}")

    def close(self):
        if rclpy.ok():
            self.stop("bridge_shutdown")
        if self.shm is not None:
            self.shm.detach()
        self.lock_file.close()
        if getattr(self, "topic_lock", None) is not None:
            self.topic_lock.close()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TxtRadarBridge()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
