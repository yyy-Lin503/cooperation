"""Bridge transport tests use a fake camera; no OpenCV or hardware required."""
import importlib.util
import socket
import struct
import threading
import time
import unittest
from pathlib import Path
from unittest.mock import MagicMock, patch

spec = importlib.util.spec_from_file_location(
    "windows_camera_bridge", Path(__file__).resolve().parents[2] / "tools/diagnostics/windows_camera_bridge.py")
bridge = importlib.util.module_from_spec(spec)
spec.loader.exec_module(bridge)


def receive(sock):
    def exact(size):
        result = b""
        while len(result) < size:
            chunk = sock.recv(size - len(result))
            if not chunk:
                raise EOFError("Partial bridge response")
            result += chunk
        return result
    size = struct.unpack("!I", exact(4))[0]
    return exact(size)


class FakeCamera:
    sequence = 1
    payload = b"latest-jpeg"

    def check(self):
        pass

    def jpeg(self, previous):
        return (previous, b"") if previous == self.sequence else (self.sequence, self.payload)


class BridgeTest(unittest.TestCase):
    def test_pull_latest_duplicate_reconnect_and_idle_shutdown(self):
        camera = FakeCamera()
        stopped = threading.Event()
        listener = socket.socket()
        self.addCleanup(listener.close)
        listener.bind(("127.0.0.1", 0))
        listener.listen(1)
        errors = []

        def run():
            try:
                bridge.serve(listener, camera, stopped.is_set)
            except Exception as exc:
                errors.append(exc)

        thread = threading.Thread(target=run, daemon=True)
        thread.start()
        try:
            with socket.create_connection(listener.getsockname(), timeout=1) as client:
                client.settimeout(0.1)
                with self.assertRaises(socket.timeout):
                    client.recv(1)  # No unsolicited video/queue.
                client.settimeout(1)
                client.sendall(b"F")
                self.assertEqual(receive(client), b"latest-jpeg")
                client.sendall(b"F")
                self.assertEqual(receive(client), b"")
                camera.sequence = 100  # Skip intermediate captures.
                camera.payload = b"newest-only"
                client.sendall(b"F")
                self.assertEqual(receive(client), b"newest-only")
            with socket.create_connection(listener.getsockname(), timeout=1) as client:
                client.sendall(b"F")
                self.assertEqual(receive(client), b"newest-only")
                stopped.set()  # Also stops while the connected client is idle.
                thread.join(1)
                self.assertFalse(thread.is_alive())
        finally:
            stopped.set()
            thread.join(1)
        self.assertFalse(errors)

    def test_msmf_falls_back_and_failed_capture_is_released(self):
        cv = MagicMock()
        first, second = MagicMock(), MagicMock()
        first.isOpened.return_value = False
        second.isOpened.return_value = True
        second.read.return_value = (False, None)
        cv.VideoCapture.side_effect = [first, second]
        with patch.dict("sys.modules", {"cv2": cv}), patch.object(bridge.threading, "Thread"):
            camera = bridge.Camera(0, 640, 480)
            self.assertEqual(cv.VideoCapture.call_args_list[0].args[1], cv.CAP_MSMF)
            self.assertEqual(cv.VideoCapture.call_args_list[1].args[1], cv.CAP_DSHOW)
            first.release.assert_called_once()
            camera._capture()
            second.release.assert_called_once()
            with self.assertRaisesRegex(RuntimeError, "stopped delivering"):
                camera.check()
            camera.close()

    def test_stale_frames_are_not_published_as_fresh(self):
        camera = bridge.Camera.__new__(bridge.Camera)
        camera.lock = threading.Condition()
        camera.frame = object()
        camera.sequence = 2
        camera.captured = time.monotonic() - 1
        camera.error = None
        self.assertEqual(camera.jpeg(1), (1, b""))
        camera.captured = time.monotonic() - 4
        with self.assertRaisesRegex(RuntimeError, "3 seconds"):
            camera.check()


if __name__ == "__main__":
    unittest.main()
