"""Windows camera -> pull-based JPEG TCP bridge for the existing WSL worker.

Protocol: client sends b'F'; server replies with a network-order uint32 length
and JPEG bytes. Length zero means no newer/fresh frame. No capture clock crosses
the Windows/WSL boundary. Run --help for standalone usage.
"""

import argparse
import json
import socket
import struct
import sys
import threading
import time
from pathlib import Path

MAX_JPEG = 8 * 1024 * 1024


class Camera:
    def __init__(self, index, width, height):
        import cv2

        self.cv2 = cv2
        self.capture = None
        for backend in (cv2.CAP_MSMF, cv2.CAP_DSHOW):
            capture = cv2.VideoCapture(index, backend)
            if capture.isOpened():
                self.capture = capture
                self.backend = capture.getBackendName()
                break
            capture.release()
        if self.capture is None:
            raise RuntimeError("Cannot open Windows camera. Enable it and check camera permissions/device index.")
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.lock = threading.Condition()
        self.stopping = threading.Event()
        self.frame = None
        self.sequence = 0
        self.captured = 0.0
        self.error = None
        self.thread = threading.Thread(target=self._capture, daemon=True)
        self.thread.start()

    def _capture(self):
        try:
            while not self.stopping.is_set():
                ok, frame = self.capture.read()
                if not ok or frame is None:
                    raise RuntimeError("Windows camera stopped delivering images.")
                with self.lock:
                    self.frame = frame
                    self.sequence += 1
                    self.captured = time.monotonic()
                    self.lock.notify_all()
        except Exception as exc:
            with self.lock:
                self.error = str(exc)
                self.lock.notify_all()
        finally:
            self.capture.release()

    def check(self):
        with self.lock:
            if self.error:
                raise RuntimeError(self.error)
            if self.captured and time.monotonic() - self.captured > 3:
                raise RuntimeError("Windows camera has not delivered an image for 3 seconds.")

    def wait_ready(self):
        deadline = time.monotonic() + 3
        with self.lock:
            while self.frame is None and not self.error:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise RuntimeError("Windows camera startup timed out.")
                self.lock.wait(min(remaining, 0.1))
            if self.error:
                raise RuntimeError(self.error)
            return self.frame.shape[1], self.frame.shape[0]

    def jpeg(self, previous):
        self.check()
        with self.lock:
            # Wait briefly for a new capture instead of spinning TCP requests.
            self.lock.wait_for(lambda: self.sequence != previous or self.error, timeout=0.1)
            if self.error:
                raise RuntimeError(self.error)
            frame, sequence, captured = self.frame, self.sequence, self.captured
        if frame is None or sequence == previous or time.monotonic() - captured > 0.5:
            return previous, b""
        ok, encoded = self.cv2.imencode(".jpg", frame, [self.cv2.IMWRITE_JPEG_QUALITY, 85])
        if not ok or encoded.nbytes > MAX_JPEG:
            raise RuntimeError("Camera JPEG encoding failed or frame exceeded 8 MiB.")
        if time.monotonic() - captured > 0.5:
            return previous, b""
        return sequence, encoded.tobytes()

    def close(self):
        self.stopping.set()
        self.thread.join(timeout=2)


def serve(listener, camera, stopped):
    listener.settimeout(0.1)
    while not stopped():
        camera.check()
        try:
            client, _ = listener.accept()
        except socket.timeout:
            continue
        with client:
            client.settimeout(0.1)
            previous = 0
            while not stopped():
                camera.check()
                try:
                    request = client.recv(1)
                except socket.timeout:
                    continue
                except ConnectionError:
                    break
                if request != b"F":
                    break
                previous, jpeg = camera.jpeg(previous)
                try:
                    client.sendall(struct.pack("!I", len(jpeg)) + jpeg)
                except (ConnectionError, socket.timeout):
                    break


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--camera-index", type=int, default=0)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=0)
    parser.add_argument("--ready-file", type=Path)
    parser.add_argument("--stop-file", type=Path)
    args = parser.parse_args()
    if args.camera_index < 0 or args.width <= 0 or args.height <= 0 or not 0 <= args.port <= 65535:
        parser.error("Invalid camera index, dimensions or port.")
    camera = None
    try:
        camera = Camera(args.camera_index, args.width, args.height)
        width, height = camera.wait_ready()
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as listener:
            listener.bind((args.host, args.port))
            listener.listen(1)
            ready = {"host": args.host, "port": listener.getsockname()[1],
                     "width": width, "height": height, "backend": camera.backend}
            if args.ready_file:
                temporary = args.ready_file.with_suffix(".tmp")
                temporary.write_text(json.dumps(ready), encoding="utf-8")
                temporary.replace(args.ready_file)
            print(json.dumps(ready), flush=True)
            serve(listener, camera, lambda: bool(args.stop_file and args.stop_file.exists()))
        return 0
    except KeyboardInterrupt:
        return 0
    except Exception as exc:
        print(f"Camera bridge failed: {exc}", file=sys.stderr, flush=True)
        return 1
    finally:
        if camera is not None:
            camera.close()


if __name__ == "__main__":
    sys.exit(main())
