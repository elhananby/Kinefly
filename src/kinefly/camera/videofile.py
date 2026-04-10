"""Video file replay source."""

from __future__ import annotations

import time

import cv2
import numpy as np

from kinefly.camera.base import CameraSource


class VideoFileSource(CameraSource):
    """Replay a recorded video file.

    Args:
        path: Path to the video file (MP4, AVI, etc.).
        realtime: If True, sleep between frames to match original framerate.
            If False, deliver frames as fast as possible (batch retracking).
    """

    def __init__(self, path: str, realtime: bool = True) -> None:
        self._path = path
        self._realtime = realtime
        self._cap: cv2.VideoCapture | None = None
        self._fps: float = 30.0
        self._resolution: tuple[int, int] = (640, 480)
        self._last_frame_time: float = 0.0

    def open(self) -> None:
        self._cap = cv2.VideoCapture(self._path)
        if not self._cap.isOpened():
            raise RuntimeError(f"Cannot open video file: {self._path}")
        self._fps = self._cap.get(cv2.CAP_PROP_FPS) or 30.0
        w = int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self._resolution = (w, h)
        self._last_frame_time = time.time()

    def read(self) -> tuple[np.ndarray, float]:
        if self._cap is None:
            raise RuntimeError("Video file not opened. Call open() first.")

        if self._realtime:
            elapsed = time.time() - self._last_frame_time
            target = 1.0 / self._fps
            if elapsed < target:
                time.sleep(target - elapsed)

        ret, frame = self._cap.read()
        if not ret:
            raise StopIteration("End of video file")

        self._last_frame_time = time.time()

        if len(frame.shape) == 3:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        timestamp = self._cap.get(cv2.CAP_PROP_POS_MSEC) / 1000.0
        return frame, timestamp

    def close(self) -> None:
        if self._cap is not None:
            self._cap.release()
            self._cap = None

    @property
    def fps(self) -> float:
        return self._fps

    @property
    def resolution(self) -> tuple[int, int]:
        return self._resolution
