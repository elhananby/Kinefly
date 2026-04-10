"""OpenCV VideoCapture camera source — development/testing fallback."""

from __future__ import annotations

import time

import cv2
import numpy as np

from kinefly.camera.base import CameraSource


class OpenCVCamera(CameraSource):
    """Camera source using OpenCV's VideoCapture for USB/built-in cameras.

    Args:
        device_index: Camera device index (default 0).
        framerate: Requested framerate (best-effort, depends on hardware).
    """

    def __init__(self, device_index: int = 0, framerate: float = 60.0) -> None:
        self._device_index = device_index
        self._requested_fps = framerate
        self._cap: cv2.VideoCapture | None = None
        self._fps: float = framerate
        self._resolution: tuple[int, int] = (640, 480)

    def open(self) -> None:
        self._cap = cv2.VideoCapture(self._device_index)
        if not self._cap.isOpened():
            raise RuntimeError(f"Cannot open camera at index {self._device_index}")
        self._cap.set(cv2.CAP_PROP_FPS, self._requested_fps)
        self._fps = self._cap.get(cv2.CAP_PROP_FPS) or self._requested_fps
        w = int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self._resolution = (w, h)

    def read(self) -> tuple[np.ndarray, float]:
        if self._cap is None:
            raise RuntimeError("Camera not opened. Call open() first.")
        ret, frame = self._cap.read()
        if not ret:
            raise RuntimeError("Failed to read frame from camera")
        # Convert to grayscale if color
        if len(frame.shape) == 3:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        return frame, time.time()

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
