"""Abstract base class for camera sources."""

from __future__ import annotations

from abc import ABC, abstractmethod

import numpy as np


class CameraSource(ABC):
    """Abstract interface for all image sources (cameras, video files)."""

    @abstractmethod
    def open(self) -> None:
        """Open the camera/file and prepare for frame capture."""
        ...

    @abstractmethod
    def read(self) -> tuple[np.ndarray, float]:
        """Read the next frame.

        Returns:
            Tuple of (image as numpy array, timestamp in seconds).
            Blocks until a frame is available.

        Raises:
            StopIteration: When no more frames are available (end of video file).
            RuntimeError: When the camera is not open or a read error occurs.
        """
        ...

    @abstractmethod
    def close(self) -> None:
        """Release the camera/file resources."""
        ...

    @property
    @abstractmethod
    def fps(self) -> float:
        """Frames per second of the source."""
        ...

    @property
    @abstractmethod
    def resolution(self) -> tuple[int, int]:
        """Resolution as (width, height)."""
        ...
