"""Video recording via FFmpeg subprocess pipe.

Frames are written as raw grayscale bytes to FFmpeg's stdin, with hardware
encoder auto-detection (h264_nvenc -> h264_qsv -> libx264).
"""

from __future__ import annotations

import logging
import subprocess
from datetime import datetime
from pathlib import Path

import numpy as np

logger = logging.getLogger(__name__)

_ENCODER_PREFERENCE = ["h264_nvenc", "h264_qsv", "libx264"]


def detect_encoder() -> str | None:
    """Return the best available H.264 encoder, or None if ffmpeg is unavailable."""
    try:
        result = subprocess.run(
            ["ffmpeg", "-encoders"],
            capture_output=True,
            text=True,
            timeout=5,
        )
        output = result.stdout + result.stderr
        for encoder in _ENCODER_PREFERENCE:
            if encoder in output:
                return encoder
        return None
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return None


class VideoRecorder:
    """Record grayscale video frames to MP4 via FFmpeg."""

    def __init__(self) -> None:
        self._process: subprocess.Popen | None = None
        self._output_path: str | None = None

    def start(
        self,
        width: int,
        height: int,
        fps: float = 30.0,
        output_path: str | None = None,
    ) -> None:
        """Open an FFmpeg pipe and begin recording.

        Args:
            width: Frame width in pixels.
            height: Frame height in pixels.
            fps: Frames per second.
            output_path: Destination .mp4 path. Auto-generated if None.

        Raises:
            RuntimeError: If FFmpeg is not available or no encoder is found.
        """
        encoder = detect_encoder()
        if encoder is None:
            raise RuntimeError(
                "No compatible H.264 encoder found. "
                "Install ffmpeg with libx264 support."
            )

        if output_path is None:
            timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
            output_dir = Path("~/kinefly_recordings").expanduser()
            output_dir.mkdir(parents=True, exist_ok=True)
            output_path = str(output_dir / f"{timestamp}.mp4")

        self._output_path = output_path

        cmd = [
            "ffmpeg",
            "-y",
            "-f", "rawvideo",
            "-pix_fmt", "gray",
            "-s", f"{width}x{height}",
            "-r", str(fps),
            "-i", "pipe:0",
            "-c:v", encoder,
            output_path,
        ]

        self._process = subprocess.Popen(
            cmd,
            stdin=subprocess.PIPE,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        logger.info("Recording started: %s (encoder=%s)", output_path, encoder)

    def write_frame(self, frame: np.ndarray) -> None:
        """Write a single grayscale frame to the recording.

        Args:
            frame: Grayscale image as uint8 numpy array.
        """
        if self._process is None or self._process.stdin is None:
            return
        self._process.stdin.write(frame.astype(np.uint8).tobytes())

    def stop(self) -> None:
        """Finalize the recording and close the FFmpeg process."""
        if self._process is not None:
            if self._process.stdin is not None:
                self._process.stdin.close()
            self._process.wait()
            self._process = None
            logger.info("Recording stopped: %s", self._output_path)

    @property
    def output_path(self) -> str | None:
        """The output file path used for this recording."""
        return self._output_path
