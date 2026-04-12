import subprocess
from pathlib import Path

import numpy as np
import pytest

from kinefly.recording.recorder import VideoRecorder, detect_encoder


def test_detect_encoder_returns_string_or_none():
    """detect_encoder() should return a valid encoder string or None."""
    result = detect_encoder()
    assert result is None or result in ("h264_videotoolbox", "h264_nvenc", "h264_qsv", "libx264")


def test_recorder_write_and_stop(tmp_path):
    """Write 10 frames and verify output file is created."""
    pytest.importorskip("subprocess")

    # Skip if ffmpeg not available
    try:
        subprocess.run(["ffmpeg", "-version"], capture_output=True, check=True)
    except (FileNotFoundError, subprocess.CalledProcessError):
        pytest.skip("ffmpeg not available")

    encoder = detect_encoder()
    if encoder is None:
        pytest.skip("no compatible encoder found")

    output_path = str(tmp_path / "test_out.mp4")
    recorder = VideoRecorder()
    recorder.start(width=64, height=64, fps=10.0, output_path=output_path)

    for _ in range(10):
        frame = np.random.randint(0, 255, (64, 64), dtype=np.uint8)
        recorder.write_frame(frame)

    recorder.stop()
    assert Path(output_path).exists()
    assert Path(output_path).stat().st_size > 0


def test_recorder_output_path_property(tmp_path):
    """output_path property reflects the actual path used."""
    try:
        subprocess.run(["ffmpeg", "-version"], capture_output=True, check=True)
    except (FileNotFoundError, subprocess.CalledProcessError):
        pytest.skip("ffmpeg not available")

    encoder = detect_encoder()
    if encoder is None:
        pytest.skip("no compatible encoder found")

    output_path = str(tmp_path / "test2.mp4")
    recorder = VideoRecorder()
    recorder.start(width=32, height=32, fps=5.0, output_path=output_path)
    assert recorder.output_path == output_path
    recorder.stop()
