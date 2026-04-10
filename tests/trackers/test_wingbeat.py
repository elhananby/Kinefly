import numpy as np

from kinefly.trackers.wingbeat import WingbeatDetector


def test_fs_dict_from_wingband():
    """Verify the undersampling framerate computation produces valid ranges."""
    wd = WingbeatDetector(fw_min=180.0, fw_max=220.0)
    fs_dict = wd.fs_dict_from_wingband(180.0, 220.0)
    assert len(fs_dict["fs_range_list"]) > 0
    # Each range should have lo < hi
    for lo, hi in fs_dict["fs_range_list"]:
        assert lo < hi


def test_freq_from_intensity_returns_float():
    """Verify freq_from_intensity runs and returns a numeric value."""
    wd = WingbeatDetector(fw_min=180.0, fw_max=220.0)
    freq = wd.freq_from_intensity(0.5, fs=100)
    assert isinstance(freq, float)


def test_buffer_wraps():
    """Feed more samples than the buffer size to verify wraparound."""
    wd = WingbeatDetector(fw_min=180.0, fw_max=220.0)
    for i in range(200):
        freq = wd.freq_from_intensity(np.sin(i * 0.1), fs=100)
    assert isinstance(freq, float)
