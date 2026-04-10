import numpy as np
import pytest

from kinefly.camera.base import CameraSource
from kinefly.camera.opencv import OpenCVCamera


def test_opencv_camera_is_camera_source():
    assert issubclass(OpenCVCamera, CameraSource)


def test_opencv_camera_has_required_methods():
    cam = OpenCVCamera(device_index=0)
    assert hasattr(cam, "open")
    assert hasattr(cam, "read")
    assert hasattr(cam, "close")
    assert hasattr(cam, "fps")
    assert hasattr(cam, "resolution")
