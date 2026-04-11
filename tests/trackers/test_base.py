import numpy as np
import pytest

from kinefly.gui.handles import Handle
from kinefly.gui.ui_colors import bgra_dict
from kinefly.trackers.base import IntensityTrackedBodypart, MotionTrackedBodypart


def test_handle_hit_inside():
    h = Handle(pt=np.array([10.0, 10.0]), color=bgra_dict["white"], name="test")
    assert h.hit_test(np.array([10.0, 10.0])) is True


def test_handle_hit_outside():
    h = Handle(pt=np.array([10.0, 10.0]), color=bgra_dict["white"], name="test")
    assert h.hit_test(np.array([100.0, 100.0])) is False


def test_motion_tracked_bodypart_init():
    part = MotionTrackedBodypart(name="head", color="white")
    assert part.name == "head"
    assert "hinge" in part.handles
    assert "angle_hi" in part.handles
    assert "angle_lo" in part.handles
    assert "radius_inner" in part.handles


def test_intensity_tracked_bodypart_init():
    part = IntensityTrackedBodypart(name="aux", color="white")
    assert part.name == "aux"
    assert "center" in part.handles
    assert "radius1" in part.handles
    assert "radius2" in part.handles
