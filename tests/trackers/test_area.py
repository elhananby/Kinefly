import numpy as np
import pytest

from kinefly.core.types import BodyPartState
from kinefly.trackers.area import AreaTracker


def test_area_tracker_init():
    tracker = AreaTracker(name="head", params={}, color="white")
    assert tracker.name == "head"
    assert isinstance(tracker.state, BodyPartState)
    assert tracker.state.angles == [0.0]
    assert tracker.state.radii == [0.0]
