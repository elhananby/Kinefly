import numpy as np

from kinefly.core.types import BodyPartState
from kinefly.trackers.tip import TipDetector, TipTracker


def test_tip_detector_init():
    det = TipDetector(threshold=0.0, sense=1)
    assert det.threshold == 0.0


def test_tip_detector_detect_empty():
    det = TipDetector(threshold=0.0, sense=1)
    img = np.zeros((20, 30), dtype=np.uint8)
    (xTip, yTip) = det.detect(img)
    # Black image should yield no clear tip
    assert xTip is None or isinstance(xTip, (int, np.integer))


def test_tip_tracker_init():
    tracker = TipTracker(name="left", params={}, color="cyan")
    assert tracker.name == "left"
    assert isinstance(tracker.state, BodyPartState)
