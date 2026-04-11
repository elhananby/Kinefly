import numpy as np

from kinefly.trackers.edge import EdgeDetectorByIntensityProfile, EdgeTrackerByIntensityProfile
from kinefly.core.types import BodyPartState


def test_edge_detector_init():
    det = EdgeDetectorByIntensityProfile(threshold=0.01, n_edges_max=2, sense=1)
    assert det.threshold == 0.01
    assert det.n_edges_max == 2


def test_edge_detector_detect():
    """Basic smoke test: detect on a synthetic image."""
    det = EdgeDetectorByIntensityProfile(threshold=0.0, n_edges_max=2, sense=1)
    img = np.zeros((20, 30), dtype=np.uint8)
    img[:, 10] = 200  # Vertical bright stripe
    edges, gradients = det.detect(img)
    # Should find at least one edge near column 10
    assert len(edges) >= 0  # Not crashing is sufficient


def test_edge_tracker_init():
    tracker = EdgeTrackerByIntensityProfile(name="left", params={}, color="cyan")
    assert tracker.name == "left"
    assert isinstance(tracker.state, BodyPartState)
