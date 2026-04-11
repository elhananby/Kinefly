import numpy as np

from kinefly.core.types import BodyPartState, FlyState
from kinefly.plugins.base import OutputPlugin
from kinefly.plugins.phidgets import PhidgetsPlugin


def test_phidgets_is_output_plugin():
    assert issubclass(PhidgetsPlugin, OutputPlugin)


def test_voltages_from_flystate():
    plugin = PhidgetsPlugin()
    # Default channels: L, R, L-R, L+R
    state = FlyState(
        timestamp=0.0,
        seq=0,
        left=BodyPartState(angles=[1.0]),
        right=BodyPartState(angles=[0.5]),
    )
    voltages = plugin.voltages_from_flystate(state)
    assert len(voltages) == 4
    np.testing.assert_allclose(voltages[0], 1.0, atol=1e-6)  # L
    np.testing.assert_allclose(voltages[1], 0.5, atol=1e-6)  # R
    np.testing.assert_allclose(voltages[2], 0.5, atol=1e-6)  # L-R
    np.testing.assert_allclose(voltages[3], 1.5, atol=1e-6)  # L+R


def test_voltages_clipped_to_range():
    plugin = PhidgetsPlugin()
    state = FlyState(
        timestamp=0.0,
        seq=0,
        left=BodyPartState(angles=[100.0]),  # Will exceed 10V
        right=BodyPartState(angles=[100.0]),
    )
    voltages = plugin.voltages_from_flystate(state)
    assert all(-10.0 <= v <= 10.0 for v in voltages)
