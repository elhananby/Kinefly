"""Tests for LED panels output plugin — no hardware required."""

from __future__ import annotations

import time
from unittest.mock import MagicMock, patch

from kinefly.core.types import BodyPartState, FlyState
from kinefly.plugins.base import OutputPlugin
from kinefly.plugins.ledpanels import LedPanelsPlugin

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _make_state(
    left_major=0.0,
    right_major=0.0,
    head_angle=0.0,
    head_radius=0.0,
    abdomen_angle=0.0,
    abdomen_radius=0.0,
    aux_intensity=0.0,
):
    return FlyState(
        timestamp=0.0,
        seq=0,
        left=BodyPartState(angles=[left_major]),
        right=BodyPartState(angles=[right_major]),
        head=BodyPartState(angles=[head_angle], radii=[head_radius]),
        abdomen=BodyPartState(angles=[abdomen_angle], radii=[abdomen_radius]),
        aux=BodyPartState(intensity=aux_intensity),
    )


# ---------------------------------------------------------------------------
# 1. Subclass check
# ---------------------------------------------------------------------------


def test_ledpanels_is_output_plugin():
    assert issubclass(LedPanelsPlugin, OutputPlugin)


# ---------------------------------------------------------------------------
# 2–5. bytes_from_command
# ---------------------------------------------------------------------------


def test_bytes_from_command_no_args():
    """'start' command: id=0x20, no args → [0x01, 0x20]."""
    plugin = LedPanelsPlugin()
    result = plugin.bytes_from_command("start", [])
    assert result == bytes([0x01, 0x20])


def test_bytes_from_command_set_pattern_id():
    """'set_pattern_id' with arg=3: 1-byte arg, total=2 → [0x02, 0x03, 3]."""
    plugin = LedPanelsPlugin()
    result = plugin.bytes_from_command("set_pattern_id", [3])
    assert result == bytes([0x02, 0x03, 3])


def test_bytes_from_command_set_position():
    """'set_position' args=[10, 5]: each 2 bytes little-endian, total=5.

    Wire: [0x05, 0x70, 10, 0, 5, 0]
    """
    plugin = LedPanelsPlugin()
    result = plugin.bytes_from_command("set_position", [10, 5])
    assert result == bytes([0x05, 0x70, 10, 0, 5, 0])


def test_bytes_from_command_send_gain_bias():
    """'send_gain_bias' args=[10, 0, -5, 0]: each 1 signed byte, total=5.

    Wire: [0x05, 0x71, 10, 0, 251, 0]  (-5 in two's complement = 251)
    """
    plugin = LedPanelsPlugin()
    result = plugin.bytes_from_command("send_gain_bias", [10, 0, -5, 0])
    assert result == bytes([0x05, 0x71, 10, 0, 251, 0])


# ---------------------------------------------------------------------------
# 6. compute_position
# ---------------------------------------------------------------------------


def test_compute_position_default_coeffs():
    """Default coeffs: x = left_major - right_major, y = 0.

    left=0.3, right=0.1 → pos_x=0.2 → index_x=int(0.2)=0, index_y=0.
    """
    plugin = LedPanelsPlugin()
    state = _make_state(left_major=0.3, right_major=0.1)
    index_x, index_y = plugin.compute_position(state)
    assert index_x == 0
    assert index_y == 0


def test_compute_position_large_values():
    """Larger values produce non-zero index_x."""
    plugin = LedPanelsPlugin()
    state = _make_state(left_major=10.0, right_major=0.0)
    index_x, index_y = plugin.compute_position(state)
    assert index_x == 10
    assert index_y == 0


# ---------------------------------------------------------------------------
# 7. compute_velocity
# ---------------------------------------------------------------------------


def test_compute_velocity_default_coeffs():
    """Default coeffs, left=0.3, right=0.1 → vel[0]=0.2 → gain_x=0, bias_x=0, gain_y=0, bias_y=0."""
    plugin = LedPanelsPlugin()
    state = _make_state(left_major=0.3, right_major=0.1)
    gain_x, bias_x, gain_y, bias_y = plugin.compute_velocity(state)
    # (int(0.2) + 128) % 256 - 128 = 128 % 256 - 128 = 0
    assert gain_x == 0
    assert bias_x == 0
    assert gain_y == 0
    assert bias_y == 0


def test_compute_velocity_nonzero():
    """Larger L-R difference produces non-zero gain_x."""
    plugin = LedPanelsPlugin()
    # left=5.0, right=0.0 → vel[0]=5.0 → gain_x=(int(5)+128)%256-128 = 133%256-128=5
    state = _make_state(left_major=5.0, right_major=0.0)
    gain_x, bias_x, gain_y, bias_y = plugin.compute_velocity(state)
    assert gain_x == 5
    assert bias_x == 0
    assert gain_y == 0
    assert bias_y == 0


# ---------------------------------------------------------------------------
# 8. on_flystate puts bytes in queue
# ---------------------------------------------------------------------------


def test_on_flystate_puts_bytes_in_queue():
    """After start with mock serial, on_flystate should enqueue bytes."""
    plugin = LedPanelsPlugin()
    mock_serial = MagicMock()

    with patch("serial.Serial", return_value=mock_serial):
        plugin.start({"port": "/dev/ttyUSB0", "mode": "velocity"})
        state = _make_state(left_major=1.0, right_major=0.0)
        plugin.on_flystate(state)
        # Give thread a moment to drain
        time.sleep(0.05)
        plugin.stop()

    mock_serial.write.assert_called()


# ---------------------------------------------------------------------------
# 9. stop closes serial
# ---------------------------------------------------------------------------


def test_stop_closes_serial():
    """stop() should close the serial port."""
    plugin = LedPanelsPlugin()
    mock_serial = MagicMock()

    with patch("serial.Serial", return_value=mock_serial):
        plugin.start({"port": "/dev/ttyUSB0"})
        plugin.stop()

    mock_serial.close.assert_called_once()


# ---------------------------------------------------------------------------
# 10. voltage method sends init command
# ---------------------------------------------------------------------------


def test_voltage_method_sends_init_command():
    """start() with method='voltage' should write set_mode_vel_custom_x bytes to serial."""
    plugin = LedPanelsPlugin()
    mock_serial = MagicMock()

    with patch("serial.Serial", return_value=mock_serial):
        plugin.start(
            {
                "port": "/dev/ttyUSB0",
                "method": "voltage",
                "mode": "velocity",
                "axis": "x",
            }
        )
        plugin.stop()

    # Build expected bytes for set_mode_vel_custom_x with default coefficients
    # adc0=1, adc1=0, adc2=0, adc3=0, funcx=0, funcy=0
    expected_bytes = plugin.bytes_from_command("set_mode_vel_custom_x", [1, 0, 0, 0, 0, 0])

    # Collect all write call args
    written = [call.args[0] for call in mock_serial.write.call_args_list]
    assert expected_bytes in written, (
        f"Expected {expected_bytes!r} to be written to serial; got {written!r}"
    )


def test_voltage_method_custom_coefficients():
    """start() with method='voltage' and custom coeff_voltage uses those values."""
    plugin = LedPanelsPlugin()
    mock_serial = MagicMock()

    with patch("serial.Serial", return_value=mock_serial):
        plugin.start(
            {
                "port": "/dev/ttyUSB0",
                "method": "voltage",
                "mode": "position",
                "axis": "y",
                "coeff_voltage": {
                    "adc0": 2,
                    "adc1": 3,
                    "adc2": 0,
                    "adc3": 0,
                    "funcx": 1,
                    "funcy": 0,
                },
            }
        )
        plugin.stop()

    expected_bytes = plugin.bytes_from_command("set_mode_pos_custom_y", [2, 3, 0, 0, 1, 0])
    written = [call.args[0] for call in mock_serial.write.call_args_list]
    assert expected_bytes in written, (
        f"Expected {expected_bytes!r} to be written to serial; got {written!r}"
    )
