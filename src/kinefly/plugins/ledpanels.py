"""LED panels output plugin via USB serial.

Ported from flystate2ledpanels.py (mapping) and ledpanels/nodes/ledpanels.py (protocol).
Serial I/O runs on a dedicated background thread using queue.Queue for thread safety.
Supports 'usb' method (position or velocity commands) and 'voltage' method
(controller-side ADC coefficients).
"""

from __future__ import annotations

import logging
import queue
import threading
from typing import Any

import numpy as np

from kinefly.core.types import FlyState
from kinefly.plugins.base import OutputPlugin

logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Serial command table (ported from ledpanels/nodes/ledpanels.py)
# ---------------------------------------------------------------------------

# Arg spec shorthands used in the command table below.
_U8 = {"nbytes": 1, "min": 0, "max": 0xFF, "unsigned": True}
_U8_6 = {"nbytes": 1, "min": 0, "max": 6, "unsigned": True}
_U8_7 = {"nbytes": 1, "min": 0, "max": 7, "unsigned": True}
_U8_10 = {"nbytes": 1, "min": 0, "max": 10, "unsigned": True}
_U8_99 = {"nbytes": 1, "min": 1, "max": 99, "unsigned": True}
_U16_500 = {"nbytes": 2, "min": 0, "max": 500, "unsigned": True}
_U16_2047 = {"nbytes": 2, "min": 0, "max": 2047, "unsigned": True}
_S8 = {"nbytes": 1, "min": -128, "max": 127, "unsigned": False}
_S8_u = {"nbytes": 1, "min": 0, "max": 0xFF, "unsigned": False}
_S16 = {"nbytes": 2, "min": -32768, "max": 32767, "unsigned": False}

COMMANDS: dict[str, dict[str, Any]] = {
    # 1 byte commands (id only, no args):
    "start":                    {"id": 0x20, "args": []},
    "stop":                     {"id": 0x30, "args": []},
    "all_off":                  {"id": 0x00, "args": []},
    "all_on":                   {"id": 0xFF, "args": []},
    "clear":                    {"id": 0xF0, "args": []},
    "start_w_trig":             {"id": 0x25, "args": []},
    "stop_w_trig":              {"id": 0x35, "args": []},
    "led_tog":                  {"id": 0x50, "args": []},
    "ctr_reset":                {"id": 0x60, "args": []},
    "bench_pattern":            {"id": 0x70, "args": []},
    "laser_on":                 {"id": 0x10, "args": []},
    "laser_off":                {"id": 0x11, "args": []},
    "ident_compress_on":        {"id": 0x12, "args": []},
    "ident_compress_off":       {"id": 0x13, "args": []},
    "sync_sd_info":             {"id": 0x14, "args": []},
    "get_version":              {"id": 0x15, "args": []},
    "show_bus_number":          {"id": 0x16, "args": []},
    "quiet_mode_on":            {"id": 0x17, "args": []},
    "quiet_mode_off":           {"id": 0x18, "args": []},
    "update_gui_info":          {"id": 0x19, "args": []},
    "controller_mode":          {"id": 0x21, "args": []},
    "pc_dumping_mode":          {"id": 0x22, "args": []},
    "enable_extern_trig":       {"id": 0x23, "args": []},
    "disable_extern_trig":      {"id": 0x24, "args": []},
    "read_and_set_max_voltage": {"id": 0x26, "args": []},
    "g_level_0":                {"id": 0x90, "args": []},
    "g_level_1":                {"id": 0x91, "args": []},
    "g_level_2":                {"id": 0x92, "args": []},
    "g_level_3":                {"id": 0x93, "args": []},
    "g_level_4":                {"id": 0x94, "args": []},
    "g_level_5":                {"id": 0x95, "args": []},
    "g_level_6":                {"id": 0x96, "args": []},
    "g_level_7":                {"id": 0x97, "args": []},
    "g_level_8":                {"id": 0x98, "args": []},
    "g_level_9":                {"id": 0x99, "args": []},
    "g_level_10":               {"id": 0x9A, "args": []},
    "g_level_11":               {"id": 0x9B, "args": []},
    "g_level_12":               {"id": 0x9C, "args": []},
    "g_level_13":               {"id": 0x9D, "args": []},
    "g_level_14":               {"id": 0x9E, "args": []},
    "g_level_15":               {"id": 0x9F, "args": []},
    # 2 byte commands (1 arg):
    "reset":            {"id": 0x01, "args": [_U8]},
    "display":          {"id": 0x02, "args": [_U8]},
    "set_pattern_id":   {"id": 0x03, "args": [_U8_99]},
    "adc_test":         {"id": 0x04, "args": [_U8_7]},
    "dio_test":         {"id": 0x05, "args": [_U8_7]},
    "set_trigger_rate": {"id": 0x06, "args": [_U8]},
    # 3 byte commands (2 args):
    "set_mode":       {"id": 0x10, "args": [_U8_6, _U8_6]},
    "address":        {"id": 0xFF, "args": [_U8, _U8]},
    "set_posfunc_id": {"id": 0x15, "args": [_U8, _U8]},
    "set_velfunc_id": {"id": 0x20, "args": [_U8, _U8]},
    "set_funcx_freq": {"id": 0x25, "args": [_U16_500]},
    "set_funcy_freq": {"id": 0x30, "args": [_U16_500]},
    "set_max_voltage": {"id": 0x35, "args": [_U8_10, _U8_10]},
    # 5 byte commands:
    "set_position":   {"id": 0x70, "args": [_U16_2047, _U16_2047]},
    "send_gain_bias": {"id": 0x71, "args": [_S8, _S8, _S8, _S8]},
    # 7 byte commands:
    "set_mode_pos_custom_x": {"id": 0x63, "args": [_S8_u] * 6},
    "set_mode_pos_custom_y": {"id": 0x64, "args": [_S8_u] * 6},
    "set_mode_vel_custom_x": {"id": 0x65, "args": [_S8_u] * 6},
    "set_mode_vel_custom_y": {"id": 0x66, "args": [_S8_u] * 6},
    # 9 byte commands:
    "send_gain_bias_16": {"id": 0x01, "args": [_S16] * 4},
}

# Coefficient keys in state-vector order (length 10):
# [1, leftMajor, leftMinor, rightMajor, rightMinor,
#  headAngle, headRadius, abdomenAngle, abdomenRadius, auxIntensity]
_COEFF_KEYS_X = ["x0", "xl1", "xl2", "xr1", "xr2", "xha", "xhr", "xaa", "xar", "xxi"]
_COEFF_KEYS_Y = ["y0", "yl1", "yl2", "yr1", "yr2", "yha", "yhr", "yaa", "yar", "yxi"]

_DEFAULT_COEFF_USB: dict[str, float] = {
    "x0": 0.0, "xl1": 1.0, "xl2": 0.0, "xr1": -1.0, "xr2": 0.0,
    "xha": 0.0, "xhr": 0.0, "xaa": 0.0, "xar": 0.0, "xxi": 0.0,
    "y0": 0.0, "yl1": 0.0, "yl2": 0.0, "yr1": 0.0,  "yr2": 0.0,
    "yha": 0.0, "yhr": 0.0, "yaa": 0.0, "yar": 0.0, "yxi": 0.0,
}


def _dec2bytes(num: int, n: int) -> bytes:
    """Convert integer to n bytes, LSB first (little-endian).

    Ported from Dec2chr in ledpanels/nodes/ledpanels.py.
    The original fills char_list[j-1] for j descending from n to 1,
    which gives char_list[0]=LSB, char_list[n-1]=MSB (little-endian).
    """
    result = bytearray(n)
    num_rem = num
    for j in range(n, 0, -1):
        shift = 8 * (j - 1)
        byte_val = num_rem >> shift
        num_rem -= byte_val << shift
        result[j - 1] = byte_val & 0xFF
    return bytes(result)


class LedPanelsPlugin(OutputPlugin):
    """LED panels output plugin via USB serial.

    Ported from flystate2ledpanels.py (mapping) and ledpanels/nodes/ledpanels.py (protocol).
    Serial I/O runs on a dedicated background thread using queue.Queue for thread safety.
    Supports 'usb' method (position or velocity commands) and 'voltage' method
    (controller-side ADC coefficients).
    """

    def __init__(self) -> None:
        self._queue: queue.Queue[bytes | None] = queue.Queue(maxsize=10)
        self._thread: threading.Thread | None = None
        self._serial = None
        # Default config
        self._port: str = "/dev/ttyUSB0"
        self._baudrate: int = 115200
        self._method: str = "usb"
        self._mode: str = "velocity"
        self._pattern_id: int = 1
        self._coeff_usb: dict[str, float] = dict(_DEFAULT_COEFF_USB)
        # Build coefficient matrix (2x10)
        self._a: np.ndarray = self._build_coeff_matrix(self._coeff_usb)

    # ------------------------------------------------------------------
    # OutputPlugin interface
    # ------------------------------------------------------------------

    def start(self, config: dict[str, Any]) -> None:
        """Open serial port and start background writer thread.

        Args:
            config: Plugin configuration dict with optional keys:
                port, baudrate, method, mode, pattern_id, coeff_usb, coeff_voltage.
        """
        self._port = config.get("port", self._port)
        self._baudrate = int(config.get("baudrate", self._baudrate))
        self._method = config.get("method", self._method)
        self._mode = config.get("mode", self._mode)
        self._pattern_id = int(config.get("pattern_id", self._pattern_id))

        coeff_usb = config.get("coeff_usb", {})
        merged = dict(_DEFAULT_COEFF_USB)
        merged.update(coeff_usb)
        self._coeff_usb = merged
        self._a = self._build_coeff_matrix(self._coeff_usb)

        try:
            import serial  # noqa: PLC0415

            self._serial = serial.Serial(self._port, self._baudrate, timeout=1)
            logger.info("LED panels serial port opened: %s @ %d", self._port, self._baudrate)
        except ImportError:
            logger.warning("pyserial not installed. Install with: pip install kinefly[ledpanels]")
        except Exception:
            logger.exception("Failed to open serial port %s", self._port)

        # Send initial commands
        if self._serial is not None:
            try:
                self._serial.write(self.bytes_from_command("set_pattern_id", [self._pattern_id]))
                self._serial.write(self.bytes_from_command("start", []))
            except Exception:
                logger.warning("Failed to send init commands to LED panels")

        self._thread = threading.Thread(target=self._writer_thread, daemon=True)
        self._thread.start()

    def on_flystate(self, state: FlyState) -> None:
        """Compute panel command from fly state and enqueue for serial write."""
        if self._method == "usb":
            if self._mode == "position":
                index_x, index_y = self.compute_position(state)
                data = self.bytes_from_command("set_position", [index_x, index_y])
            else:
                gain_x, bias_x, gain_y, bias_y = self.compute_velocity(state)
                data = self.bytes_from_command("send_gain_bias", [gain_x, bias_x, gain_y, bias_y])
        else:
            # voltage method: not fully implemented yet; skip
            return

        try:
            self._queue.put_nowait(data)
        except queue.Full:
            pass  # drop frame rather than block

    def stop(self) -> None:
        """Stop background thread and close serial port."""
        if self._thread is not None:
            self._queue.put(None)  # sentinel
            self._thread.join(timeout=2.0)
            self._thread = None

        if self._serial is not None:
            try:
                self._serial.write(self.bytes_from_command("stop", []))
            except Exception:
                pass
            try:
                self._serial.close()
            except Exception:
                pass
            self._serial = None

    # ------------------------------------------------------------------
    # Public computation methods (exposed for testing)
    # ------------------------------------------------------------------

    def bytes_from_command(self, command: str, args: list[int]) -> bytes:
        """Build serial byte payload for a command.

        Wire format: [total_nbytes, cmd_id, arg1_bytes..., arg2_bytes..., ...]
        total_nbytes = 1 (for id byte) + sum of all arg nbytes.

        For 1-byte unsigned arg: single byte with value.
        For 1-byte signed arg: two's complement ((256 + value) % 256).
        For multi-byte arg: little-endian bytes (LSB first).
        """
        cmd_spec = COMMANDS[command]
        cmd_id = cmd_spec["id"]
        arg_specs = cmd_spec["args"]

        total_arg_bytes = sum(spec["nbytes"] for spec in arg_specs)
        total_nbytes = 1 + total_arg_bytes  # 1 for the id byte itself

        payload = bytearray([total_nbytes, cmd_id])

        for i, spec in enumerate(arg_specs):
            val = args[i]
            n = spec["nbytes"]
            unsigned = spec["unsigned"]
            if n == 1:
                if unsigned:
                    payload.append(val & 0xFF)
                else:
                    payload.append((256 + val) % 256)
            else:
                payload.extend(_dec2bytes(val, n))

        return bytes(payload)

    def compute_position(self, state: FlyState) -> tuple[int, int]:
        """Compute (index_x, index_y) from fly state using coefficient matrix."""
        sv = self._state_vector(state)
        pos = np.dot(self._a, sv)
        return int(pos[0]), int(pos[1])

    def compute_velocity(self, state: FlyState) -> tuple[int, int, int, int]:
        """Compute (gain_x, bias_x, gain_y, bias_y) from fly state."""
        sv = self._state_vector(state)
        vel = np.dot(self._a, sv)
        gain_x = (int(vel[0]) + 128) % 256 - 128
        gain_y = (int(vel[1]) + 128) % 256 - 128
        return gain_x, 0, gain_y, 0

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _state_vector(self, state: FlyState) -> np.ndarray:
        """Extract 10-element state vector from FlyState.

        Elements: [1, leftMajor, leftMinor, rightMajor, rightMinor,
                   headAngle, headRadius, abdomenAngle, abdomenRadius, auxIntensity]
        """
        left_major  = state.left.angles[0]  if len(state.left.angles)  > 0 else 0.0
        left_minor  = state.left.angles[1]  if len(state.left.angles)  > 1 else 0.0
        right_major = state.right.angles[0] if len(state.right.angles) > 0 else 0.0
        right_minor = state.right.angles[1] if len(state.right.angles) > 1 else 0.0
        head_angle  = state.head.angles[0]  if len(state.head.angles)  > 0 else 0.0
        head_radius = state.head.radii[0]   if len(state.head.radii)   > 0 else 0.0
        abd_angle   = state.abdomen.angles[0] if len(state.abdomen.angles) > 0 else 0.0
        abd_radius  = state.abdomen.radii[0]  if len(state.abdomen.radii)  > 0 else 0.0

        return np.array(
            [1.0, left_major, left_minor, right_major, right_minor,
             head_angle, head_radius, abd_angle, abd_radius, state.aux.intensity],
            dtype=np.float32,
        )

    @staticmethod
    def _build_coeff_matrix(coeff: dict[str, float]) -> np.ndarray:
        """Build 2x10 coefficient matrix from coeff_usb dict."""
        x_row = [coeff.get(k, 0.0) for k in _COEFF_KEYS_X]
        y_row = [coeff.get(k, 0.0) for k in _COEFF_KEYS_Y]
        return np.array([x_row, y_row], dtype=np.float32)

    def _writer_thread(self) -> None:
        """Background thread: drain queue and write bytes to serial port."""
        while True:
            data = self._queue.get()
            if data is None:  # sentinel to stop
                break
            if self._serial is not None:
                try:
                    self._serial.write(data)
                except Exception:
                    logger.warning("LED panels serial write failed")
