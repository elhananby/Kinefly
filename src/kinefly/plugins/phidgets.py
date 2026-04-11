"""PhidgetsAnalog voltage output plugin.

Ported from nodes/flystate2phidgetsanalog.py. Maps fly state to 4-channel
analog voltage output via a configurable coefficient matrix.
"""

from __future__ import annotations

import logging
from typing import Any

import numpy as np

from kinefly.config.models import PhidgetsChannelConfig, PhidgetsConfig
from kinefly.core.types import FlyState
from kinefly.plugins.base import OutputPlugin

logger = logging.getLogger(__name__)

# Coefficient keys in the order they appear in the state vector
_COEFF_KEYS = ["offset", "l1", "l2", "lr", "r1", "r2", "rr", "ha", "hr", "aa", "ar", "xi"]


class PhidgetsPlugin(OutputPlugin):
    """PhidgetsAnalog 4-channel voltage output.

    Each channel outputs a voltage computed as a linear combination of
    body part angles, radii, and intensities.
    """

    def __init__(self) -> None:
        self._analog = None
        self._attached = False
        self._config: PhidgetsConfig = PhidgetsConfig()
        self._coefficients: np.ndarray = self._build_coefficient_matrix(self._config.channels)
        self._enable: list[bool] = [ch.enable for ch in self._config.channels]

        # For autorange
        self._state_min = np.full(12, np.inf)
        self._state_max = np.full(12, -np.inf)
        self._count = 0

    def start(self, config: dict[str, Any]) -> None:
        """Connect to the PhidgetsAnalog device.

        Args:
            config: The 'phidgets' section from rig config as a dict.
        """
        self._config = PhidgetsConfig(
            serial=config.get("serial", 0),
            autorange=config.get("autorange", False),
            channels=[
                PhidgetsChannelConfig(
                    enable=ch.get("enable", True),
                    coefficients=ch.get("coefficients", {}),
                )
                for ch in config.get("channels", [])
            ]
            or PhidgetsConfig().channels,
        )
        self._coefficients = self._build_coefficient_matrix(self._config.channels)
        self._enable = [ch.enable for ch in self._config.channels]

        try:
            from Phidget22.Devices.VoltageOutput import VoltageOutput

            self._channels_hw = []
            for i in range(4):
                ch = VoltageOutput()
                ch.setChannel(i)
                if self._config.serial != 0:
                    ch.setDeviceSerialNumber(self._config.serial)
                ch.openWaitForAttachment(5000)
                self._channels_hw.append(ch)
            self._attached = True
            logger.info("PhidgetsAnalog attached (serial=%s)", self._config.serial)
        except ImportError:
            logger.warning("Phidgets22 not installed. Install with: pip install kinefly[phidgets]")
        except Exception:
            logger.exception("Failed to connect to PhidgetsAnalog")

    def on_flystate(self, state: FlyState) -> None:
        self._count += 1
        voltages = self.voltages_from_flystate(state)

        if self._attached:
            for i in range(4):
                if self._enable[i]:
                    try:
                        self._channels_hw[i].setVoltage(voltages[i])
                    except Exception:
                        pass

    def stop(self) -> None:
        if self._attached:
            for ch in self._channels_hw:
                try:
                    ch.setVoltage(0.0)
                    ch.close()
                except Exception:
                    pass
            self._attached = False

    def voltages_from_flystate(self, state: FlyState) -> np.ndarray:
        """Compute 4-channel voltage output from fly state.

        This is the core computation ported from flystate2phidgetsanalog.py.
        """
        angle1_left = state.left.angles[0] if len(state.left.angles) > 0 else 0.0
        angle2_left = state.left.angles[1] if len(state.left.angles) > 1 else 0.0
        radius_left = state.left.radii[0] if len(state.left.radii) > 0 else 0.0
        angle1_right = state.right.angles[0] if len(state.right.angles) > 0 else 0.0
        angle2_right = state.right.angles[1] if len(state.right.angles) > 1 else 0.0
        radius_right = state.right.radii[0] if len(state.right.radii) > 0 else 0.0
        angle_head = state.head.angles[0] if len(state.head.angles) > 0 else 0.0
        radius_head = state.head.radii[0] if len(state.head.radii) > 0 else 0.0
        angle_abdomen = state.abdomen.angles[0] if len(state.abdomen.angles) > 0 else 0.0
        radius_abdomen = state.abdomen.radii[0] if len(state.abdomen.radii) > 0 else 0.0

        state_vec = np.array(
            [
                1.0,
                angle1_left,
                angle2_left,
                radius_left,
                angle1_right,
                angle2_right,
                radius_right,
                angle_head,
                radius_head,
                angle_abdomen,
                radius_abdomen,
                state.aux.intensity,
            ],
            dtype=np.float32,
        )

        if self._config.autorange and self._count > 10:
            self._state_min = np.minimum(self._state_min, state_vec)
            self._state_max = np.maximum(state_vec, self._state_max)
            state_mean = (self._state_min + self._state_max) * 0.5
            d = (self._state_max - state_mean) * 0.001
            self._state_max -= d
            self._state_min += d
            # Autorange updates coefficients dynamically — not implemented in initial port
            # as it modifies self._coefficients in place. Preserved for parity.

        voltages = np.dot(self._coefficients, state_vec)
        return voltages.clip(-10.0, 10.0)

    @staticmethod
    def _build_coefficient_matrix(channels: list[PhidgetsChannelConfig]) -> np.ndarray:
        """Build 4x12 coefficient matrix from channel configs."""
        matrix = np.zeros((4, 12), dtype=np.float32)
        for i, ch in enumerate(channels[:4]):
            for j, key in enumerate(_COEFF_KEYS):
                matrix[i, j] = ch.coefficients.get(key, 0.0)
        return matrix
