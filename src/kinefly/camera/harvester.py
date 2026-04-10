"""GenICam camera source via Harvesters library."""

from __future__ import annotations

import logging
import time

import numpy as np

from kinefly.camera.base import CameraSource

logger = logging.getLogger(__name__)


class HarvesterCamera(CameraSource):
    """Camera source using Harvesters for GenICam-compliant cameras.

    Args:
        cti_file: Path to the GenTL producer CTI file.
        serial: Camera serial number. Empty string for first available camera.
        framerate: Requested framerate.
        exposure_us: Exposure time in microseconds.
        gain: Camera gain.
    """

    def __init__(
        self,
        cti_file: str,
        serial: str = "",
        framerate: float = 60.0,
        exposure_us: int = 5000,
        gain: float = 0.0,
    ) -> None:
        self._cti_file = cti_file
        self._serial = serial
        self._requested_fps = framerate
        self._exposure_us = exposure_us
        self._gain = gain
        self._harvester = None
        self._acquirer = None
        self._fps: float = framerate
        self._resolution: tuple[int, int] = (640, 480)

    def open(self) -> None:
        try:
            from harvesters.core import Harvester
        except ImportError:
            raise ImportError(
                "harvesters not installed. Install with: pip install kinefly[harvester]"
            )

        self._harvester = Harvester()
        self._harvester.add_file(self._cti_file)
        self._harvester.update()

        if len(self._harvester.device_info_list) == 0:
            raise RuntimeError("No GenICam cameras found")

        if self._serial:
            self._acquirer = self._harvester.create(
                {"serial_number": self._serial}
            )
        else:
            self._acquirer = self._harvester.create()

        node_map = self._acquirer.remote_device.node_map
        try:
            node_map.ExposureTime.value = self._exposure_us
        except Exception:
            logger.warning("Could not set ExposureTime")
        try:
            node_map.AcquisitionFrameRate.value = self._requested_fps
        except Exception:
            logger.warning("Could not set AcquisitionFrameRate")
        try:
            node_map.Gain.value = self._gain
        except Exception:
            logger.warning("Could not set Gain")

        self._acquirer.start()

        w = node_map.Width.value
        h = node_map.Height.value
        self._resolution = (w, h)
        self._fps = self._requested_fps

    def read(self) -> tuple[np.ndarray, float]:
        if self._acquirer is None:
            raise RuntimeError("Camera not opened. Call open() first.")

        with self._acquirer.fetch() as buffer:
            component = buffer.payload.components[0]
            frame = component.data.reshape(component.height, component.width)
            frame = frame.copy()
            timestamp = time.time()

        return frame, timestamp

    def close(self) -> None:
        if self._acquirer is not None:
            self._acquirer.stop()
            self._acquirer.destroy()
            self._acquirer = None
        if self._harvester is not None:
            self._harvester.reset()
            self._harvester = None

    @property
    def fps(self) -> float:
        return self._fps

    @property
    def resolution(self) -> tuple[int, int]:
        return self._resolution
