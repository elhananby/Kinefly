"""Wingbeat frequency detector via undersampling aliasing analysis.

Ported from nodes/wingbeatdetector.py. Measures wingbeat frequency (typically
180-220 Hz) from low-framerate camera intensity data using undersampling theory.
"""

from __future__ import annotations

import logging

import numpy as np

logger = logging.getLogger(__name__)


class WingbeatDetector:
    def __init__(self, fw_min: float, fw_max: float) -> None:
        self.n = 64
        self.buffer = np.zeros([2 * self.n, 2])  # Holds intensities & framerates
        self.set(fw_min, fw_max)

    def set(self, fw_min: float, fw_max: float) -> None:
        self.i = 0
        self.fw_min = fw_min
        self.fw_max = fw_max
        self.fw_center = (fw_min + fw_max) / 2.0

        self.fs_dict = self.fs_dict_from_wingband(fw_min, fw_max)
        if len(self.fs_dict["fs_range_list"]) > 0:
            (self.fs_lo, self.fs_hi) = self.fs_dict["fs_range_list"][0]
        else:
            (self.fs_lo, self.fs_hi) = (0.0, 0.0)

    def warn(self) -> None:
        logger.warning(
            "Wingbeat detector set to measure frequencies in [%.1f, %.1f] Hz. "
            "Camera framerate must stay in one of: %s",
            self.fw_min,
            self.fw_max,
            self.fs_dict["fs_range_list"],
        )

    def fs_dict_from_wingband(
        self, fw_min: float, fw_max: float
    ) -> dict[str, list]:
        fs_range_list = []
        m_list = []

        bw = fw_max - fw_min
        m = 1
        while True:
            fs_hi = (2.0 * self.fw_center - bw) / m
            fs_lo = max((2.0 * self.fw_center + bw) / (m + 1), 2 * bw)
            if 2 * bw < fs_hi:
                fs_range_list.append([fs_lo, fs_hi])
                m_list.append(m)
            else:
                break
            m += 1

        fs_range_list.reverse()
        m_list.reverse()

        return {"fs_range_list": fs_range_list, "m_list": m_list}

    def wingband_from_fs(
        self, fs_lo: float, fs_hi: float, fw_center: float
    ) -> tuple[float, float, bool]:
        fs = (fs_lo + fs_hi) / 2.0

        if fs != 0.0:
            n = np.round(fw_center / (fs / 2))
            if n * fs / 2 < fw_center:
                fw_min = n * fs / 2
                fw_max = (n + 1) * fs / 2
            else:
                fw_min = (n - 1) * fs / 2
                fw_max = n * fs / 2
            bReversed = (n % 2) == 1
        else:
            fw_min = 0.0
            fw_max = 1.0
            bReversed = False

        return (fw_min, fw_max, bReversed)

    def get_baseband_range(self, fs: float, m: int) -> tuple[float, float]:
        kMax = int(np.floor(2 * self.fw_center / m))
        fbb_min = 0.0
        fbb_max = 0.0

        if m % 2 == 0:
            for k in range(kMax):
                fbb_min_tmp = self.fw_min - k * fs
                fbb_max_tmp = self.fw_max - k * fs
                if fbb_min_tmp >= 0:
                    fbb_min = fbb_min_tmp
                    fbb_max = fbb_max_tmp
                else:
                    break
        else:
            for k in range(kMax):
                fbb_min = -self.fw_min + k * fs
                fbb_max = -self.fw_max + k * fs
                if fbb_max >= 0:
                    break

        return (fbb_min, fbb_max)

    def get_baseband_range_from_framerates(
        self, framerates: np.ndarray
    ) -> tuple[bool, np.ndarray]:
        bValid = False

        fs_lo = np.min(framerates)
        fs_hi = np.max(framerates)
        iRange = 0
        for iRange in range(len(self.fs_dict["fs_range_list"])):
            (fs_min, fs_max) = self.fs_dict["fs_range_list"][iRange]
            if fs_min < fs_lo < fs_hi < fs_max:
                bValid = True
                break

        m = self.fs_dict["m_list"][iRange]

        if bValid:
            fs = np.mean(framerates)
            (fbb_min, fbb_max) = self.get_baseband_range(fs, m)
        else:
            fbb_min = 0.0
            fbb_max = np.inf

        return (bValid, np.array([fbb_min, fbb_max]))

    def freq_from_intensity(self, intensity: float, fs: float = 0) -> float:
        self.buffer[self.i] = [intensity, fs]
        self.buffer[self.i + self.n] = [intensity, fs]

        framerates = self.buffer[(self.i + 1) : (self.i + 1 + self.n), 1]

        (bValid, fbb_range) = self.get_baseband_range_from_framerates(framerates)
        if fbb_range[0] < fbb_range[1]:
            fbb_min = fbb_range[0]
            fbb_max = fbb_range[1]  # noqa: F841
            bReverse = False
        else:
            fbb_min = fbb_range[1]
            fbb_max = fbb_range[0]  # noqa: F841
            bReverse = True

        if bValid:
            intensities = self.buffer[(self.i + 1) : (self.i + 1 + self.n), 0]

            fft = np.fft.rfft(intensities)
            fft[0] = 0
            i_max = np.argmax(np.abs(fft))
            f_offset = np.abs(np.fft.fftfreq(self.n)[i_max]) * fs - fbb_min
            if bReverse:
                freq = self.fw_max - f_offset
            else:
                freq = self.fw_min + f_offset
        else:
            freq = 0.0

        self.i += 1
        self.i %= self.n

        return float(freq)
