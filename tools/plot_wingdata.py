#!/usr/bin/env python3
"""Diagnostic plot tool for Kinefly. Subscribes to ZeroMQ FlyState stream.

Shows rolling time-series of left and right wing angles and intensities.

usage: plot_wingdata.py [--address tcp://localhost:5555]
"""

from __future__ import annotations

import argparse
import collections
import logging
import sys
from typing import Any

import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np

logger = logging.getLogger(__name__)
WINDOW = 200  # rolling window size


def connect_subscriber(address: str) -> Any:
    """Connect a ZeroMQ SUB socket to address. Returns socket or None."""
    try:
        import zmq

        ctx = zmq.Context()
        sub = ctx.socket(zmq.SUB)
        sub.connect(address)
        sub.setsockopt(zmq.SUBSCRIBE, b"")
        sub.setsockopt(zmq.RCVTIMEO, 100)  # 100ms receive timeout
        return sub
    except ImportError:
        logger.error("pyzmq not installed. Install with: pip install kinefly[zmq]")
        sys.exit(1)


def recv_state(sub: Any) -> dict | None:
    """Non-blocking receive. Returns state dict or None on timeout."""
    try:
        import msgpack

        data = sub.recv()
        return msgpack.unpackb(data, raw=False)
    except Exception:  # timeout or error
        return None


class _RollingBuffer:
    """Fixed-size rolling buffer backed by collections.deque."""

    def __init__(self, size: int) -> None:
        self._buf: collections.deque[float] = collections.deque([0.0] * size, maxlen=size)

    def push(self, v: float) -> None:
        self._buf.append(v)

    def array(self) -> np.ndarray:
        return np.array(self._buf)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Live dual-wing angle and intensity plot (left + right).",
    )
    parser.add_argument(
        "--address",
        default="tcp://localhost:5555",
        help="ZeroMQ address to connect to (default: tcp://localhost:5555)",
    )
    args = parser.parse_args()

    sub = connect_subscriber(args.address)

    left_angle_buf = _RollingBuffer(WINDOW)
    right_angle_buf = _RollingBuffer(WINDOW)
    left_intensity_buf = _RollingBuffer(WINDOW)
    right_intensity_buf = _RollingBuffer(WINDOW)
    x = np.arange(WINDOW)

    fig, (ax_angle, ax_intensity) = plt.subplots(2, 1, figsize=(10, 6))
    fig.suptitle("Wing Data")

    ax_angle.set_title("Wing Angles")
    ax_angle.set_ylabel("angle (rad)")
    ax_angle.set_xlim(0, WINDOW - 1)
    (line_left_angle,) = ax_angle.plot(x, left_angle_buf.array(), color="green", label="left")
    (line_right_angle,) = ax_angle.plot(x, right_angle_buf.array(), color="red", label="right")
    ax_angle.legend(loc="upper right")

    ax_intensity.set_title("Wing Intensity")
    ax_intensity.set_ylabel("intensity")
    ax_intensity.set_xlabel("samples")
    ax_intensity.set_xlim(0, WINDOW - 1)
    (line_left_intensity,) = ax_intensity.plot(
        x, left_intensity_buf.array(), color="green", label="left"
    )
    (line_right_intensity,) = ax_intensity.plot(
        x, right_intensity_buf.array(), color="red", label="right"
    )
    ax_intensity.legend(loc="upper right")

    def update(_frame: int) -> tuple:
        state = recv_state(sub)
        if state is not None:
            left = state.get("left", {})
            right = state.get("right", {})

            left_angles = left.get("angles", [])
            right_angles = right.get("angles", [])

            left_angle_buf.push(float(left_angles[0]) if left_angles else 0.0)
            right_angle_buf.push(float(right_angles[0]) if right_angles else 0.0)
            left_intensity_buf.push(float(left.get("intensity", 0.0)))
            right_intensity_buf.push(float(right.get("intensity", 0.0)))

        la = left_angle_buf.array()
        ra = right_angle_buf.array()
        li = left_intensity_buf.array()
        ri = right_intensity_buf.array()

        line_left_angle.set_ydata(la)
        line_right_angle.set_ydata(ra)
        combined_angles = np.concatenate([la, ra])
        ax_angle.set_ylim(
            np.min(combined_angles) - 0.1,
            np.max(combined_angles) + 0.1,
        )

        line_left_intensity.set_ydata(li)
        line_right_intensity.set_ydata(ri)
        combined_intensity = np.concatenate([li, ri])
        ax_intensity.set_ylim(
            np.min(combined_intensity) - 1.0,
            np.max(combined_intensity) + 1.0,
        )

        return line_left_angle, line_right_angle, line_left_intensity, line_right_intensity

    ani = animation.FuncAnimation(fig, update, interval=50, blit=False)
    plt.tight_layout()
    plt.show()

    # Keep reference so GC doesn't collect the animation
    _ = ani


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    main()
