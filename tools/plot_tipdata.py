#!/usr/bin/env python3
"""Diagnostic plot tool for Kinefly. Subscribes to ZeroMQ FlyState stream.

Shows rolling time-series of tip angle and radius for a single wing.

usage: plot_tipdata.py [--address tcp://localhost:5555] [--tracker left|right]
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
        description="Live tip tracker plot — angle and radius for a single wing.",
    )
    parser.add_argument(
        "--address",
        default="tcp://localhost:5555",
        help="ZeroMQ address to connect to (default: tcp://localhost:5555)",
    )
    parser.add_argument(
        "--tracker",
        choices=["left", "right"],
        default="left",
        help="Wing to plot (default: left)",
    )
    args = parser.parse_args()

    sub = connect_subscriber(args.address)

    angle_buf = _RollingBuffer(WINDOW)
    radius_buf = _RollingBuffer(WINDOW)
    x = np.arange(WINDOW)

    fig, (ax_angle, ax_radius) = plt.subplots(2, 1, figsize=(10, 6))
    fig.suptitle(f"Tip Tracker: {args.tracker}")

    ax_angle.set_title("Tip Angle")
    ax_angle.set_ylabel("angle (rad)")
    ax_angle.set_xlim(0, WINDOW - 1)
    (line_angle,) = ax_angle.plot(x, angle_buf.array(), color="steelblue")

    ax_radius.set_title("Tip Radius")
    ax_radius.set_ylabel("radius (px)")
    ax_radius.set_xlabel("samples")
    ax_radius.set_xlim(0, WINDOW - 1)
    (line_radius,) = ax_radius.plot(x, radius_buf.array(), color="darkorange")

    def update(_frame: int) -> tuple:
        state = recv_state(sub)
        if state is not None:
            part = state.get(args.tracker, {})
            angles = part.get("angles", [])
            radii = part.get("radii", [])
            angle_val = float(angles[0]) if angles else 0.0
            radius_val = float(radii[0]) if radii else 0.0
            angle_buf.push(angle_val)
            radius_buf.push(radius_val)

        angle_data = angle_buf.array()
        radius_data = radius_buf.array()

        line_angle.set_ydata(angle_data)
        ax_angle.set_ylim(
            np.min(angle_data) - 0.1,
            np.max(angle_data) + 0.1,
        )

        line_radius.set_ydata(radius_data)
        ax_radius.set_ylim(
            np.min(radius_data) - 1.0,
            np.max(radius_data) + 1.0,
        )

        return line_angle, line_radius

    ani = animation.FuncAnimation(fig, update, interval=50, blit=False)
    plt.tight_layout()
    plt.show()

    # Keep reference so GC doesn't collect the animation
    _ = ani


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    main()
