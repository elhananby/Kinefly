#!/usr/bin/env python3
"""Diagnostic plot tool for Kinefly. Subscribes to ZeroMQ FlyState stream.

Shows rolling time-series of tip angle and radius for a single wing.

usage: plot_tipdata.py [--address tcp://localhost:5555] [--tracker left|right]
"""

from __future__ import annotations

import argparse
import logging
import sys
from pathlib import Path

# Allow running as a standalone script from any directory
sys.path.insert(0, str(Path(__file__).parent))
from _plot_common import WINDOW, RollingBuffer, connect_subscriber, recv_state  # isort: skip

import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np

logger = logging.getLogger(__name__)


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

    angle_buf = RollingBuffer(WINDOW)
    radius_buf = RollingBuffer(WINDOW)
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

    ani = animation.FuncAnimation(fig, update, interval=50, blit=False)  # noqa: F841 — kept alive by plt.show()
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    main()
