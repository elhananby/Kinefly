#!/usr/bin/env python3
"""Diagnostic plot tool for Kinefly. Subscribes to ZeroMQ FlyState stream.

Shows a scrolling time-series of angles and intensity for a single body part.

usage: plot_tracker.py [--address tcp://localhost:5555] [--tracker left|right|head|abdomen|aux]
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
        description="Live time-series plot of angles and intensity for a single body part.",
    )
    parser.add_argument(
        "--address",
        default="tcp://localhost:5555",
        help="ZeroMQ address to connect to (default: tcp://localhost:5555)",
    )
    parser.add_argument(
        "--tracker",
        choices=["left", "right", "head", "abdomen", "aux"],
        default="left",
        help="Body part to plot (default: left)",
    )
    args = parser.parse_args()

    sub = connect_subscriber(args.address)

    angles_buf = RollingBuffer(WINDOW)
    intensity_buf = RollingBuffer(WINDOW)
    x = np.arange(WINDOW)

    fig, (ax_angle, ax_intensity) = plt.subplots(2, 1, figsize=(10, 6))
    fig.suptitle(f"Tracker: {args.tracker}")

    ax_angle.set_title("Angles")
    ax_angle.set_ylabel("angle (rad)")
    ax_angle.set_xlim(0, WINDOW - 1)
    (line_angle,) = ax_angle.plot(x, angles_buf.array(), color="steelblue")

    ax_intensity.set_title("Intensity")
    ax_intensity.set_ylabel("intensity")
    ax_intensity.set_xlabel("samples")
    ax_intensity.set_xlim(0, WINDOW - 1)
    (line_intensity,) = ax_intensity.plot(x, intensity_buf.array(), color="darkorange")

    def update(_frame: int) -> tuple:
        state = recv_state(sub)
        if state is not None:
            part = state.get(args.tracker, {})
            angles = part.get("angles", [])
            angle_val = float(angles[0]) if angles else 0.0
            intensity_val = float(part.get("intensity", 0.0))
            angles_buf.push(angle_val)
            intensity_buf.push(intensity_val)

        angle_data = angles_buf.array()
        intensity_data = intensity_buf.array()

        line_angle.set_ydata(angle_data)
        ax_angle.set_ylim(
            np.min(angle_data) - 0.1,
            np.max(angle_data) + 0.1,
        )

        line_intensity.set_ydata(intensity_data)
        ax_intensity.set_ylim(
            np.min(intensity_data) - 1.0,
            np.max(intensity_data) + 1.0,
        )

        return line_angle, line_intensity

    ani = animation.FuncAnimation(fig, update, interval=50, blit=False)  # noqa: F841 — kept alive by plt.show()
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    main()
