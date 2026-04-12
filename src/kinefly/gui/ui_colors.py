"""Color constants and drawing utilities for the Kinefly GUI.

Ported from nodes/ui.py. Colors are BGRA tuples for use with OpenCV.
"""

from __future__ import annotations

import numpy as np


def draw_scale(image: np.ndarray, reference: int = 480) -> float:
    """Return a scale factor relative to a 480-pixel reference dimension.

    Use this to make handle radii and line thicknesses proportional to the
    camera frame resolution so the overlay looks the same regardless of
    whether the camera is 480p, 1080p, or higher.

    Parameters
    ----------
    image:
        The frame being drawn on (used for its shape).
    reference:
        The resolution at which scale == 1.0 (default: 480 px).
    """
    h, w = image.shape[:2]
    return min(h, w) / reference


# Button side constants (kept for layout use)
SIDE_TOP = 1
SIDE_BOTTOM = 2
SIDE_LEFT = 4
SIDE_RIGHT = 8
SIDE_ALL = SIDE_TOP | SIDE_BOTTOM | SIDE_LEFT | SIDE_RIGHT

# Colors as BGRA tuples (Blue, Green, Red, Alpha) for OpenCV.
bgra_dict: dict[str, tuple[float, float, float, float]] = {
    "black": (0.0, 0.0, 0.0, 0.0),
    "white": (255.0, 255.0, 255.0, 0.0),
    "dark_gray": (64.0, 64.0, 64.0, 0.0),
    "gray": (128.0, 128.0, 128.0, 0.0),
    "light_gray": (192.0, 192.0, 192.0, 0.0),
    "red": (0.0, 0.0, 255.0, 0.0),
    "green": (0.0, 255.0, 0.0, 0.0),
    "blue": (255.0, 0.0, 0.0, 0.0),
    "cyan": (255.0, 255.0, 0.0, 0.0),
    "magenta": (255.0, 0.0, 255.0, 0.0),
    "yellow": (0.0, 255.0, 255.0, 0.0),
    "dark_red": (0.0, 0.0, 128.0, 0.0),
    "dark_green": (0.0, 128.0, 0.0, 0.0),
    "dark_blue": (128.0, 0.0, 0.0, 0.0),
    "dark_cyan": (128.0, 128.0, 0.0, 0.0),
    "dark_magenta": (128.0, 0.0, 128.0, 0.0),
    "dark_yellow": (0.0, 128.0, 128.0, 0.0),
    "light_red": (175.0, 175.0, 255.0, 0.0),
    "light_green": (175.0, 255.0, 175.0, 0.0),
    "light_blue": (255.0, 175.0, 175.0, 0.0),
    "light_cyan": (255.0, 255.0, 175.0, 0.0),
    "light_magenta": (255.0, 175.0, 255.0, 0.0),
    "light_yellow": (175.0, 255.0, 255.0, 0.0),
}
