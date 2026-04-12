"""Handle class for draggable GUI overlay points.

Ported from nodes/ui.py.
"""

from __future__ import annotations

import cv2
import numpy as np

from kinefly.gui.ui_colors import bgra_dict, draw_scale


class Handle:
    def __init__(
        self,
        pt: np.ndarray = np.array([0, 0]),
        color: tuple = bgra_dict["white"],
        name: str | None = None,
    ) -> None:
        self.pt = pt
        self.name = name
        self.scale = 1.0
        self.color = color
        self.radiusDraw = 3
        # Base hit radius at 480p reference; scales with image resolution.
        # Kept deliberately larger than radiusDraw so handles are easy to grab.
        self.radiusHit = 15
        # Cached pixel hit radius updated each draw() call.
        self._hit_radius_px: float = 15.0

    def hit_test(self, ptMouse: np.ndarray) -> bool:
        d = np.linalg.norm(self.pt - ptMouse)
        return bool(d < self._hit_radius_px)

    def draw(self, image: np.ndarray) -> None:
        scale = draw_scale(image)
        radius = max(2, round(self.radiusDraw * scale))
        self._hit_radius_px = max(10.0, self.radiusHit * scale)
        cv2.circle(image, tuple(self.pt.astype(int)), radius, self.color, -1)
