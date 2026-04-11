"""Handle class for draggable GUI overlay points.

Ported from nodes/ui.py.
"""

from __future__ import annotations

import cv2
import numpy as np

from kinefly.gui.ui_colors import bgra_dict


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
        self.radiusHit = 6

    def hit_test(self, ptMouse: np.ndarray) -> bool:
        d = np.linalg.norm(self.pt - ptMouse)
        return bool(d < self.radiusHit)

    def draw(self, image: np.ndarray) -> None:
        cv2.circle(image, tuple(self.pt.astype(int)), self.radiusDraw, self.color, -1)
