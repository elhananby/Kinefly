"""Simple OpenCV image display window.

Ported from nodes/imagewindow.py — no changes required (no ROS imports).
"""

from __future__ import annotations

import cv2
import numpy as np


class ImageWindow:
    def __init__(self, bEnable: bool, name: str) -> None:
        self.image = None
        self.shape = (100, 100)
        self.name = name
        self.bEnable = False
        self.set_enable(bEnable)

    def set_shape(self, shape: tuple[int, int]) -> None:
        self.shape = shape

    def set_ravelled(self, ravel: np.ndarray) -> None:
        self.image = np.reshape(ravel, self.shape)

    def set_image(self, image: np.ndarray | None) -> None:
        if image is not None:
            self.image = image.astype(np.uint8)
            self.shape = image.shape
        else:
            self.image = None

    def show(self) -> None:
        if self.bEnable:
            if self.image is not None and self.image.size > 0:
                img = self.image
            else:
                img = np.zeros(self.shape)

            (h, w) = img.shape[:2]
            if h > 0 and w > 0:
                cv2.imshow(self.name, img)

    def set_enable(self, bEnable: bool) -> None:
        if self.bEnable and not bEnable:
            cv2.destroyWindow(self.name)
        if not self.bEnable and bEnable:
            cv2.namedWindow(self.name)
        self.bEnable = bEnable
