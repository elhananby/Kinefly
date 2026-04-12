"""Custom Qt widget helpers for Kinefly GUI."""

from __future__ import annotations

import numpy as np
from PySide6.QtCore import QPoint, Qt, Signal
from PySide6.QtGui import QImage, QPixmap
from PySide6.QtWidgets import QLabel


class ImageLabel(QLabel):
    """QLabel that displays camera frames and translates mouse events to image coordinates.

    Emits mouse_pressed, mouse_moved, mouse_released with float (x, y) in image space.
    """

    mouse_pressed = Signal(float, float)
    mouse_moved = Signal(float, float)
    mouse_released = Signal(float, float)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.setMinimumSize(320, 240)
        self.setMouseTracking(True)
        self._img_size: tuple[int, int] | None = None  # (width, height)
        self._is_pressed = False

    def set_frame(self, rgb_array: np.ndarray) -> None:
        """Display a numpy uint8 RGB array."""
        h, w = rgb_array.shape[:2]
        self._img_size = (w, h)
        bytes_per_line = 3 * w
        qimg = QImage(
            rgb_array.data, w, h, bytes_per_line, QImage.Format.Format_RGB888
        )
        self.setPixmap(
            QPixmap.fromImage(qimg).scaled(
                self.size(),
                Qt.AspectRatioMode.KeepAspectRatio,
                Qt.TransformationMode.SmoothTransformation,
            )
        )

    def _widget_to_image(self, pos: QPoint) -> tuple[float, float] | None:
        """Convert a widget-space pixel position to image-space (float) coordinates.

        Returns None if the position is outside the displayed pixmap region.
        """
        if self._img_size is None or self.pixmap() is None:
            return None
        iw, ih = self._img_size
        pm = self.pixmap()
        pw, ph = pm.width(), pm.height()
        if pw == 0 or ph == 0:
            return None
        # The pixmap is centered inside the label widget.
        x_off = (self.width() - pw) // 2
        y_off = (self.height() - ph) // 2
        rx = pos.x() - x_off
        ry = pos.y() - y_off
        if rx < 0 or ry < 0 or rx >= pw or ry >= ph:
            return None
        img_x = rx * iw / pw
        img_y = ry * ih / ph
        return img_x, img_y

    def mousePressEvent(self, event) -> None:
        pos = self._widget_to_image(event.position().toPoint())
        if pos is not None:
            self._is_pressed = True
            self.mouse_pressed.emit(*pos)

    def mouseMoveEvent(self, event) -> None:
        if self._is_pressed:
            pos = self._widget_to_image(event.position().toPoint())
            if pos is not None:
                self.mouse_moved.emit(*pos)

    def mouseReleaseEvent(self, event) -> None:
        if self._is_pressed:
            self._is_pressed = False
            pos = self._widget_to_image(event.position().toPoint())
            if pos is not None:
                self.mouse_released.emit(*pos)
