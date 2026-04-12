"""MainWindow for the Kinefly PySide6 GUI."""

from __future__ import annotations

import numpy as np
from PySide6.QtCore import Signal
from PySide6.QtWidgets import (
    QCheckBox,
    QLabel,
    QMainWindow,
    QPushButton,
    QStatusBar,
    QToolBar,
)

from kinefly.gui.widgets import ImageLabel


class MainWindow(QMainWindow):
    """Main application window.

    Signals
    -------
    track_toggled(part, enabled)
        Emitted when a Track checkbox is toggled. ``part`` is one of
        ``"head"``, ``"abdomen"``, ``"left"``, ``"right"``, ``"aux"``.
    record_toggled(recording)
        Emitted when the Record button is toggled.
    save_background_clicked
        Emitted when the user clicks "Save BG".
    exit_clicked
        Emitted when the user clicks "Exit".

    The mouse signals (``mouse_pressed``, ``mouse_moved``, ``mouse_released``)
    are forwarded from the central :class:`ImageLabel` and carry image-space
    ``(x, y)`` float coordinates.
    """

    track_toggled = Signal(str, bool)
    record_toggled = Signal(bool)
    save_background_clicked = Signal()
    exit_clicked = Signal()

    _BODY_PARTS = ("head", "abdomen", "left", "right", "aux")

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Kinefly")

        # Central image display
        self._image_label = ImageLabel()
        self.setCentralWidget(self._image_label)

        # Forward mouse signals
        self.mouse_pressed = self._image_label.mouse_pressed
        self.mouse_moved = self._image_label.mouse_moved
        self.mouse_released = self._image_label.mouse_released

        # Toolbar -------------------------------------------------------
        toolbar = QToolBar("Controls")
        self.addToolBar(toolbar)

        # Track checkboxes — one per body part
        self._track_checks: dict[str, QCheckBox] = {}
        for part in self._BODY_PARTS:
            cb = QCheckBox(f"Track {part[0].upper()}")
            cb.setChecked(False)
            cb.toggled.connect(lambda checked, p=part: self.track_toggled.emit(p, checked))
            toolbar.addWidget(cb)
            self._track_checks[part] = cb

        toolbar.addSeparator()

        # Record button (toggle)
        self._btn_record = QPushButton("Record")
        self._btn_record.setCheckable(True)
        self._btn_record.toggled.connect(self._on_record_toggled)
        toolbar.addWidget(self._btn_record)

        # Save Background
        btn_bg = QPushButton("Save BG")
        btn_bg.clicked.connect(self.save_background_clicked)
        toolbar.addWidget(btn_bg)

        # Exit
        btn_exit = QPushButton("Exit")
        btn_exit.clicked.connect(self.exit_clicked)
        toolbar.addWidget(btn_exit)

        # Status bar ----------------------------------------------------
        self._status_bar = QStatusBar()
        self.setStatusBar(self._status_bar)
        self._fps_label = QLabel("FPS: --")
        self._rec_label = QLabel("")
        self._status_bar.addWidget(self._fps_label)
        self._status_bar.addWidget(self._rec_label)

        self.resize(800, 600)

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def update_image(self, rgb_array: np.ndarray) -> None:
        """Display a numpy uint8 RGB frame in the central widget."""
        self._image_label.set_frame(rgb_array)

    def set_fps(self, fps: float) -> None:
        """Update the FPS readout in the status bar."""
        self._fps_label.setText(f"FPS: {fps:.1f}")

    def set_track_state(self, part: str, enabled: bool) -> None:
        """Programmatically set a Track checkbox without triggering the signal."""
        cb = self._track_checks.get(part)
        if cb is not None:
            cb.blockSignals(True)
            cb.setChecked(enabled)
            cb.blockSignals(False)

    # ------------------------------------------------------------------
    # Private slots
    # ------------------------------------------------------------------

    def _on_record_toggled(self, checked: bool) -> None:
        self._btn_record.setText("Stop" if checked else "Record")
        self._rec_label.setText("● REC" if checked else "")
        self.record_toggled.emit(checked)
