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
    subtract_bg_toggled(part, enabled)
        Emitted when a SubtBG checkbox is toggled for a body part.
    invert_color_toggled(enabled)
        Emitted when the InvertColor checkbox is toggled.
    windows_toggled(enabled)
        Emitted when the Windows checkbox is toggled.
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
    subtract_bg_toggled = Signal(str, bool)
    invert_color_toggled = Signal(bool)
    windows_toggled = Signal(bool)
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

        # ── Toolbar 1: Tracking ─────────────────────────────────────────
        toolbar1 = QToolBar("Tracking")
        self.addToolBar(toolbar1)

        # Track checkboxes — one per body part
        self._track_checks: dict[str, QCheckBox] = {}
        for part in self._BODY_PARTS:
            cb = QCheckBox(f"Track {part[0].upper()}")
            cb.setChecked(False)
            cb.toggled.connect(lambda checked, p=part: self.track_toggled.emit(p, checked))
            toolbar1.addWidget(cb)
            self._track_checks[part] = cb

        toolbar1.addSeparator()

        # Record button (toggle)
        self._btn_record = QPushButton("Record")
        self._btn_record.setCheckable(True)
        self._btn_record.toggled.connect(self._on_record_toggled)
        toolbar1.addWidget(self._btn_record)

        # Save Background
        btn_bg = QPushButton("Save BG")
        btn_bg.clicked.connect(self.save_background_clicked)
        toolbar1.addWidget(btn_bg)

        # Exit
        btn_exit = QPushButton("Exit")
        btn_exit.clicked.connect(self.exit_clicked)
        toolbar1.addWidget(btn_exit)

        # ── Toolbar 2: Display options ───────────────────────────────────
        toolbar2 = QToolBar("Display")
        self.addToolBarBreak()
        self.addToolBar(toolbar2)

        # SubtractBG checkboxes — one per body part
        self._subtract_bg_checks: dict[str, QCheckBox] = {}
        for part in self._BODY_PARTS:
            cb = QCheckBox(f"SubtBG {part[0].upper()}")
            cb.setChecked(False)
            cb.setToolTip(
                f"Subtract background from the {part} region before tracking.\n"
                "Use 'Save BG' first to capture a clean background frame."
            )
            cb.toggled.connect(
                lambda checked, p=part: self.subtract_bg_toggled.emit(p, checked)
            )
            toolbar2.addWidget(cb)
            self._subtract_bg_checks[part] = cb

        toolbar2.addSeparator()

        # InvertColor checkbox (global)
        self._cb_invert_color = QCheckBox("InvertColor")
        self._cb_invert_color.setChecked(False)
        self._cb_invert_color.setToolTip(
            "Invert pixel intensities before processing.\n"
            "Use when the fly is lighter than the background."
        )
        self._cb_invert_color.toggled.connect(self.invert_color_toggled)
        toolbar2.addWidget(self._cb_invert_color)

        toolbar2.addSeparator()

        # Windows checkbox — show/hide per-tracker OpenCV debug windows
        self._cb_windows = QCheckBox("Windows")
        self._cb_windows.setChecked(False)
        self._cb_windows.setToolTip(
            "Show per-tracker diagnostic image windows (OpenCV imshow).\n"
            "Useful for debugging tracking quality."
        )
        self._cb_windows.toggled.connect(self.windows_toggled)
        toolbar2.addWidget(self._cb_windows)

        toolbar2.addSeparator()

        # Symmetric — disabled placeholder (feature not yet implemented)
        cb_symmetric = QCheckBox("Symmetric")
        cb_symmetric.setEnabled(False)
        cb_symmetric.setToolTip(
            "NOT YET IMPLEMENTED.\n\n"
            "Symmetric mode mirrors the left-wing tracker geometry to the right side\n"
            "automatically, so both wings share the same wedge shape. Implementing this\n"
            "requires:\n"
            "  1. A bilateral-symmetry axis derived from the fly's body axis.\n"
            "  2. Logic in Fly.update() to copy left-tracker handle positions to right,\n"
            "     reflecting hinge, angle_hi/lo, and radius_inner across the axis.\n"
            "  3. Disabling the right-side handles in the GUI when active.\n"
        )
        toolbar2.addWidget(cb_symmetric)

        # ── Status bar ───────────────────────────────────────────────────
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

    def set_subtract_bg_state(self, part: str, enabled: bool) -> None:
        """Programmatically set a SubtBG checkbox without triggering the signal."""
        cb = self._subtract_bg_checks.get(part)
        if cb is not None:
            cb.blockSignals(True)
            cb.setChecked(enabled)
            cb.blockSignals(False)

    def set_invert_color_state(self, enabled: bool) -> None:
        """Programmatically set the InvertColor checkbox without triggering the signal."""
        self._cb_invert_color.blockSignals(True)
        self._cb_invert_color.setChecked(enabled)
        self._cb_invert_color.blockSignals(False)

    def set_windows_state(self, enabled: bool) -> None:
        """Programmatically set the Windows checkbox without triggering the signal."""
        self._cb_windows.blockSignals(True)
        self._cb_windows.setChecked(enabled)
        self._cb_windows.blockSignals(False)

    # ------------------------------------------------------------------
    # Private slots
    # ------------------------------------------------------------------

    def _on_record_toggled(self, checked: bool) -> None:
        self._btn_record.setText("Stop" if checked else "Record")
        self._rec_label.setText("● REC" if checked else "")
        self.record_toggled.emit(checked)
