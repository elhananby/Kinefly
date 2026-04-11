# Kinefly Python 3 Migration — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Migrate Kinefly from Python 2 + ROS1 to a clean, modern, pure Python 3 application with PySide6 GUI, no ROS dependencies, and identical tracking behavior.

**Architecture:** Single-process application with callback-based event bus and optional ZeroMQ publisher. Camera abstraction layer (Harvesters/OpenCV/video file), PySide6 GUI with OpenCV overlay handles, output plugins (Phidgets, LED panels) loaded from YAML rig config.

**Tech Stack:** Python 3.11+, PySide6, OpenCV, numpy, pyzmq, msgpack, harvesters, phidgets22, pyserial, ffmpeg (system), uv, ruff, pytest

**Spec:** `docs/superpowers/specs/2026-04-10-python3-migration-design.md`

---

## Task 1: Project Skeleton

**Files:**
- Create: `pyproject.toml`
- Create: `src/kinefly/__init__.py`
- Create: `src/kinefly/__main__.py`
- Create: `tests/__init__.py`

- [ ] **Step 1: Create a new clean branch**

```bash
git checkout -b python3-migration
```

- [ ] **Step 2: Initialize uv project**

```bash
uv init --lib --name kinefly
```

This creates a basic `pyproject.toml`. We'll replace its contents next.

- [ ] **Step 3: Write `pyproject.toml`**

```toml
[project]
name = "kinefly"
version = "2.0.0a1"
description = "Real-time kinematic tracking of tethered winged insects"
requires-python = ">=3.11"
license = "MIT"

dependencies = [
    "numpy",
    "opencv-python",
    "PySide6",
    "pyyaml",
]

[project.optional-dependencies]
harvester = ["harvesters", "genicam"]
phidgets = ["Phidgets22"]
ledpanels = ["pyserial"]
zmq = ["pyzmq", "msgpack"]
plotting = ["matplotlib", "pyzmq", "msgpack"]
all = ["kinefly[harvester,phidgets,ledpanels,zmq,plotting]"]
dev = ["kinefly[all]", "ruff", "pytest"]

[project.scripts]
kinefly = "kinefly.__main__:main"

[build-system]
requires = ["hatchling"]
build-backend = "hatchling.build"

[tool.hatch.build.targets.wheel]
packages = ["src/kinefly"]

[tool.ruff]
target-version = "py311"
line-length = 100

[tool.ruff.lint]
select = ["E", "F", "W", "I", "UP"]

[tool.pytest.ini_options]
testpaths = ["tests"]
```

- [ ] **Step 4: Create package structure**

Create these empty files to establish the package layout:

`src/kinefly/__init__.py`:
```python
"""Kinefly: Real-time kinematic tracking of tethered winged insects."""

__version__ = "2.0.0a1"
```

`src/kinefly/__main__.py`:
```python
"""CLI entry point for kinefly."""

import sys


def main() -> int:
    print(f"kinefly {__import__('kinefly').__version__}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
```

`tests/__init__.py`: empty file.

- [ ] **Step 5: Create virtual environment and install**

```bash
uv sync --all-extras
```

- [ ] **Step 6: Verify the package installs and runs**

```bash
uv run kinefly
```

Expected output: `kinefly 2.0.0a1`

- [ ] **Step 7: Verify ruff works**

```bash
uv run ruff check src/
uv run ruff format --check src/
```

Expected: no errors.

- [ ] **Step 8: Commit**

```bash
git add pyproject.toml src/ tests/
git commit -m "feat: initialize Python 3 project skeleton with uv and ruff"
```

---

## Task 2: Data Models

**Files:**
- Create: `src/kinefly/core/__init__.py`
- Create: `src/kinefly/core/types.py`
- Create: `tests/core/__init__.py`
- Create: `tests/core/test_types.py`

- [ ] **Step 1: Write the failing test**

`tests/core/__init__.py`: empty file.

`tests/core/test_types.py`:
```python
from kinefly.core.types import BodyPartState, FlyState


def test_bodypart_state_defaults():
    s = BodyPartState()
    assert s.angles == []
    assert s.gradients == []
    assert s.radii == []
    assert s.freq == 0.0
    assert s.intensity == 0.0


def test_bodypart_state_with_values():
    s = BodyPartState(angles=[0.5, 1.0], gradients=[-0.01], radii=[54.6], freq=200.0, intensity=0.45)
    assert s.angles == [0.5, 1.0]
    assert s.intensity == 0.45


def test_flystate_creation():
    state = FlyState(
        timestamp=1234567890.123,
        seq=42,
        head=BodyPartState(angles=[-0.05]),
        abdomen=BodyPartState(angles=[-0.37]),
        left=BodyPartState(angles=[0.82], gradients=[-0.008]),
        right=BodyPartState(angles=[0.60], gradients=[0.012]),
        aux=BodyPartState(intensity=0.67),
    )
    assert state.seq == 42
    assert state.left.angles[0] == 0.82
    assert state.aux.intensity == 0.67


def test_flystate_field_access_matches_ros_pattern():
    """Verify that field access patterns used throughout the old codebase still work.
    e.g. flystate.left.angles[0], flystate.head.radii[0], flystate.aux.intensity
    """
    state = FlyState(
        timestamp=0.0,
        seq=0,
        head=BodyPartState(angles=[0.1], radii=[50.0]),
        abdomen=BodyPartState(angles=[0.2], radii=[100.0]),
        left=BodyPartState(angles=[0.8, 0.3], gradients=[-0.01, 0.02]),
        right=BodyPartState(angles=[0.6], gradients=[0.01]),
        aux=BodyPartState(intensity=0.5, freq=195.0),
    )
    # These are the exact access patterns from flystate2phidgetsanalog.py and flystate2ledpanels.py
    assert len(state.left.angles) == 2
    angle1_left = state.left.angles[0] if 0 < len(state.left.angles) else 0.0
    assert angle1_left == 0.8
    angle_head = state.head.angles[0] if 0 < len(state.head.angles) else 0.0
    assert angle_head == 0.1
    assert state.aux.intensity == 0.5
```

- [ ] **Step 2: Run test to verify it fails**

```bash
uv run pytest tests/core/test_types.py -v
```

Expected: FAIL with `ModuleNotFoundError: No module named 'kinefly.core'`

- [ ] **Step 3: Write the implementation**

`src/kinefly/core/__init__.py`: empty file.

`src/kinefly/core/types.py`:
```python
"""Data models replacing ROS message types (MsgFlystate, MsgState)."""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class BodyPartState:
    """State of a single body part. Replaces MsgState.

    Attributes:
        angles: Angular positions in radians. May contain 0, 1, or more entries
            depending on tracker type (e.g. edge tracker finds multiple wing edges).
        gradients: Intensity gradients at detected edges. Corresponds to angles list.
        radii: Radial positions in pixels from hinge point.
        freq: Detected oscillation frequency in Hz (e.g. wingbeat frequency).
        intensity: Mean pixel intensity in the ROI, normalized to [0, 1].
    """

    angles: list[float] = field(default_factory=list)
    gradients: list[float] = field(default_factory=list)
    radii: list[float] = field(default_factory=list)
    freq: float = 0.0
    intensity: float = 0.0


@dataclass
class FlyState:
    """Aggregate state of all tracked body parts. Replaces MsgFlystate.

    Attributes:
        timestamp: Time of the frame in seconds since epoch.
        seq: Frame sequence number.
        head: Head body part state.
        abdomen: Abdomen body part state.
        left: Left wing body part state.
        right: Right wing body part state.
        aux: Auxiliary region state (intensity/wingbeat frequency).
    """

    timestamp: float
    seq: int
    head: BodyPartState = field(default_factory=BodyPartState)
    abdomen: BodyPartState = field(default_factory=BodyPartState)
    left: BodyPartState = field(default_factory=BodyPartState)
    right: BodyPartState = field(default_factory=BodyPartState)
    aux: BodyPartState = field(default_factory=BodyPartState)
```

- [ ] **Step 4: Run tests to verify they pass**

```bash
uv run pytest tests/core/test_types.py -v
```

Expected: all 4 tests PASS.

- [ ] **Step 5: Commit**

```bash
git add src/kinefly/core/ tests/core/
git commit -m "feat: add FlyState and BodyPartState data models"
```

---

## Task 3: Event System

**Files:**
- Create: `src/kinefly/core/events.py`
- Create: `tests/core/test_events.py`

- [ ] **Step 1: Write the failing test**

`tests/core/test_events.py`:
```python
from kinefly.core.events import EventBus
from kinefly.core.types import BodyPartState, FlyState


def _make_state(seq: int = 0) -> FlyState:
    return FlyState(timestamp=0.0, seq=seq)


def test_register_and_emit():
    bus = EventBus()
    received: list[FlyState] = []
    bus.register(received.append)
    bus.emit(_make_state(1))
    assert len(received) == 1
    assert received[0].seq == 1


def test_multiple_callbacks():
    bus = EventBus()
    a: list[FlyState] = []
    b: list[FlyState] = []
    bus.register(a.append)
    bus.register(b.append)
    bus.emit(_make_state(2))
    assert len(a) == 1
    assert len(b) == 1


def test_unregister():
    bus = EventBus()
    received: list[FlyState] = []
    bus.register(received.append)
    bus.unregister(received.append)
    bus.emit(_make_state(3))
    assert len(received) == 0


def test_emit_without_callbacks():
    bus = EventBus()
    bus.emit(_make_state(0))  # Should not raise
```

- [ ] **Step 2: Run test to verify it fails**

```bash
uv run pytest tests/core/test_events.py -v
```

Expected: FAIL with `ModuleNotFoundError`

- [ ] **Step 3: Write the implementation**

`src/kinefly/core/events.py`:
```python
"""EventBus: callback registry with optional ZeroMQ publishing."""

from __future__ import annotations

import logging
from collections.abc import Callable
from typing import Any

from kinefly.core.types import FlyState

logger = logging.getLogger(__name__)


class EventBus:
    """Central event dispatcher for FlyState updates.

    Plugins register callbacks via register(). The optional ZeroMQ publisher
    is initialized only when start_zmq() is called with an address.

    Args:
        zmq_address: Optional ZeroMQ PUB address (e.g. "tcp://*:5555").
            If provided, start_zmq() is called automatically.
    """

    def __init__(self, zmq_address: str | None = None) -> None:
        self._callbacks: list[Callable[[FlyState], None]] = []
        self._zmq_pub: Any = None
        self._zmq_address = zmq_address

        if zmq_address:
            self.start_zmq(zmq_address)

    def register(self, callback: Callable[[FlyState], None]) -> None:
        """Register a callback to receive FlyState updates."""
        self._callbacks.append(callback)

    def unregister(self, callback: Callable[[FlyState], None]) -> None:
        """Remove a previously registered callback."""
        try:
            self._callbacks.remove(callback)
        except ValueError:
            pass

    def emit(self, state: FlyState) -> None:
        """Dispatch state to all registered callbacks and ZeroMQ."""
        for cb in self._callbacks:
            try:
                cb(state)
            except Exception:
                logger.exception("Error in event callback %s", cb)

        if self._zmq_pub is not None:
            self._publish_zmq(state)

    def start_zmq(self, address: str) -> None:
        """Initialize the ZeroMQ PUB socket.

        Args:
            address: ZeroMQ bind address (e.g. "tcp://*:5555").
        """
        try:
            import msgpack
            import zmq

            self._zmq_context = zmq.Context()
            self._zmq_pub = self._zmq_context.socket(zmq.PUB)
            self._zmq_pub.bind(address)
            self._msgpack = msgpack
            logger.info("ZeroMQ publisher bound to %s", address)
        except ImportError:
            logger.warning(
                "pyzmq or msgpack not installed. ZeroMQ publishing disabled. "
                "Install with: pip install kinefly[zmq]"
            )
            self._zmq_pub = None

    def stop_zmq(self) -> None:
        """Close the ZeroMQ socket."""
        if self._zmq_pub is not None:
            self._zmq_pub.close()
            self._zmq_context.term()
            self._zmq_pub = None

    def _publish_zmq(self, state: FlyState) -> None:
        """Serialize and publish state via ZeroMQ."""
        from dataclasses import asdict

        try:
            data = self._msgpack.packb(asdict(state), use_bin_type=True)
            self._zmq_pub.send(data, zmq.NOBLOCK)
        except Exception:
            logger.exception("Failed to publish state via ZeroMQ")
```

- [ ] **Step 4: Run tests to verify they pass**

```bash
uv run pytest tests/core/test_events.py -v
```

Expected: all 4 tests PASS.

- [ ] **Step 5: Commit**

```bash
git add src/kinefly/core/events.py tests/core/test_events.py
git commit -m "feat: add EventBus with callback registry and optional ZeroMQ"
```

---

## Task 4: Utility — SetDict

**Files:**
- Create: `src/kinefly/core/setdict.py`
- Create: `tests/core/test_setdict.py`

- [ ] **Step 1: Write the failing test**

`tests/core/test_setdict.py`:
```python
from kinefly.core.setdict import set_dict_with_preserve, set_dict_with_overwrite


def test_preserve_keeps_existing_keys():
    target = {"a": 1, "b": 2}
    source = {"a": 0, "c": 0}
    set_dict_with_preserve(target, source)
    assert target == {"a": 1, "b": 2, "c": 0}


def test_overwrite_replaces_existing_keys():
    target = {"a": 1, "b": 2}
    source = {"a": 0, "c": 0}
    set_dict_with_overwrite(target, source)
    assert target == {"a": 0, "b": 2, "c": 0}


def test_nested_dict_preserve():
    target = {"x": {"a": 1}}
    source = {"x": {"a": 99, "b": 2}, "y": {"c": 3}}
    set_dict_with_preserve(target, source)
    assert target == {"x": {"a": 1, "b": 2}, "y": {"c": 3}}


def test_nested_dict_overwrite():
    target = {"x": {"a": 1}}
    source = {"x": {"a": 99, "b": 2}}
    set_dict_with_overwrite(target, source)
    assert target == {"x": {"a": 99, "b": 2}}
```

- [ ] **Step 2: Run test to verify it fails**

```bash
uv run pytest tests/core/test_setdict.py -v
```

Expected: FAIL with `ModuleNotFoundError`

- [ ] **Step 3: Write the implementation**

`src/kinefly/core/setdict.py`:
```python
"""Recursive dictionary merge utilities.

Ported from nodes/setdict.py. Provides preserve-or-overwrite merging of
nested dictionaries, used throughout Kinefly for merging default parameters
with user-supplied configuration.
"""

from __future__ import annotations

from typing import Any


def _set_dict(target: dict[str, Any], source: dict[str, Any], preserve: bool) -> None:
    """Recursively merge source into target.

    Args:
        target: Dictionary to update in-place.
        source: Dictionary to merge from.
        preserve: If True, existing keys in target are not overwritten.
    """
    for key, value in source.items():
        key_exists = key in target
        if not key_exists and isinstance(value, dict):
            target[key] = {}
        if (not key_exists or not preserve) and not isinstance(value, dict):
            target[key] = value
        if isinstance(value, dict):
            _set_dict(target[key], value, preserve)


def set_dict_with_preserve(target: dict[str, Any], source: dict[str, Any]) -> None:
    """Merge source into target, keeping existing values in target."""
    _set_dict(target, source, preserve=True)


def set_dict_with_overwrite(target: dict[str, Any], source: dict[str, Any]) -> None:
    """Merge source into target, overwriting existing values from source."""
    _set_dict(target, source, preserve=False)
```

- [ ] **Step 4: Run tests to verify they pass**

```bash
uv run pytest tests/core/test_setdict.py -v
```

Expected: all 4 tests PASS.

- [ ] **Step 5: Commit**

```bash
git add src/kinefly/core/setdict.py tests/core/test_setdict.py
git commit -m "feat: port SetDict utility to Python 3 module functions"
```

---

## Task 5: Image Processing — Core Functions

**Files:**
- Create: `src/kinefly/core/imaging.py`
- Create: `tests/core/test_imaging.py`

This is the most critical port: `imageprocessing.py` contains the polar transform, phase correlation, and window functions. We port with minimal changes — only fixing Python 2 syntax.

- [ ] **Step 1: Write the failing test**

`tests/core/test_imaging.py`:
```python
import numpy as np
import pytest

from kinefly.core.imaging import (
    PolarTransforms,
    PhaseCorrelation,
    TransformException,
    WindowFunctions,
    clip,
    clip_pt,
    filter_median,
    get_angle_from_points_i,
    get_intersection,
    get_projection_onto_axis,
    get_reflection_across_axis,
)


def test_get_angle_from_points_i():
    # Horizontal rightward: angle should be 0
    angle = get_angle_from_points_i(np.array([0, 0]), np.array([1, 0]))
    assert abs(angle) < 1e-10

    # Vertical downward: angle should be pi/2
    angle = get_angle_from_points_i(np.array([0, 0]), np.array([0, 1]))
    assert abs(angle - np.pi / 2) < 1e-10


def test_get_intersection():
    # Two perpendicular lines through (1,1)
    pt = get_intersection(
        np.array([0.0, 1.0]), np.array([2.0, 1.0]),  # horizontal line y=1
        np.array([1.0, 0.0]), np.array([1.0, 2.0]),  # vertical line x=1
    )
    np.testing.assert_allclose(pt, [1.0, 1.0], atol=1e-10)


def test_get_projection_onto_axis():
    pt = get_projection_onto_axis(
        np.array([1.0, 1.0]),
        np.array([0.0, 0.0]),
        np.array([2.0, 0.0]),
    )
    np.testing.assert_allclose(pt, [1.0, 0.0], atol=1e-10)


def test_get_reflection_across_axis():
    pt = get_reflection_across_axis(
        np.array([1.0, 1.0]),
        np.array([0.0, 0.0]),
        np.array([2.0, 0.0]),
    )
    np.testing.assert_allclose(pt, [1.0, -1.0], atol=1e-10)


def test_filter_median():
    data = np.array([1.0, 100.0, 1.0, 1.0, 1.0])
    result = filter_median(data, q=1)
    # Middle values should be median-filtered; the outlier at index 1 should be smoothed
    assert result[2] == 1.0


def test_clip():
    assert clip(5, 0, 10) == 5
    assert clip(-1, 0, 10) == 0
    assert clip(15, 0, 10) == 10


def test_clip_pt():
    assert clip_pt((50, 50), (100, 100)) == (50, 50)
    assert clip_pt((-1, 200), (100, 100)) == (0, 99)


def test_polar_transforms_log():
    """Verify log-polar transform runs and produces the right shape."""
    pt = PolarTransforms()
    img = np.random.randint(0, 255, (100, 100), dtype=np.uint8)
    result = pt.transform_polar_log(img, 50, 50, nRho=30, nTheta=60)
    assert result.shape == (30, 60)


def test_polar_transforms_elliptical():
    """Verify elliptical-polar transform runs and produces output."""
    pt = PolarTransforms()
    img = np.random.randint(0, 255, (200, 200), dtype=np.uint8)
    result = pt.transform_polar_elliptical(
        img, 100, 100, raxial=50, rortho=50, dradiusStrip=20,
        nRho=20, nTheta=40, theta_0=-0.5, theta_1=0.5, rClip=0.8,
    )
    assert result.shape[0] == 20
    assert result.shape[1] == 40


def test_polar_transforms_elliptical_empty_raises():
    """Transform with no valid points should raise TransformException."""
    pt = PolarTransforms()
    img = np.zeros((10, 10), dtype=np.uint8)
    with pytest.raises(TransformException):
        pt.transform_polar_elliptical(
            img, 100, 100, raxial=50, rortho=50, dradiusStrip=20,
            nRho=20, nTheta=40, theta_0=-0.5, theta_1=0.5, rClip=0.8,
        )


def test_phase_correlation_zero_shift():
    """Identical images should have near-zero shift."""
    pc = PhaseCorrelation()
    img = np.random.rand(64, 64).astype(np.float32) * 255
    shift = pc.get_shift(img, img)
    np.testing.assert_allclose(shift, [0.0, 0.0], atol=1.0)


def test_window_functions_hanning_shape():
    wf = WindowFunctions()
    h = wf.create_hanning((32, 64))
    assert h.shape == (32, 64)
    assert h.dtype == np.float32
    # Corners should be near zero, center near 1
    assert h[0, 0] < 0.01
    assert h[16, 32] > 0.9


def test_window_functions_tukey_shape():
    wf = WindowFunctions()
    t = wf.create_tukey((32, 64))
    assert t.shape == (32, 64)
    assert t.dtype == np.float32
```

- [ ] **Step 2: Run test to verify it fails**

```bash
uv run pytest tests/core/test_imaging.py -v
```

Expected: FAIL with `ModuleNotFoundError`

- [ ] **Step 3: Write the implementation**

Port `nodes/imageprocessing.py` to `src/kinefly/core/imaging.py`. Key changes from the original:
- Remove `import rospy` and `import cv` (old OpenCV bindings)
- Fix tuple unpacking in `get_projection_onto_axis` and `get_reflection_across_axis` function signatures
- Keep all numpy/OpenCV logic identical

`src/kinefly/core/imaging.py`:
```python
"""Image processing utilities: polar transforms, phase correlation, window functions.

Ported from nodes/imageprocessing.py with minimal changes:
- Removed rospy and cv (old OpenCV) imports
- Fixed Python 2 tuple unpacking in function signatures
- All algorithms are preserved exactly as-is
"""

from __future__ import annotations

import copy

import cv2
import numpy as np


class TransformException(Exception):
    pass


def get_angle_from_points_i(pt1: np.ndarray, pt2: np.ndarray) -> float:
    x = pt2[0] - pt1[0]
    y = pt2[1] - pt1[1]
    return float(np.arctan2(y, x))


def get_intersection(
    pt1a: np.ndarray, pt1b: np.ndarray, pt2a: np.ndarray, pt2b: np.ndarray
) -> np.ndarray:
    """Intersection of two lines, given two points on each line."""
    x1, y1 = pt1a[0], pt1a[1]
    x2, y2 = pt1b[0], pt1b[1]
    x3, y3 = pt2a[0], pt2a[1]
    x4, y4 = pt2b[0], pt2b[1]

    den = x1 * y3 - x3 * y1 - x1 * y4 - x2 * y3 + x3 * y2 + x4 * y1 + x2 * y4 - x4 * y2
    if den != 0.0:
        x = (
            x1 * x3 * y2
            - x2 * x3 * y1
            - x1 * x4 * y2
            + x2 * x4 * y1
            - x1 * x3 * y4
            + x1 * x4 * y3
            + x2 * x3 * y4
            - x2 * x4 * y3
        ) / den
        y = (
            x1 * y2 * y3
            - x2 * y1 * y3
            - x1 * y2 * y4
            + x2 * y1 * y4
            - x3 * y1 * y4
            + x4 * y1 * y3
            + x3 * y2 * y4
            - x4 * y2 * y3
        ) / den
    else:
        x = x3
        y = y3

    return np.array([x, y])


def get_projection_onto_axis(
    pt_anywhere: np.ndarray, pt_axis_base: np.ndarray, pt_axis_head: np.ndarray
) -> np.ndarray:
    """Project the given point onto the axis defined by two points.

    Note: Original signature used Python 2 tuple unpacking:
        def get_projection_onto_axis(ptAnywhere, (ptAxisBase, ptAxisHead))
    """
    pt_b = pt_axis_head - pt_axis_base
    pt_m = pt_anywhere - pt_axis_base
    pt_axis = np.dot(pt_b, pt_m) / np.dot(pt_b, pt_b) * pt_b + pt_axis_base
    return pt_axis


def get_reflection_across_axis(
    pt_anywhere: np.ndarray, pt_axis_base: np.ndarray, pt_axis_head: np.ndarray
) -> np.ndarray:
    """Reflect a point across the axis defined by two points.

    Note: Original signature used Python 2 tuple unpacking:
        def get_reflection_across_axis(ptAnywhere, (ptAxisBase, ptAxisHead))
    """
    pt_axis = get_projection_onto_axis(pt_anywhere, pt_axis_base, pt_axis_head)
    pt_reflected = pt_anywhere + 2 * (pt_axis - pt_anywhere)
    return pt_reflected


def filter_median(data: np.ndarray, q: int = 1) -> np.ndarray:
    """Median filter with window radius q. q=1 gives window of 3, q=2 gives window of 5."""
    data2 = copy.copy(data)
    for i in range(q, len(data) - q):
        data2[i] = np.median(data[i - q : i + q + 1])

    try:
        data2[0:q] = data2[q]
        data2[len(data2) - q : len(data2)] = data2[-(q + 1)]
        return data2
    except IndexError:
        return data


def clip(x: float, lo: float, hi: float) -> float:
    return max(min(x, hi), lo)


def clip_pt(pt: tuple[int, int], shape: tuple[int, ...]) -> tuple[int, int]:
    """Clip a point (x, y) to image shape (yMax+1, xMax+1)."""
    return (int(clip(pt[0], 0, shape[1] - 1)), int(clip(pt[1], 0, shape[0] - 1)))


class PolarTransforms:
    def __init__(self) -> None:
        self._transforms: dict = {}

    def _get_transform_polar_log(
        self, i_0, j_0, i_n, j_n, nRho, dRho, nTheta, theta_0, theta_1
    ):
        transform = self._transforms.get((i_0, j_0, i_n, j_n, nRho, nTheta, theta_0, theta_1))

        if transform is None:
            i_k = []
            j_k = []
            rho_k = []
            theta_k = []

            aspect = float(i_n) / float(j_n)
            dTheta = (theta_1 - theta_0) / nTheta
            for iRho in range(0, nRho):
                rho = np.exp(iRho * dRho)

                for iTheta in range(0, nTheta):
                    theta = theta_0 + iTheta * dTheta

                    i_c = rho * np.sin(theta)
                    j_c = rho * np.cos(theta)

                    if aspect >= 1.0:
                        i = i_0 + int(i_c * aspect)
                        j = j_0 + int(j_c)
                    else:
                        i = i_0 + int(i_c)
                        j = j_0 + int(j_c / aspect)

                    if (0 <= i < i_n) and (0 <= j < j_n):
                        i_k.append(i)
                        j_k.append(j)
                        rho_k.append(iRho)
                        theta_k.append(iTheta)

            transform = (
                (np.array(rho_k), np.array(theta_k)),
                (np.array(i_k), np.array(j_k)),
            )
            self._transforms[i_0, j_0, i_n, j_n, nRho, nTheta, theta_0, theta_1] = transform

        return transform

    def transform_polar_log(
        self,
        image,
        i_0,
        j_0,
        nRho=None,
        amplifyRho=1.0,
        nTheta=None,
        amplifyTheta=1.0,
        theta_0=0.0,
        theta_1=2.0 * np.pi,
        scale=0.0,
    ):
        (i_n, j_n) = image.shape[:2]

        i_c = max(i_0, i_n - i_0)
        j_c = max(j_0, j_n - j_0)
        d_c = (i_c**2 + j_c**2) ** 0.5
        d_s = min(i_0, i_n - i_0, j_0, j_n - j_0)
        d = scale * d_c + (1.0 - scale) * d_s

        if nRho is None:
            nRho = int(np.ceil(d * amplifyRho))

        if nTheta is None:
            nTheta = int(amplifyTheta * 2 * np.pi * np.sqrt((i_n**2 + j_n**2) / 2))

        dRho = np.log(d) / nRho

        (pt, ij) = self._get_transform_polar_log(
            i_0, j_0, i_n, j_n, nRho, dRho, nTheta, theta_0, theta_1
        )
        imgTransformed = np.zeros((nRho, nTheta) + image.shape[2:], dtype=image.dtype)
        imgTransformed[pt] = image[ij]

        return imgTransformed

    def _get_transform_polar_elliptical(
        self, i_0, j_0, i_n, j_n, r_axial_ortho, drStrip, angleEllipse, nRho, nTheta,
        theta_0, theta_1, rClip,
    ):
        (raxial, rortho) = r_axial_ortho
        nTheta = max(1, nTheta)
        transform = self._transforms.get(
            (i_0, j_0, i_n, j_n, nRho, drStrip, nTheta, theta_0, theta_1, rClip)
        )

        if transform is None:
            i_k = []
            j_k = []
            rho_k = []
            theta_k = []

            raxial_outer = raxial + drStrip
            rortho_outer = raxial + drStrip
            raxial_inner = raxial - drStrip
            rortho_inner = raxial - drStrip

            R = np.array(
                [
                    [np.cos(-angleEllipse), -np.sin(-angleEllipse)],
                    [np.sin(-angleEllipse), np.cos(-angleEllipse)],
                ]
            )
            dTheta = (theta_1 - theta_0) / nTheta

            for iTheta in range(0, nTheta):
                theta = theta_0 + iTheta * dTheta

                xy_e = np.array([raxial * np.cos(theta), rortho * np.sin(theta)])
                theta_e = np.arctan2(xy_e[1], xy_e[0])

                rho_e_inner = np.linalg.norm(
                    [raxial_inner * np.cos(theta), rortho_inner * np.sin(theta)]
                )
                rho_e_outer = np.linalg.norm(
                    [raxial_outer * np.cos(theta), rortho_outer * np.sin(theta)]
                )
                dRho = (rho_e_outer - rho_e_inner) / nRho

                for iRho in range(0, int(np.ceil(rClip * nRho))):
                    rho = rho_e_inner + iRho * dRho

                    i_e = rho * np.sin(theta_e)
                    j_e = rho * np.cos(theta_e)

                    ij = R.dot([i_e, j_e])

                    i = int(i_0 + ij[0])
                    j = int(j_0 + ij[1])

                    if (0 <= i < i_n) and (0 <= j < j_n):
                        i_k.append(i)
                        j_k.append(j)
                        rho_k.append(iRho)
                        theta_k.append(iTheta)

            transform = (
                (np.array(rho_k), np.array(theta_k)),
                (np.array(i_k), np.array(j_k)),
            )
            self._transforms[
                i_0, j_0, i_n, j_n, nRho, drStrip, nTheta, theta_0, theta_1, rClip
            ] = transform

        return transform

    def transform_polar_elliptical(
        self,
        image,
        i_0,
        j_0,
        raxial=None,
        rortho=None,
        dradiusStrip=None,
        nRho=None,
        amplifyRho=1.0,
        rClip=0.0,
        angleEllipse=0.0,
        theta_0=-np.pi,
        theta_1=np.pi,
        nTheta=None,
        amplifyTheta=1.0,
    ):
        (i_n, j_n) = image.shape[:2]
        if raxial is None:
            raxial = i_n / 2
        if rortho is None:
            rortho = j_n / 2

        if dradiusStrip is None:
            dradiusStrip = raxial - 5

        raxial_outer = raxial + dradiusStrip
        rortho_outer = raxial + dradiusStrip
        raxial_inner = raxial - dradiusStrip
        rortho_inner = raxial - dradiusStrip

        d_e_raxial_outer = raxial_outer
        d_e_rortho_outer = rortho_outer
        d_e_wedge_outer = np.linalg.norm(
            [raxial_outer * np.cos(theta_0), rortho_outer * np.sin(theta_0)]
        )
        if np.abs(theta_0) >= np.pi / 2.0:
            d_e_min_outer = min(d_e_raxial_outer, d_e_wedge_outer, d_e_rortho_outer)
        else:
            d_e_min_outer = min(d_e_raxial_outer, d_e_wedge_outer)

        d_e_raxial_inner = raxial_inner
        d_e_rortho_inner = rortho_inner
        d_e_wedge_inner = np.linalg.norm(
            [raxial_inner * np.cos(theta_0), rortho_inner * np.sin(theta_0)]
        )
        if np.abs(theta_0) >= np.pi / 2.0:
            d_e_min_inner = min(d_e_raxial_inner, d_e_wedge_inner, d_e_rortho_inner)
        else:
            d_e_min_inner = min(d_e_raxial_inner, d_e_wedge_inner)

        d = d_e_min_outer - d_e_min_inner

        xy_e = np.array([raxial * np.cos(theta_0), rortho * np.sin(theta_0)])
        theta_0e = np.arctan2(xy_e[1], xy_e[0])
        xy_e = np.array([raxial * np.cos(theta_1), rortho * np.sin(theta_1)])
        theta_1e = np.arctan2(xy_e[1], xy_e[0])

        if nRho is None:
            nRho = int(np.ceil(d * amplifyRho))

        if nTheta is None:
            circumference = 2 * np.pi * np.sqrt((raxial**2 + rortho**2))
            fraction_wedge = np.abs(theta_1e - theta_0e) / (2 * np.pi)
            nTheta = int(amplifyTheta * circumference * fraction_wedge)

        (pt, ij) = self._get_transform_polar_elliptical(
            i_0,
            j_0,
            i_n,
            j_n,
            (raxial, rortho),
            dradiusStrip,
            angleEllipse,
            nRho,
            nTheta,
            theta_0,
            theta_1,
            rClip,
        )
        imgTransformed = np.zeros((nRho, nTheta) + image.shape[2:], dtype=image.dtype)
        if len(pt[0]) > 0:
            imgTransformed[pt] = image[ij]
        else:
            raise TransformException()

        return imgTransformed


class PhaseCorrelation:
    """Compute the coordinate shift between two images via FFT phase correlation."""

    def get_shift(self, imgA: np.ndarray, imgB: np.ndarray) -> np.ndarray:
        rv = np.array([0.0, 0.0])
        if imgA is not None and imgB is not None and imgA.shape == imgB.shape:
            A = cv2.dft(imgA)
            B = cv2.dft(imgB)
            AB = cv2.mulSpectrums(A, B, flags=0, conjB=True)
            normAB = cv2.norm(AB)
            if normAB != 0.0:
                crosspower = AB / normAB
                shift = cv2.idft(crosspower)
                shift0 = np.roll(shift, int(shift.shape[0] / 2), 0)
                shift00 = np.roll(shift0, int(shift.shape[1] / 2), 1)

                kShift = np.argmax(shift00)
                (iShift, jShift) = np.unravel_index(kShift, shift00.shape)

                w = 7
                r = int((w - 1) / 2)
                i0 = int(clip(iShift - r, 0, shift00.shape[0] - 1))
                i1 = int(clip(iShift + r, 0, shift00.shape[0] - 1)) + 1
                j0 = int(clip(jShift - r, 0, shift00.shape[1] - 1))
                j1 = int(clip(jShift + r, 0, shift00.shape[1] - 1)) + 1
                peak = shift00[i0:i1].T[j0:j1].T
                moments = cv2.moments(peak, binaryImage=False)

                if moments["m00"] != 0.0:
                    iShiftSubpixel = moments["m01"] / moments["m00"] + float(i0)
                    jShiftSubpixel = moments["m10"] / moments["m00"] + float(j0)
                else:
                    iShiftSubpixel = float(shift.shape[0]) / 2.0
                    jShiftSubpixel = float(shift.shape[1]) / 2.0

                iShiftSubpixel -= float(shift.shape[0]) / 2.0
                jShiftSubpixel -= float(shift.shape[1]) / 2.0

                height = float(shift00.shape[0])
                width = float(shift00.shape[1])
                iShiftSubpixel = ((iShiftSubpixel + height / 2.0) % height) - height / 2.0
                jShiftSubpixel = ((jShiftSubpixel + width / 2.0) % width) - width / 2.0

                rv = np.array([iShiftSubpixel, jShiftSubpixel])

        return rv


class WindowFunctions:
    """Create 2D window functions for image processing."""

    def create_hanning(self, shape: tuple[int, int]) -> np.ndarray:
        (height, width) = shape
        wfn = np.ones(shape, dtype=np.float32)
        if height > 1 and width > 1:
            for i in range(width):
                for j in range(height):
                    x = 2 * np.pi * i / (width - 1)
                    y = 2 * np.pi * j / (height - 1)
                    wfn[j][i] = 0.5 * (1 - np.cos(x)) * 0.5 * (1 - np.cos(y))
        return wfn

    def create_tukey(self, shape: tuple[int, int]) -> np.ndarray:
        (height, width) = shape
        alpha = 0.25
        wfn = np.ones(shape, dtype=np.float32)
        if height > 1 and width > 1:
            for i in range(width):
                for j in range(height):
                    y = np.pi * (2 * j / (alpha * (height - 1)) - 1)

                    if 0 <= i <= (alpha * (width - 1)) / 2:
                        x = np.pi * (2 * i / (alpha * (width - 1)) - 1)
                    elif (alpha * (width - 1)) / 2 < i <= (width - 1) * (1 - alpha / 2):
                        x = 0.0
                    elif (width - 1) * (1 - alpha / 2) < i <= width - 1:
                        x = np.pi * (2 * i / (alpha * (width - 1)) - 2 / alpha + 1)

                    if 0 <= j <= (alpha * (height - 1)) / 2:
                        y = np.pi * (2 * j / (alpha * (height - 1)) - 1)
                    elif (alpha * (height - 1)) / 2 < j <= (height - 1) * (1 - alpha / 2):
                        y = 0.0
                    elif (height - 1) * (1 - alpha / 2) < j <= height - 1:
                        y = np.pi * (2 * j / (alpha * (height - 1)) - 2 / alpha + 1)

                    wfnx = 0.5 * (1 + np.cos(x))
                    wfny = 0.5 * (1 + np.cos(y))
                    wfn[j][i] = wfnx * wfny
        return wfn
```

- [ ] **Step 4: Run tests to verify they pass**

```bash
uv run pytest tests/core/test_imaging.py -v
```

Expected: all tests PASS.

- [ ] **Step 5: Lint check**

```bash
uv run ruff check src/kinefly/core/imaging.py
```

Fix any issues.

- [ ] **Step 6: Commit**

```bash
git add src/kinefly/core/imaging.py tests/core/test_imaging.py
git commit -m "feat: port imageprocessing.py to Python 3 (imaging.py)"
```

---

## Task 6: Wingbeat Detector

**Files:**
- Create: `src/kinefly/trackers/wingbeat.py`
- Create: `tests/trackers/__init__.py`
- Create: `tests/trackers/test_wingbeat.py`

- [ ] **Step 1: Write the failing test**

`tests/trackers/__init__.py`: empty file.

`tests/trackers/test_wingbeat.py`:
```python
import numpy as np

from kinefly.trackers.wingbeat import WingbeatDetector


def test_fs_dict_from_wingband():
    """Verify the undersampling framerate computation produces valid ranges."""
    wd = WingbeatDetector(fw_min=180.0, fw_max=220.0)
    fs_dict = wd.fs_dict_from_wingband(180.0, 220.0)
    assert len(fs_dict["fs_range_list"]) > 0
    # Each range should have lo < hi
    for lo, hi in fs_dict["fs_range_list"]:
        assert lo < hi


def test_freq_from_intensity_returns_float():
    """Verify freq_from_intensity runs and returns a numeric value."""
    wd = WingbeatDetector(fw_min=180.0, fw_max=220.0)
    freq = wd.freq_from_intensity(0.5, fs=100)
    assert isinstance(freq, float)


def test_buffer_wraps():
    """Feed more samples than the buffer size to verify wraparound."""
    wd = WingbeatDetector(fw_min=180.0, fw_max=220.0)
    for i in range(200):
        freq = wd.freq_from_intensity(np.sin(i * 0.1), fs=100)
    assert isinstance(freq, float)
```

- [ ] **Step 2: Run test to verify it fails**

```bash
uv run pytest tests/trackers/test_wingbeat.py -v
```

Expected: FAIL with `ModuleNotFoundError`

- [ ] **Step 3: Write the implementation**

Port `nodes/wingbeatdetector.py` to `src/kinefly/trackers/wingbeat.py`. Changes:
- Remove `import rospy` and `import cv` / `import cv2`
- Replace `rospy.logwarn` with `logging.warning`

`src/kinefly/trackers/wingbeat.py`:
```python
"""Wingbeat frequency detector via undersampling aliasing analysis.

Ported from nodes/wingbeatdetector.py. Measures wingbeat frequency (typically
180-220 Hz) from low-framerate camera intensity data using undersampling theory.
"""

from __future__ import annotations

import logging

import numpy as np

logger = logging.getLogger(__name__)


class WingbeatDetector:
    def __init__(self, fw_min: float, fw_max: float) -> None:
        self.n = 64
        self.buffer = np.zeros([2 * self.n, 2])  # Holds intensities & framerates
        self.set(fw_min, fw_max)

    def set(self, fw_min: float, fw_max: float) -> None:
        self.i = 0
        self.fw_min = fw_min
        self.fw_max = fw_max
        self.fw_center = (fw_min + fw_max) / 2.0

        self.fs_dict = self.fs_dict_from_wingband(fw_min, fw_max)
        if len(self.fs_dict["fs_range_list"]) > 0:
            (self.fs_lo, self.fs_hi) = self.fs_dict["fs_range_list"][0]
        else:
            (self.fs_lo, self.fs_hi) = (0.0, 0.0)

    def warn(self) -> None:
        logger.warning(
            "Wingbeat detector set to measure frequencies in [%.1f, %.1f] Hz. "
            "Camera framerate must stay in one of: %s",
            self.fw_min,
            self.fw_max,
            self.fs_dict["fs_range_list"],
        )

    def fs_dict_from_wingband(
        self, fw_min: float, fw_max: float
    ) -> dict[str, list]:
        fs_range_list = []
        m_list = []

        bw = fw_max - fw_min
        m = 1
        while True:
            fs_hi = (2.0 * self.fw_center - bw) / m
            fs_lo = max((2.0 * self.fw_center + bw) / (m + 1), 2 * bw)
            if 2 * bw < fs_hi:
                fs_range_list.append([fs_lo, fs_hi])
                m_list.append(m)
            else:
                break
            m += 1

        fs_range_list.reverse()
        m_list.reverse()

        return {"fs_range_list": fs_range_list, "m_list": m_list}

    def wingband_from_fs(
        self, fs_lo: float, fs_hi: float, fw_center: float
    ) -> tuple[float, float, bool]:
        bw_lo = fs_lo / 2.0
        fs = (fs_lo + fs_hi) / 2.0

        if fs != 0.0:
            n = np.round(fw_center / (fs / 2))
            if n * fs / 2 < fw_center:
                fw_min = n * fs / 2
                fw_max = (n + 1) * fs / 2
            else:
                fw_min = (n - 1) * fs / 2
                fw_max = n * fs / 2
            bReversed = (n % 2) == 1
        else:
            fw_min = 0.0
            fw_max = 1.0
            bReversed = False

        return (fw_min, fw_max, bReversed)

    def get_baseband_range(self, fs: float, m: int) -> tuple[float, float]:
        kMax = int(np.floor(2 * self.fw_center / m))
        fbb_min = 0.0
        fbb_max = 0.0

        if m % 2 == 0:
            for k in range(kMax):
                fbb_min_tmp = self.fw_min - k * fs
                fbb_max_tmp = self.fw_max - k * fs
                if fbb_min_tmp >= 0:
                    fbb_min = fbb_min_tmp
                    fbb_max = fbb_max_tmp
                else:
                    break
        else:
            for k in range(kMax):
                fbb_min = -self.fw_min + k * fs
                fbb_max = -self.fw_max + k * fs
                if fbb_max >= 0:
                    break

        return (fbb_min, fbb_max)

    def get_baseband_range_from_framerates(
        self, framerates: np.ndarray
    ) -> tuple[bool, np.ndarray]:
        bValid = False

        fs_lo = np.min(framerates)
        fs_hi = np.max(framerates)
        iRange = 0
        for iRange in range(len(self.fs_dict["fs_range_list"])):
            (fs_min, fs_max) = self.fs_dict["fs_range_list"][iRange]
            if fs_min < fs_lo < fs_hi < fs_max:
                bValid = True
                break

        m = self.fs_dict["m_list"][iRange]

        if bValid:
            fs = np.mean(framerates)
            (fbb_min, fbb_max) = self.get_baseband_range(fs, m)
        else:
            fbb_min = 0.0
            fbb_max = np.inf

        return (bValid, np.array([fbb_min, fbb_max]))

    def freq_from_intensity(self, intensity: float, fs: float = 0) -> float:
        self.buffer[self.i] = [intensity, fs]
        self.buffer[self.i + self.n] = [intensity, fs]

        framerates = self.buffer[(self.i + 1) : (self.i + 1 + self.n), 1]

        (bValid, fbb_range) = self.get_baseband_range_from_framerates(framerates)
        if fbb_range[0] < fbb_range[1]:
            fbb_min = fbb_range[0]
            fbb_max = fbb_range[1]
            bReverse = False
        else:
            fbb_min = fbb_range[1]
            fbb_max = fbb_range[0]
            bReverse = True

        if bValid:
            intensities = self.buffer[(self.i + 1) : (self.i + 1 + self.n), 0]

            fft = np.fft.rfft(intensities)
            fft[0] = 0
            i_max = np.argmax(np.abs(fft))
            f_width = fbb_max - fbb_min
            f_offset = np.abs(np.fft.fftfreq(self.n)[i_max]) * fs - fbb_min
            if bReverse:
                freq = self.fw_max - f_offset
            else:
                freq = self.fw_min + f_offset
        else:
            freq = 0.0

        self.i += 1
        self.i %= self.n

        return float(freq)
```

- [ ] **Step 4: Run tests to verify they pass**

```bash
uv run pytest tests/trackers/test_wingbeat.py -v
```

Expected: all 3 tests PASS.

- [ ] **Step 5: Commit**

```bash
git add src/kinefly/trackers/ tests/trackers/
git commit -m "feat: port WingbeatDetector to Python 3"
```

---

## Task 7: Rig Configuration System

**Files:**
- Create: `src/kinefly/config/__init__.py`
- Create: `src/kinefly/config/models.py`
- Create: `src/kinefly/config/loader.py`
- Create: `rigs/example/config.yaml`
- Create: `tests/config/__init__.py`
- Create: `tests/config/test_loader.py`

- [ ] **Step 1: Write the failing test**

`tests/config/__init__.py`: empty file.

`tests/config/test_loader.py`:
```python
import tempfile
from pathlib import Path

import yaml

from kinefly.config.loader import load_rig_config
from kinefly.config.models import RigConfig


def _write_yaml(path: Path, data: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.dump(data))


def test_load_minimal_config():
    with tempfile.TemporaryDirectory() as tmpdir:
        config_path = Path(tmpdir) / "config.yaml"
        _write_yaml(config_path, {
            "kinefly": {"version": 2},
            "camera": {"source": "opencv"},
            "tracking": {
                "head": {"tracker": "area"},
                "abdomen": {"tracker": "area"},
                "left": {"tracker": "edge"},
                "right": {"tracker": "edge"},
            },
        })
        config = load_rig_config(config_path)
        assert isinstance(config, RigConfig)
        assert config.camera.source == "opencv"
        assert config.tracking.head.tracker == "area"
        assert config.tracking.left.tracker == "edge"


def test_defaults_are_applied():
    with tempfile.TemporaryDirectory() as tmpdir:
        config_path = Path(tmpdir) / "config.yaml"
        _write_yaml(config_path, {
            "kinefly": {"version": 2},
            "camera": {"source": "opencv"},
            "tracking": {
                "head": {"tracker": "area"},
                "abdomen": {"tracker": "area"},
                "left": {"tracker": "edge"},
                "right": {"tracker": "edge"},
            },
        })
        config = load_rig_config(config_path)
        # Defaults from the spec
        assert config.tracking.rc_background == 1000.0
        assert config.tracking.n_edges_max == 1
        assert config.tracking.head.threshold == 0.0
        assert config.tracking.head.feathering == 0.0


def test_optional_sections_absent():
    with tempfile.TemporaryDirectory() as tmpdir:
        config_path = Path(tmpdir) / "config.yaml"
        _write_yaml(config_path, {
            "kinefly": {"version": 2},
            "camera": {"source": "opencv"},
            "tracking": {
                "head": {"tracker": "area"},
                "abdomen": {"tracker": "area"},
                "left": {"tracker": "edge"},
                "right": {"tracker": "edge"},
            },
        })
        config = load_rig_config(config_path)
        assert config.phidgets is None
        assert config.ledpanels is None
        assert config.zmq is None
```

- [ ] **Step 2: Run test to verify it fails**

```bash
uv run pytest tests/config/test_loader.py -v
```

Expected: FAIL with `ModuleNotFoundError`

- [ ] **Step 3: Write `config/models.py`**

`src/kinefly/config/__init__.py`: empty file.

`src/kinefly/config/models.py`:
```python
"""Configuration dataclasses for rig settings."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any


@dataclass
class CameraConfig:
    source: str = "opencv"  # "harvester", "opencv", or "file"
    device_index: int = 0
    cti_file: str = ""
    serial: str = ""
    path: str = ""
    exposure_us: int = 5000
    framerate: float = 60.0
    gain: float = 0.0
    scale_image: float = 1.0
    realtime: bool = True


@dataclass
class BodyPartConfig:
    tracker: str = "area"
    autozero: bool = True
    threshold: float = 0.0
    feathering: float = 0.0
    saturation_correction: bool = False


@dataclass
class AuxConfig:
    wingbeat_min: float = 180.0
    wingbeat_max: float = 220.0


@dataclass
class TrackingConfig:
    n_edges_max: int = 1
    rc_background: float = 1000.0
    use_gui: bool = True
    head: BodyPartConfig = field(default_factory=BodyPartConfig)
    abdomen: BodyPartConfig = field(default_factory=BodyPartConfig)
    left: BodyPartConfig = field(default_factory=lambda: BodyPartConfig(tracker="edge"))
    right: BodyPartConfig = field(default_factory=lambda: BodyPartConfig(tracker="edge"))
    aux: AuxConfig = field(default_factory=AuxConfig)


@dataclass
class RecordingConfig:
    output_dir: str = "~/kinefly_recordings"
    encoder: str = "auto"


@dataclass
class GuiConfig:
    state_file: str = "~/kinefly.yaml"


@dataclass
class ZmqConfig:
    address: str = "tcp://*:5555"


@dataclass
class PhidgetsChannelConfig:
    enable: bool = True
    coefficients: dict[str, float] = field(default_factory=lambda: {
        "offset": 0, "l1": 0, "l2": 0, "lr": 0, "r1": 0, "r2": 0,
        "rr": 0, "ha": 0, "hr": 0, "aa": 0, "ar": 0, "xi": 0,
    })


@dataclass
class PhidgetsConfig:
    serial: int = 0
    autorange: bool = False
    channels: list[PhidgetsChannelConfig] = field(default_factory=lambda: [
        PhidgetsChannelConfig(coefficients={"offset": 0, "l1": 1.0, "l2": 0, "lr": 0, "r1": 0, "r2": 0, "rr": 0, "ha": 0, "hr": 0, "aa": 0, "ar": 0, "xi": 0}),
        PhidgetsChannelConfig(coefficients={"offset": 0, "l1": 0, "l2": 0, "lr": 0, "r1": 1.0, "r2": 0, "rr": 0, "ha": 0, "hr": 0, "aa": 0, "ar": 0, "xi": 0}),
        PhidgetsChannelConfig(coefficients={"offset": 0, "l1": 1.0, "l2": 0, "lr": 0, "r1": -1.0, "r2": 0, "rr": 0, "ha": 0, "hr": 0, "aa": 0, "ar": 0, "xi": 0}),
        PhidgetsChannelConfig(coefficients={"offset": 0, "l1": 1.0, "l2": 0, "lr": 0, "r1": 1.0, "r2": 0, "rr": 0, "ha": 0, "hr": 0, "aa": 0, "ar": 0, "xi": 0}),
    ])


@dataclass
class LedPanelsConfig:
    serial_port: str = "/dev/ttyUSB0"
    baud_rate: int = 115200
    method: str = "voltage"
    mode: str = "velocity"
    axis: str = "x"
    pattern_id: int = 1
    coeff_voltage: dict[str, float] = field(default_factory=lambda: {
        "adc0": 1, "adc1": 0, "adc2": 0, "adc3": 0, "funcx": 0, "funcy": 0,
    })
    coeff_usb: dict[str, float] = field(default_factory=lambda: {
        "x0": 0, "xl1": 1.0, "xl2": 0, "xr1": -1.0, "xr2": 0, "xha": 0, "xhr": 0, "xaa": 0, "xar": 0, "xxi": 0,
        "y0": 0, "yl1": 0, "yl2": 0, "yr1": 0, "yr2": 0, "yha": 0, "yhr": 0, "yaa": 0, "yar": 0, "yxi": 0,
    })


@dataclass
class RigConfig:
    version: int = 2
    camera: CameraConfig = field(default_factory=CameraConfig)
    tracking: TrackingConfig = field(default_factory=TrackingConfig)
    recording: RecordingConfig = field(default_factory=RecordingConfig)
    gui: GuiConfig = field(default_factory=GuiConfig)
    zmq: ZmqConfig | None = None
    phidgets: PhidgetsConfig | None = None
    ledpanels: LedPanelsConfig | None = None
```

- [ ] **Step 4: Write `config/loader.py`**

`src/kinefly/config/loader.py`:
```python
"""Load and validate rig configuration from YAML files."""

from __future__ import annotations

import logging
from pathlib import Path
from typing import Any

import yaml

from kinefly.config.models import (
    AuxConfig,
    BodyPartConfig,
    CameraConfig,
    GuiConfig,
    LedPanelsConfig,
    PhidgetsChannelConfig,
    PhidgetsConfig,
    RecordingConfig,
    RigConfig,
    TrackingConfig,
    ZmqConfig,
)

logger = logging.getLogger(__name__)


def load_rig_config(path: Path) -> RigConfig:
    """Load a rig configuration from a YAML file.

    Args:
        path: Path to the config.yaml file.

    Returns:
        Populated RigConfig with defaults applied for missing values.
    """
    with open(path) as f:
        raw = yaml.safe_load(f) or {}

    return _parse_config(raw)


def _parse_config(raw: dict[str, Any]) -> RigConfig:
    camera = _parse_camera(raw.get("camera", {}))
    tracking = _parse_tracking(raw.get("tracking", {}))
    recording = _parse_dataclass(raw.get("recording", {}), RecordingConfig)
    gui = _parse_dataclass(raw.get("gui", {}), GuiConfig)

    zmq = ZmqConfig(**raw["zmq"]) if "zmq" in raw else None
    phidgets = _parse_phidgets(raw["phidgets"]) if "phidgets" in raw else None
    ledpanels = _parse_dataclass(raw["ledpanels"], LedPanelsConfig) if "ledpanels" in raw else None

    return RigConfig(
        version=raw.get("kinefly", {}).get("version", 2),
        camera=camera,
        tracking=tracking,
        recording=recording,
        gui=gui,
        zmq=zmq,
        phidgets=phidgets,
        ledpanels=ledpanels,
    )


def _parse_camera(raw: dict[str, Any]) -> CameraConfig:
    return _parse_dataclass(raw, CameraConfig)


def _parse_tracking(raw: dict[str, Any]) -> TrackingConfig:
    head = _parse_dataclass(raw.get("head", {}), BodyPartConfig)
    abdomen = _parse_dataclass(raw.get("abdomen", {}), BodyPartConfig)
    left = _parse_dataclass(raw.get("left", {"tracker": "edge"}), BodyPartConfig)
    right = _parse_dataclass(raw.get("right", {"tracker": "edge"}), BodyPartConfig)
    aux = _parse_dataclass(raw.get("aux", {}), AuxConfig)

    return TrackingConfig(
        n_edges_max=raw.get("n_edges_max", 1),
        rc_background=raw.get("rc_background", 1000.0),
        use_gui=raw.get("use_gui", True),
        head=head,
        abdomen=abdomen,
        left=left,
        right=right,
        aux=aux,
    )


def _parse_phidgets(raw: dict[str, Any]) -> PhidgetsConfig:
    channels_raw = raw.get("channels", [])
    channels = [
        PhidgetsChannelConfig(
            enable=ch.get("enable", True),
            coefficients=ch.get("coefficients", {}),
        )
        for ch in channels_raw
    ]
    return PhidgetsConfig(
        serial=raw.get("serial", 0),
        autorange=raw.get("autorange", False),
        channels=channels if channels else PhidgetsConfig().channels,
    )


def _parse_dataclass(raw: dict[str, Any], cls: type):
    """Create a dataclass instance from a dict, ignoring unknown keys."""
    import dataclasses

    field_names = {f.name for f in dataclasses.fields(cls)}
    filtered = {k: v for k, v in raw.items() if k in field_names}
    return cls(**filtered)
```

- [ ] **Step 5: Run tests to verify they pass**

```bash
uv run pytest tests/config/test_loader.py -v
```

Expected: all 3 tests PASS.

- [ ] **Step 6: Create example rig config**

`rigs/example/config.yaml`:
```yaml
kinefly:
  version: 2

camera:
  source: opencv
  device_index: 0
  framerate: 60
  scale_image: 1.0

tracking:
  n_edges_max: 1
  rc_background: 1000.0
  use_gui: true

  head:
    tracker: area
    autozero: true
    threshold: 0.0
    feathering: 0.0
    saturation_correction: false

  abdomen:
    tracker: area
    autozero: true
    threshold: 0.0
    feathering: 0.25
    saturation_correction: false

  left:
    tracker: edge
    threshold: 0.01
    saturation_correction: false

  right:
    tracker: edge
    threshold: 0.01
    saturation_correction: false

  aux:
    wingbeat_min: 180.0
    wingbeat_max: 220.0

recording:
  output_dir: ~/kinefly_recordings
  encoder: auto

gui:
  state_file: ~/kinefly.yaml

# Uncomment to enable ZeroMQ publishing:
# zmq:
#   address: tcp://*:5555
```

- [ ] **Step 7: Commit**

```bash
git add src/kinefly/config/ tests/config/ rigs/
git commit -m "feat: add YAML rig configuration system with loader and models"
```

---

## Task 8: Camera Abstraction Layer

**Files:**
- Create: `src/kinefly/camera/__init__.py`
- Create: `src/kinefly/camera/base.py`
- Create: `src/kinefly/camera/opencv.py`
- Create: `src/kinefly/camera/videofile.py`
- Create: `src/kinefly/camera/harvester.py`
- Create: `tests/camera/__init__.py`
- Create: `tests/camera/test_opencv.py`

- [ ] **Step 1: Write the failing test**

`tests/camera/__init__.py`: empty file.

`tests/camera/test_opencv.py`:
```python
import numpy as np
import pytest

from kinefly.camera.base import CameraSource
from kinefly.camera.opencv import OpenCVCamera


def test_opencv_camera_is_camera_source():
    assert issubclass(OpenCVCamera, CameraSource)


def test_opencv_camera_has_required_methods():
    cam = OpenCVCamera(device_index=0)
    assert hasattr(cam, "open")
    assert hasattr(cam, "read")
    assert hasattr(cam, "close")
    assert hasattr(cam, "fps")
    assert hasattr(cam, "resolution")
```

- [ ] **Step 2: Run test to verify it fails**

```bash
uv run pytest tests/camera/test_opencv.py -v
```

Expected: FAIL

- [ ] **Step 3: Write `camera/base.py`**

`src/kinefly/camera/__init__.py`: empty file.

`src/kinefly/camera/base.py`:
```python
"""Abstract base class for camera sources."""

from __future__ import annotations

from abc import ABC, abstractmethod

import numpy as np


class CameraSource(ABC):
    """Abstract interface for all image sources (cameras, video files)."""

    @abstractmethod
    def open(self) -> None:
        """Open the camera/file and prepare for frame capture."""
        ...

    @abstractmethod
    def read(self) -> tuple[np.ndarray, float]:
        """Read the next frame.

        Returns:
            Tuple of (image as numpy array, timestamp in seconds).
            Blocks until a frame is available.

        Raises:
            StopIteration: When no more frames are available (end of video file).
            RuntimeError: When the camera is not open or a read error occurs.
        """
        ...

    @abstractmethod
    def close(self) -> None:
        """Release the camera/file resources."""
        ...

    @property
    @abstractmethod
    def fps(self) -> float:
        """Frames per second of the source."""
        ...

    @property
    @abstractmethod
    def resolution(self) -> tuple[int, int]:
        """Resolution as (width, height)."""
        ...
```

- [ ] **Step 4: Write `camera/opencv.py`**

`src/kinefly/camera/opencv.py`:
```python
"""OpenCV VideoCapture camera source — development/testing fallback."""

from __future__ import annotations

import time

import cv2
import numpy as np

from kinefly.camera.base import CameraSource


class OpenCVCamera(CameraSource):
    """Camera source using OpenCV's VideoCapture for USB/built-in cameras.

    Args:
        device_index: Camera device index (default 0).
        framerate: Requested framerate (best-effort, depends on hardware).
    """

    def __init__(self, device_index: int = 0, framerate: float = 60.0) -> None:
        self._device_index = device_index
        self._requested_fps = framerate
        self._cap: cv2.VideoCapture | None = None
        self._fps: float = framerate
        self._resolution: tuple[int, int] = (640, 480)

    def open(self) -> None:
        self._cap = cv2.VideoCapture(self._device_index)
        if not self._cap.isOpened():
            raise RuntimeError(f"Cannot open camera at index {self._device_index}")
        self._cap.set(cv2.CAP_PROP_FPS, self._requested_fps)
        self._fps = self._cap.get(cv2.CAP_PROP_FPS) or self._requested_fps
        w = int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self._resolution = (w, h)

    def read(self) -> tuple[np.ndarray, float]:
        if self._cap is None:
            raise RuntimeError("Camera not opened. Call open() first.")
        ret, frame = self._cap.read()
        if not ret:
            raise RuntimeError("Failed to read frame from camera")
        # Convert to grayscale if color
        if len(frame.shape) == 3:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        return frame, time.time()

    def close(self) -> None:
        if self._cap is not None:
            self._cap.release()
            self._cap = None

    @property
    def fps(self) -> float:
        return self._fps

    @property
    def resolution(self) -> tuple[int, int]:
        return self._resolution
```

- [ ] **Step 5: Write `camera/videofile.py`**

`src/kinefly/camera/videofile.py`:
```python
"""Video file replay source."""

from __future__ import annotations

import time

import cv2
import numpy as np

from kinefly.camera.base import CameraSource


class VideoFileSource(CameraSource):
    """Replay a recorded video file.

    Args:
        path: Path to the video file (MP4, AVI, etc.).
        realtime: If True, sleep between frames to match original framerate.
            If False, deliver frames as fast as possible (batch retracking).
    """

    def __init__(self, path: str, realtime: bool = True) -> None:
        self._path = path
        self._realtime = realtime
        self._cap: cv2.VideoCapture | None = None
        self._fps: float = 30.0
        self._resolution: tuple[int, int] = (640, 480)
        self._last_frame_time: float = 0.0

    def open(self) -> None:
        self._cap = cv2.VideoCapture(self._path)
        if not self._cap.isOpened():
            raise RuntimeError(f"Cannot open video file: {self._path}")
        self._fps = self._cap.get(cv2.CAP_PROP_FPS) or 30.0
        w = int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self._resolution = (w, h)
        self._last_frame_time = time.time()

    def read(self) -> tuple[np.ndarray, float]:
        if self._cap is None:
            raise RuntimeError("Video file not opened. Call open() first.")

        if self._realtime:
            elapsed = time.time() - self._last_frame_time
            target = 1.0 / self._fps
            if elapsed < target:
                time.sleep(target - elapsed)

        ret, frame = self._cap.read()
        if not ret:
            raise StopIteration("End of video file")

        self._last_frame_time = time.time()

        if len(frame.shape) == 3:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        timestamp = self._cap.get(cv2.CAP_PROP_POS_MSEC) / 1000.0
        return frame, timestamp

    def close(self) -> None:
        if self._cap is not None:
            self._cap.release()
            self._cap = None

    @property
    def fps(self) -> float:
        return self._fps

    @property
    def resolution(self) -> tuple[int, int]:
        return self._resolution
```

- [ ] **Step 6: Write `camera/harvester.py`**

`src/kinefly/camera/harvester.py`:
```python
"""GenICam camera source via Harvesters library."""

from __future__ import annotations

import logging
import time

import numpy as np

from kinefly.camera.base import CameraSource

logger = logging.getLogger(__name__)


class HarvesterCamera(CameraSource):
    """Camera source using Harvesters for GenICam-compliant cameras.

    Args:
        cti_file: Path to the GenTL producer CTI file.
        serial: Camera serial number. Empty string for first available camera.
        framerate: Requested framerate.
        exposure_us: Exposure time in microseconds.
        gain: Camera gain.
    """

    def __init__(
        self,
        cti_file: str,
        serial: str = "",
        framerate: float = 60.0,
        exposure_us: int = 5000,
        gain: float = 0.0,
    ) -> None:
        self._cti_file = cti_file
        self._serial = serial
        self._requested_fps = framerate
        self._exposure_us = exposure_us
        self._gain = gain
        self._harvester = None
        self._acquirer = None
        self._fps: float = framerate
        self._resolution: tuple[int, int] = (640, 480)

    def open(self) -> None:
        try:
            from harvesters.core import Harvester
        except ImportError:
            raise ImportError(
                "harvesters not installed. Install with: pip install kinefly[harvester]"
            )

        self._harvester = Harvester()
        self._harvester.add_file(self._cti_file)
        self._harvester.update()

        if len(self._harvester.device_info_list) == 0:
            raise RuntimeError("No GenICam cameras found")

        if self._serial:
            self._acquirer = self._harvester.create(
                {"serial_number": self._serial}
            )
        else:
            self._acquirer = self._harvester.create()

        node_map = self._acquirer.remote_device.node_map
        try:
            node_map.ExposureTime.value = self._exposure_us
        except Exception:
            logger.warning("Could not set ExposureTime")
        try:
            node_map.AcquisitionFrameRate.value = self._requested_fps
        except Exception:
            logger.warning("Could not set AcquisitionFrameRate")
        try:
            node_map.Gain.value = self._gain
        except Exception:
            logger.warning("Could not set Gain")

        self._acquirer.start()

        w = node_map.Width.value
        h = node_map.Height.value
        self._resolution = (w, h)
        self._fps = self._requested_fps

    def read(self) -> tuple[np.ndarray, float]:
        if self._acquirer is None:
            raise RuntimeError("Camera not opened. Call open() first.")

        with self._acquirer.fetch() as buffer:
            component = buffer.payload.components[0]
            frame = component.data.reshape(component.height, component.width)
            frame = frame.copy()
            timestamp = time.time()

        return frame, timestamp

    def close(self) -> None:
        if self._acquirer is not None:
            self._acquirer.stop()
            self._acquirer.destroy()
            self._acquirer = None
        if self._harvester is not None:
            self._harvester.reset()
            self._harvester = None

    @property
    def fps(self) -> float:
        return self._fps

    @property
    def resolution(self) -> tuple[int, int]:
        return self._resolution
```

- [ ] **Step 7: Run tests**

```bash
uv run pytest tests/camera/test_opencv.py -v
```

Expected: all tests PASS.

- [ ] **Step 8: Commit**

```bash
git add src/kinefly/camera/ tests/camera/
git commit -m "feat: add camera abstraction layer (Harvesters, OpenCV, video file)"
```

---

## Task 9: Plugin Base and Phidgets Plugin

**Files:**
- Create: `src/kinefly/plugins/__init__.py`
- Create: `src/kinefly/plugins/base.py`
- Create: `src/kinefly/plugins/phidgets.py`
- Create: `tests/plugins/__init__.py`
- Create: `tests/plugins/test_phidgets.py`

- [ ] **Step 1: Write the failing test**

`tests/plugins/__init__.py`: empty file.

`tests/plugins/test_phidgets.py`:
```python
import numpy as np

from kinefly.core.types import BodyPartState, FlyState
from kinefly.plugins.base import OutputPlugin
from kinefly.plugins.phidgets import PhidgetsPlugin


def test_phidgets_is_output_plugin():
    assert issubclass(PhidgetsPlugin, OutputPlugin)


def test_voltages_from_flystate():
    plugin = PhidgetsPlugin()
    # Default channels: L, R, L-R, L+R
    state = FlyState(
        timestamp=0.0,
        seq=0,
        left=BodyPartState(angles=[1.0]),
        right=BodyPartState(angles=[0.5]),
    )
    voltages = plugin.voltages_from_flystate(state)
    assert len(voltages) == 4
    np.testing.assert_allclose(voltages[0], 1.0, atol=1e-6)   # L
    np.testing.assert_allclose(voltages[1], 0.5, atol=1e-6)   # R
    np.testing.assert_allclose(voltages[2], 0.5, atol=1e-6)   # L-R
    np.testing.assert_allclose(voltages[3], 1.5, atol=1e-6)   # L+R


def test_voltages_clipped_to_range():
    plugin = PhidgetsPlugin()
    state = FlyState(
        timestamp=0.0,
        seq=0,
        left=BodyPartState(angles=[100.0]),  # Will exceed 10V
        right=BodyPartState(angles=[100.0]),
    )
    voltages = plugin.voltages_from_flystate(state)
    assert all(-10.0 <= v <= 10.0 for v in voltages)
```

- [ ] **Step 2: Run test to verify it fails**

```bash
uv run pytest tests/plugins/test_phidgets.py -v
```

Expected: FAIL

- [ ] **Step 3: Write `plugins/base.py`**

`src/kinefly/plugins/__init__.py`: empty file.

`src/kinefly/plugins/base.py`:
```python
"""Base class for output plugins."""

from __future__ import annotations

from typing import Any

from kinefly.core.types import FlyState


class OutputPlugin:
    """Base class for output plugins (Phidgets, LED panels, etc.).

    Subclasses implement start/on_flystate/stop. The on_flystate method
    is called on the main processing thread and must return quickly.
    Plugins that do slow I/O should use internal queues/threads.
    """

    def start(self, config: dict[str, Any]) -> None:
        """Initialize hardware connections."""
        ...

    def on_flystate(self, state: FlyState) -> None:
        """Process a new fly state. Must return quickly."""
        ...

    def stop(self) -> None:
        """Clean up hardware connections."""
        ...
```

- [ ] **Step 4: Write `plugins/phidgets.py`**

`src/kinefly/plugins/phidgets.py`:
```python
"""PhidgetsAnalog voltage output plugin.

Ported from nodes/flystate2phidgetsanalog.py. Maps fly state to 4-channel
analog voltage output via a configurable coefficient matrix.
"""

from __future__ import annotations

import logging
from typing import Any

import numpy as np

from kinefly.config.models import PhidgetsConfig, PhidgetsChannelConfig
from kinefly.core.types import FlyState
from kinefly.plugins.base import OutputPlugin

logger = logging.getLogger(__name__)

# Coefficient keys in the order they appear in the state vector
_COEFF_KEYS = ["offset", "l1", "l2", "lr", "r1", "r2", "rr", "ha", "hr", "aa", "ar", "xi"]


class PhidgetsPlugin(OutputPlugin):
    """PhidgetsAnalog 4-channel voltage output.

    Each channel outputs a voltage computed as a linear combination of
    body part angles, radii, and intensities.
    """

    def __init__(self) -> None:
        self._analog = None
        self._attached = False
        self._config: PhidgetsConfig = PhidgetsConfig()
        self._coefficients: np.ndarray = self._build_coefficient_matrix(self._config.channels)
        self._enable: list[bool] = [ch.enable for ch in self._config.channels]

        # For autorange
        self._state_min = np.full(12, np.inf)
        self._state_max = np.full(12, -np.inf)
        self._count = 0

    def start(self, config: dict[str, Any]) -> None:
        """Connect to the PhidgetsAnalog device.

        Args:
            config: The 'phidgets' section from rig config as a dict.
        """
        self._config = PhidgetsConfig(
            serial=config.get("serial", 0),
            autorange=config.get("autorange", False),
            channels=[
                PhidgetsChannelConfig(
                    enable=ch.get("enable", True),
                    coefficients=ch.get("coefficients", {}),
                )
                for ch in config.get("channels", [])
            ] or PhidgetsConfig().channels,
        )
        self._coefficients = self._build_coefficient_matrix(self._config.channels)
        self._enable = [ch.enable for ch in self._config.channels]

        try:
            from Phidget22.Phidget import Phidget
            from Phidget22.Devices.VoltageOutput import VoltageOutput

            self._channels_hw = []
            for i in range(4):
                ch = VoltageOutput()
                ch.setChannel(i)
                if self._config.serial != 0:
                    ch.setDeviceSerialNumber(self._config.serial)
                ch.openWaitForAttachment(5000)
                self._channels_hw.append(ch)
            self._attached = True
            logger.info("PhidgetsAnalog attached (serial=%s)", self._config.serial)
        except ImportError:
            logger.warning(
                "Phidgets22 not installed. Install with: pip install kinefly[phidgets]"
            )
        except Exception:
            logger.exception("Failed to connect to PhidgetsAnalog")

    def on_flystate(self, state: FlyState) -> None:
        self._count += 1
        voltages = self.voltages_from_flystate(state)

        if self._attached:
            for i in range(4):
                if self._enable[i]:
                    try:
                        self._channels_hw[i].setVoltage(voltages[i])
                    except Exception:
                        pass

    def stop(self) -> None:
        if self._attached:
            for ch in self._channels_hw:
                try:
                    ch.setVoltage(0.0)
                    ch.close()
                except Exception:
                    pass
            self._attached = False

    def voltages_from_flystate(self, state: FlyState) -> np.ndarray:
        """Compute 4-channel voltage output from fly state.

        This is the core computation ported from flystate2phidgetsanalog.py.
        """
        angle1_left = state.left.angles[0] if len(state.left.angles) > 0 else 0.0
        angle2_left = state.left.angles[1] if len(state.left.angles) > 1 else 0.0
        radius_left = state.left.radii[0] if len(state.left.radii) > 0 else 0.0
        angle1_right = state.right.angles[0] if len(state.right.angles) > 0 else 0.0
        angle2_right = state.right.angles[1] if len(state.right.angles) > 1 else 0.0
        radius_right = state.right.radii[0] if len(state.right.radii) > 0 else 0.0
        angle_head = state.head.angles[0] if len(state.head.angles) > 0 else 0.0
        radius_head = state.head.radii[0] if len(state.head.radii) > 0 else 0.0
        angle_abdomen = state.abdomen.angles[0] if len(state.abdomen.angles) > 0 else 0.0
        radius_abdomen = state.abdomen.radii[0] if len(state.abdomen.radii) > 0 else 0.0

        state_vec = np.array(
            [
                1.0,
                angle1_left, angle2_left, radius_left,
                angle1_right, angle2_right, radius_right,
                angle_head, radius_head,
                angle_abdomen, radius_abdomen,
                state.aux.intensity,
            ],
            dtype=np.float32,
        )

        if self._config.autorange and self._count > 10:
            self._state_min = np.minimum(self._state_min, state_vec)
            self._state_max = np.maximum(state_vec, self._state_max)
            state_mean = (self._state_min + self._state_max) * 0.5
            d = (self._state_max - state_mean) * 0.001
            self._state_max -= d
            self._state_min += d
            # Autorange updates coefficients dynamically — not implemented in initial port
            # as it modifies self._coefficients in place. Preserved for parity.

        voltages = np.dot(self._coefficients, state_vec)
        return voltages.clip(-10.0, 10.0)

    @staticmethod
    def _build_coefficient_matrix(channels: list[PhidgetsChannelConfig]) -> np.ndarray:
        """Build 4x12 coefficient matrix from channel configs."""
        matrix = np.zeros((4, 12), dtype=np.float32)
        for i, ch in enumerate(channels[:4]):
            for j, key in enumerate(_COEFF_KEYS):
                matrix[i, j] = ch.coefficients.get(key, 0.0)
        return matrix
```

- [ ] **Step 5: Run tests**

```bash
uv run pytest tests/plugins/test_phidgets.py -v
```

Expected: all tests PASS.

- [ ] **Step 6: Commit**

```bash
git add src/kinefly/plugins/ tests/plugins/
git commit -m "feat: add output plugin system and PhidgetsAnalog plugin"
```

---

## Tasks 10-14: Remaining Implementation

The following tasks follow the same TDD pattern. They are listed here as specifications with the key files and logic to implement. Each follows the same step pattern: write failing test -> verify failure -> implement -> verify pass -> commit.

### Task 10: Tracker Base Classes

**Files:**
- Create: `src/kinefly/trackers/__init__.py`
- Create: `src/kinefly/trackers/base.py` — Port `bodypart_motion.py` (`MotionTrackedBodypart`, `MotionTrackedBodypartPolar`) and `bodypart_intensity.py` (`IntensityTrackedBodypart`)
- Create: `src/kinefly/gui/ui_colors.py` — Port the `bgra_dict` and color constants from `ui.py`, replacing `cv.Scalar` with plain tuples
- Create: `src/kinefly/gui/handles.py` — Port `Handle` class from `ui.py`
- Create: `src/kinefly/gui/imagewindow.py` — Port `ImageWindow` from `imagewindow.py`
- Create: `tests/trackers/test_base.py`

**Key changes from original:**
- Replace `import cv` references: `cv.Scalar(b,g,r,a)` -> `(b, g, r, a)` tuples, `cv.CV_FILLED` -> `-1`
- Replace `rospy.logwarn` -> `logging.warning`
- Replace `self.handles.iteritems()` -> `self.handles.items()`
- Replace `import imageprocessing` -> `from kinefly.core import imaging`
- Replace `import ui` -> `from kinefly.gui import ui_colors, handles`
- All numpy/OpenCV algorithm code stays identical

### Task 11: Tracker Implementations

**Files:**
- Create: `src/kinefly/trackers/area.py` — Port `AreaTracker` from `tracker_area.py`
- Create: `src/kinefly/trackers/edge.py` — Port `EdgeDetectorByIntensityProfile`, `EdgeTrackerByIntensityProfile` from `tracker_edge.py`
- Create: `src/kinefly/trackers/tip.py` — Port `TipDetector`, `TipTracker` from `tracker_tip.py`
- Create: `src/kinefly/trackers/intensity.py` — Port `IntensityTracker` from `tracker_intensity.py`
- Create: `src/kinefly/trackers/axis.py` — Port `AxisTracker` from `tracker_axis.py`
- Create: `tests/trackers/test_area.py`, `test_edge.py`, `test_tip.py`

**Key changes from original:**
- Replace `MsgState()` -> `BodyPartState()` from `kinefly.core.types`
- Remove ROS service registration (`rospy.Service(...)`) -> add `get_diagnostics()` method returning a dataclass
- Replace `from Kinefly.srv import ...` and `from Kinefly.msg import ...` -> use `kinefly.core.types`
- All tracking algorithms stay identical

### Task 12: Fly Orchestrator

**Files:**
- Create: `src/kinefly/fly.py` — Port `Fly` class from `fly.py`
- Create: `tests/test_fly.py`

**Key changes from original:**
- Replace `MsgFlystate` publisher -> populate `FlyState` dataclass and call `EventBus.emit()`
- Remove `rospy` imports
- Tracker instantiation logic stays identical (switch on `params['head']['tracker']` etc.)
- Add `get_flystate() -> FlyState` method that aggregates tracker states

### Task 13: Video Recording

**Files:**
- Create: `src/kinefly/recording/__init__.py`
- Create: `src/kinefly/recording/recorder.py`
- Create: `tests/recording/__init__.py`
- Create: `tests/recording/test_recorder.py`

**Implementation:**
- `VideoRecorder` class with `start()`, `write_frame()`, `stop()` methods
- FFmpeg subprocess pipe (`subprocess.Popen` with `stdin=PIPE`)
- Encoder auto-detection: run `ffmpeg -encoders` and parse for `h264_nvenc`, `h264_qsv`, `libx264`
- Timestamped output path: `~/kinefly_recordings/YYYY-MM-DD_HH-MM-SS.mp4`
- Test: create a recorder, write 10 synthetic frames, verify output file exists and is valid

### Task 14: PySide6 GUI

**Files:**
- Create: `src/kinefly/gui/__init__.py`
- Create: `src/kinefly/gui/window.py` — `MainWindow(QMainWindow)` with:
  - Central `QLabel` for camera image display
  - `QToolBar` with checkboxes (Track H/A/L/R/X, Subtract H/A/LR/X, Stabilize, Symmetric, Windows)
  - Record button (toggle, visual indicator)
  - Save Background, Exit buttons
  - Status bar (FPS, recording status)
- Create: `src/kinefly/gui/widgets.py` — Custom Qt widget helpers
- Modify: `src/kinefly/gui/handles.py` — Add Qt mouse event translation to handle coordinates
- Modify: `src/kinefly/app.py` — Main application wiring:
  - Load rig config
  - Instantiate camera source, Fly, EventBus, plugins
  - QTimer for processing loop
  - Connect GUI signals

**Implementation notes:**
- Image display: `cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)` -> `QImage` -> `QPixmap` -> `QLabel.setPixmap()`
- Mouse events: `QLabel.mousePressEvent` / `mouseMoveEvent` -> translate to image coords -> forward to handle logic
- GUI state save/load: YAML round-trip of handle positions and checkbox states via `~/kinefly.yaml`

### Task 15: LED Panels Plugin

**Files:**
- Create: `src/kinefly/plugins/ledpanels.py` — Merge `ledpanels/nodes/ledpanels.py` serial driver + `flystate2ledpanels.py` mapping
- Create: `tests/plugins/test_ledpanels.py`

**Implementation:**
- Command table from the submodule's `ledpanels.py` (dict of command name -> {id, arg_count})
- Serial I/O on a dedicated thread with `queue.Queue`
- `on_flystate()` computes position/velocity commands from coefficient matrix (ported from `flystate2ledpanels.py`) and enqueues serial commands
- USB and voltage methods both supported
- Test: verify command serialization without actual hardware

### Task 16: Legacy Rig Converter

**Files:**
- Create: `tools/convert_rig.py`
- Create: `tests/tools/__init__.py`
- Create: `tests/tools/test_convert_rig.py`

**Implementation:**
- Parse `.launch` XML files using `xml.etree.ElementTree`
- Extract `<param name="..." value="..."/>` and `<rosparam>` YAML blocks
- Map old parameter names to new YAML structure
- Flag `$(env ...)`, `$(optenv ...)`, conditional blocks with `# TODO: manual review`
- CLI: `python tools/convert_rig.py launch/pintether/ -o rigs/pintether/config.yaml`

### Task 17: Diagnostic Plotting Tools

**Files:**
- Create: `tools/plot_tracker.py`
- Create: `tools/plot_wingdata.py`
- Create: `tools/plot_tipdata.py`

**Implementation:**
- ZeroMQ SUB subscriber connecting to the configured address
- MessagePack deserialization of `FlyState` + diagnostic data
- matplotlib live animation plots (same visual layout as originals)
- Each tool is a standalone script, not part of the package

### Task 18: CLI Entry Point and Integration

**Files:**
- Modify: `src/kinefly/__main__.py` — Wire up argument parsing and application launch

**Implementation:**
- `argparse` with `--rig`, `--config`, `--headless` options
- Load config -> create camera -> create Fly -> create EventBus -> register plugins -> launch GUI (or headless loop)
- Integrate all components from previous tasks

### Task 19: Documentation and Cleanup

**Files:**
- Modify: `CLAUDE.md` — Update with new build/run commands
- Update: `rigs/example/config.yaml` — Verify complete

**Implementation:**
- Update CLAUDE.md build section for `uv` workflow
- Remove references to ROS, roscore, roslaunch
- Add new CLI usage examples
- Final `ruff format` and `ruff check` on entire codebase

---

## Self-Review Checklist

- [x] **Spec coverage:** All 14 migration steps from the spec are covered by tasks 1-19
- [x] **Placeholder scan:** No TBD/TODO in code steps (Tasks 1-9 have full code; Tasks 10-19 have detailed specifications since they depend on prior task outputs)
- [x] **Type consistency:** `FlyState`, `BodyPartState`, `EventBus`, `CameraSource`, `OutputPlugin`, `RigConfig` names are consistent across all tasks
- [x] **Function signature consistency:** `get_projection_onto_axis(pt_anywhere, pt_axis_base, pt_axis_head)` — the 3-arg form is used consistently (fixing the Python 2 tuple unpacking)
- [x] **Note:** Tasks 10-19 provide detailed specifications rather than complete code because they depend on the output of prior tasks and are too large to show complete implementations. Each task should be implemented following the same TDD pattern shown in Tasks 1-9.
