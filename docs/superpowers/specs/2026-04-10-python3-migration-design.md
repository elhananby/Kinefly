# Kinefly Python 3 Migration Design Spec

## Goal

Migrate Kinefly from Python 2 + ROS1 to a clean, modern, pure Python 3 application. Remove all ROS dependencies. Preserve tracking algorithm behavior exactly — this is accurate scientific software.

## Architecture Overview

### Core Pattern: Callback + ZeroMQ Hybrid

The main application runs in a single process. The `Fly` object processes each camera frame through the tracker pipeline and emits `FlyState` through an `EventBus`. Output handlers (Phidgets, LED panels, recording) register as callbacks on the EventBus and are called directly. An optional ZeroMQ PUB socket broadcasts `FlyState` for external consumers (plotting tools, custom scripts).

```
Camera Source
    |
    v
Main Loop (QTimer / worker thread)
    |
    v
Fly.update(image) -> FlyState
    |
    v
EventBus.emit(state)
    |--- callback ---> PhidgetsPlugin.on_flystate()
    |--- callback ---> LEDPanelsPlugin.on_flystate()
    |--- callback ---> VideoRecorder.write_frame()
    |--- ZeroMQ PUB -> external subscribers (plot tools, custom scripts)
```

ZeroMQ is opt-in — only activated if the rig config specifies a `zmq.address`. Serialization uses MessagePack (with a `--json` flag on diagnostic tools for debugging).

### Plugin Interface

```python
class OutputPlugin:
    def start(self, config: dict) -> None: ...
    def on_flystate(self, state: FlyState) -> None: ...
    def stop(self) -> None: ...
```

Plugins are loaded based on rig config. If a config section (e.g. `phidgets`, `ledpanels`) is present, the corresponding plugin is instantiated and registered on the EventBus. Plugins that do slow I/O (serial, hardware) manage their own internal queues/threads so `on_flystate()` returns immediately.

## Project Structure

```
kinefly/
├── pyproject.toml
├── src/
│   └── kinefly/
│       ├── __init__.py
│       ├── __main__.py         # CLI entry point
│       ├── app.py              # Main application (PySide6 MainWindow)
│       ├── fly.py              # Fly orchestrator
│       ├── config/
│       │   ├── __init__.py
│       │   ├── models.py       # Config dataclasses (rig, camera, bodypart params)
│       │   └── loader.py       # YAML config loading, validation, defaults
│       ├── core/
│       │   ├── __init__.py
│       │   ├── types.py        # FlyState, BodyPartState dataclasses
│       │   ├── events.py       # EventBus (callback registry + ZeroMQ publisher)
│       │   └── imaging.py      # Polar transforms, phase correlation, window functions
│       ├── camera/
│       │   ├── __init__.py
│       │   ├── base.py         # Abstract CameraSource interface
│       │   ├── harvester.py    # GenICam via Harvesters
│       │   ├── opencv.py       # OpenCV VideoCapture fallback
│       │   └── videofile.py    # Video file replay
│       ├── trackers/
│       │   ├── __init__.py
│       │   ├── base.py         # MotionTrackedBodypart, IntensityTrackedBodypart
│       │   ├── area.py         # AreaTracker
│       │   ├── edge.py         # EdgeTrackerByIntensityProfile
│       │   ├── tip.py          # TipTracker
│       │   ├── intensity.py    # IntensityTracker
│       │   ├── axis.py         # AxisTracker
│       │   └── wingbeat.py     # WingbeatDetector
│       ├── gui/
│       │   ├── __init__.py
│       │   ├── window.py       # PySide6 main window
│       │   ├── widgets.py      # Custom Qt widgets
│       │   └── handles.py      # Handle overlay logic (drawn on image via OpenCV)
│       ├── recording/
│       │   ├── __init__.py
│       │   └── recorder.py     # FFmpeg-based video recording
│       └── plugins/
│           ├── __init__.py
│           ├── base.py         # OutputPlugin interface
│           ├── phidgets.py     # PhidgetsAnalog voltage output
│           └── ledpanels.py    # LED panels serial driver
├── tools/
│   ├── convert_rig.py          # Legacy launch-file to YAML converter
│   ├── plot_tracker.py         # Diagnostic: tracker data (ZeroMQ subscriber)
│   ├── plot_wingdata.py        # Diagnostic: wing data
│   └── plot_tipdata.py         # Diagnostic: tip data
├── rigs/
│   └── example/
│       └── config.yaml
└── tests/
```

## Data Models

Replace all ROS message types with Python dataclasses. Identical field names so downstream access patterns (`state.left.angles[0]`) are unchanged.

```python
@dataclass
class BodyPartState:
    """Replaces MsgState."""
    angles: Sequence[float] = field(default_factory=list)
    gradients: Sequence[float] = field(default_factory=list)
    radii: Sequence[float] = field(default_factory=list)
    freq: float = 0.0
    intensity: float = 0.0

@dataclass
class FlyState:
    """Replaces MsgFlystate."""
    timestamp: float
    seq: int
    head: BodyPartState
    abdomen: BodyPartState
    left: BodyPartState
    right: BodyPartState
    aux: BodyPartState
```

## Camera Abstraction

Abstract `CameraSource` interface with three implementations:

```python
class CameraSource(ABC):
    def open(self) -> None: ...
    def read(self) -> tuple[np.ndarray, float]: ...  # (image, timestamp)
    def close(self) -> None: ...
    @property
    def fps(self) -> float: ...
    @property
    def resolution(self) -> tuple[int, int]: ...
```

- **`HarvesterCamera`** — GenICam via Harvesters library. Config: CTI file path, serial number, exposure, gain, framerate.
- **`OpenCVCamera`** — `cv2.VideoCapture(device_index)`. Development/testing fallback.
- **`VideoFileSource`** — `cv2.VideoCapture(filepath)`. Replays recorded video. Option to run at original framerate or as fast as possible (for batch retracking).

Selected via `camera.source` in rig config (`harvester`, `opencv`, or `file`).

## Video Recording

FFmpeg subprocess pipe — frames written as raw pixels to FFmpeg's stdin.

- **Encoder auto-detection**: probes `ffmpeg -encoders`, selects `h264_nvenc` -> `h264_qsv` -> `libx264`
- **Output format**: MP4, timestamped filename (e.g. `~/kinefly_recordings/2026-04-10_14-53-28.mp4`)
- **GUI**: Record button in toolbar, red indicator when active
- **Performance**: FFmpeg runs in its own process; pipe buffer absorbs timing jitter; doesn't block tracking pipeline
- **Dependency**: ffmpeg is a system dependency, detected at runtime. Fails gracefully with clear message if not found.

## Tracker Migration Strategy

### Principle: Preserve Algorithmic Behavior Exactly

Trackers are migrated with **minimal changes only**:
- Fix Python 2 syntax (`except X, e:` -> `except X as e:`, tuple unpacking in function signatures)
- `rospy.logwarn` -> Python `logging`
- ROS service responses (diagnostic data) -> direct method calls returning dataclasses
- `cv_bridge` removed (images are already numpy arrays from camera abstraction)
- All OpenCV and numpy calls remain identical
- `imageprocessing.py` functions preserved as-is (polar transforms, phase correlation, window functions)

### Class Hierarchy (preserved)

```
TrackerBase (new thin ABC)
├── MotionTrackedBodypart (from bodypart_motion.py)
│   └── MotionTrackedBodypartPolar
│       ├── AreaTracker
│       ├── EdgeTrackerByIntensityProfile
│       └── TipTracker
└── IntensityTrackedBodypart (from bodypart_intensity.py)
    └── IntensityTracker

AxisTracker — standalone
WingbeatDetector — used by IntensityTracker
```

### Deferred Optimization Opportunities

NOT done during migration. Each requires numerical equivalence testing first:
- `filter_median` -> `scipy.ndimage.median_filter`
- `PhaseCorrelation.get_shift` -> `skimage.registration.phase_cross_correlation`
- Window functions -> `scipy.signal.windows`
- Polar transforms -> `scipy.ndimage.map_coordinates`

## GUI (PySide6)

### Approach: Minimal Port

- PySide6 `QMainWindow` with central `QLabel` displaying camera image
- Handles still drawn on the image via OpenCV before display — preserves exact interaction behavior
- Mouse events on QLabel translated to image coordinates, forwarded to existing handle logic
- Toolbar with real Qt widgets replacing hand-drawn buttons:
  - Track checkboxes: H, A, L, R, X
  - Subtract checkboxes: H, A, LR, X
  - Stabilize, Symmetric, Windows checkboxes
  - Record button (toggle, red indicator)
  - Save Background, Exit buttons
- Status bar: framerate, recording status
- Debug windows (`ImageWindow`) become additional QLabel windows, toggled by Windows checkbox

### GUI State Persistence

Handle positions and checkbox states saved to `~/kinefly.yaml` on exit, restored on launch. Separate from rig config. Path configurable via `gui.state_file` in rig config.

### Image Update Loop

A `QTimer` drives the processing loop on the main thread (simplest approach, adequate for single-camera setups at typical framerates). Each tick:
1. Grab frame from camera source
2. `Fly.update(image)` -> produces `FlyState`
3. Draw overlays (handles, tracking visualization) on image
4. Update QLabel with annotated image
5. `EventBus.emit(state)` -> dispatches to plugins and ZeroMQ

## Rig Configuration

### Format: Single YAML File Per Rig

```yaml
kinefly:
  version: 2

camera:
  source: harvester
  cti_file: /opt/mvIMPACT/lib/mvGenTLProducer.cti
  serial: "DA0012345"
  exposure_us: 5000
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
  encoder: auto  # auto | h264_nvenc | h264_qsv | libx264

gui:
  state_file: ~/kinefly.yaml

# Optional — omit to disable
zmq:
  address: tcp://*:5555

phidgets:
  serial: 0
  autorange: false
  channels:
    - enable: true
      coefficients: {offset: 0, l1: 1.0, l2: 0, lr: 0, r1: 0, r2: 0, rr: 0, ha: 0, hr: 0, aa: 0, ar: 0, xi: 0}
    - enable: true
      coefficients: {offset: 0, l1: 0, l2: 0, lr: 0, r1: 1.0, r2: 0, rr: 0, ha: 0, hr: 0, aa: 0, ar: 0, xi: 0}
    - enable: true
      coefficients: {offset: 0, l1: 1.0, l2: 0, lr: 0, r1: -1.0, r2: 0, rr: 0, ha: 0, hr: 0, aa: 0, ar: 0, xi: 0}
    - enable: true
      coefficients: {offset: 0, l1: 1.0, l2: 0, lr: 0, r1: 1.0, r2: 0, rr: 0, ha: 0, hr: 0, aa: 0, ar: 0, xi: 0}

ledpanels:
  serial_port: /dev/ttyUSB0
  baud_rate: 115200
  xpos: {adc0: 1.0, adc1: 0, adc2: 0, adc3: 0, funcx: 0, funcy: 0}
  xvel: {adc0: 0, adc1: 0, adc2: 0, adc3: 0, funcx: 0, funcy: 0}
  ypos: {adc0: 0, adc1: 0, adc2: 0, adc3: 0, funcx: 0, funcy: 0}
  yvel: {adc0: 0, adc1: 0, adc2: 0, adc3: 0, funcx: 0, funcy: 0}
```

### Rig Discovery

1. `kinefly --rig pintether` -> looks in `./rigs/pintether/config.yaml`
2. `kinefly --config /absolute/path/to/config.yaml` for arbitrary locations
3. `$KINEFLY_RIG_DIR` env var fallback

### Legacy Converter

`python tools/convert_rig.py launch/pintether/ -o rigs/pintether/config.yaml`

- Parses `.launch` XML files, extracts `<param>` and `<rosparam>` blocks
- Maps old parameter names to new YAML structure
- Flags unconvertible items with `# TODO: manual review` comments (env var references, conditional logic)
- Prints conversion summary

## Phidgets Integration

PhidgetsAnalog voltage output only, using `phidgets22` library.

- 4-channel configurable linear combination of body part measurements -> voltage
- Coefficient matrix with named keys per channel (replaces cryptic `v0l1` naming)
- Autorange mode: automatically scales L, R, L-R, L+R to [-10V, +10V] range
- Attach/detach callbacks for hot-plugging
- Zeroes all channels on shutdown

## LED Panels Integration

Merged from the `ledpanels` git submodule into `plugins/ledpanels.py`.

- Full command table preserved (~100+ commands with IDs, argument counts, validation)
- Serial protocol unchanged (variable-length byte sequences)
- Serial I/O runs on a dedicated thread with command queue so `on_flystate()` returns immediately
- Coefficient-based mapping from fly state to panel position/velocity commands (absorbed from `flystate2ledpanels.py`)
- The `ledpanels` submodule can be removed after migration

## Diagnostic Plotting Tools

Three standalone scripts in `tools/`, connecting over ZeroMQ:

- `plot_tracker.py` — internal tracker data (intensity profiles, thresholds)
- `plot_wingdata.py` — wing edge data
- `plot_tipdata.py` — tip tracker data

These subscribe to ZeroMQ, so they can attach/detach at runtime without affecting the main application. They serve as reference implementations for external ZeroMQ consumers.

Tracker diagnostic data is exposed via a `get_diagnostics()` method on each tracker, serialized and published alongside `FlyState` on a separate ZeroMQ topic.

## Dependencies

### Core (required)
- `numpy`
- `opencv-python`
- `PySide6`
- `pyyaml`

### Optional (via extras)
- `harvesters`, `genicam` — GenICam camera support
- `Phidgets22` — voltage output
- `pyserial` — LED panels
- `pyzmq`, `msgpack` — ZeroMQ event broadcasting
- `matplotlib` — plotting tools (plus `pyzmq`, `msgpack`)

### System
- `ffmpeg` — video recording (detected at runtime)

### Tooling
- `uv` — project management
- `ruff` — linting/formatting (target Python 3.11, line length 100)
- `pytest` — testing

### Python Version
- `>=3.11`

## Migration Order

Incremental, bottom-up, each step independently testable:

1. Project skeleton (uv, pyproject.toml, ruff config, package structure)
2. Rig config system (YAML loader, validation, defaults) — needed by all subsequent steps
3. Data models (`core/types.py`) and event system (`core/events.py`)
4. Leaf modules: `setdict.py`, `imageprocessing.py`, `wingbeatdetector.py`
5. Tracker base classes (`trackers/base.py` from `bodypart_motion.py`, `bodypart_intensity.py`)
6. Tracker implementations (area, edge, tip, intensity, axis)
7. `Fly` orchestrator
8. Camera abstraction layer (Harvesters, OpenCV, video file)
9. PySide6 GUI
10. Video recording
11. Output plugins (Phidgets, LED panels)
12. Legacy rig converter (`tools/convert_rig.py`)
13. Diagnostic plotting tools (ZeroMQ subscribers)
14. Example rig configs and documentation

## References

- Original publication: Safarik et al., J. Neurosci. 2016, DOI: 10.1523/JNEUROSCI.2277-16.2016
- Source: https://github.com/ssafarik/Kinefly
- LED panels hardware: bitbucket.org/mreiser/panels
