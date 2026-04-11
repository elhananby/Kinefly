# Kinefly

Real-time kinematic tracking of tethered winged insects from camera images.

Kinefly extracts body-part motion variables — head, abdomen, left wing, right wing, and an auxiliary region — from a live or recorded camera stream. Several tracking algorithms are available per body part, and all output is accessible via Python callbacks or a ZeroMQ socket for downstream closed-loop experiments.

---

## Requirements

- Python ≥ 3.11
- [uv](https://docs.astral.sh/uv/) (package manager)

## Installation

```bash
git clone https://github.com/elhananby/Kinefly.git
cd Kinefly
uv sync                  # core deps only
uv sync --extra dev      # + ruff, pytest (recommended for development)
uv sync --extra all      # + all optional hardware/plotting deps
```

### Optional extras

| Extra | Installs | When you need it |
|---|---|---|
| `harvester` | `harvesters`, `genicam` | GenICam / GigE cameras |
| `phidgets` | `Phidget22` | PhidgetsAnalog voltage output |
| `ledpanels` | `pyserial` | LED panel serial control |
| `zmq` | `pyzmq`, `msgpack` | ZeroMQ state publishing |
| `plotting` | `matplotlib`, `pyzmq`, `msgpack` | Diagnostic plot tools |
| `all` | everything above | Full rig setup |

---

## Quick start

```bash
# Run with built-in/USB webcam (OpenCV fallback)
uv run python -m kinefly --rig rigs/example/config.yaml
```

Press `Ctrl-C` to stop.

> **Note:** Without a GUI state file (`~/kinefly.yaml`) containing hinge positions, trackers run in stub mode — frames are read and the pipeline runs, but no kinematic state is computed. The PySide6 GUI (in development) is used to set hinge positions interactively.

---

## Configuration

Each experimental rig is described by a single YAML file. See [`rigs/example/config.yaml`](rigs/example/config.yaml) for the full schema. Key sections:

```yaml
camera:
  source: opencv        # opencv | videofile | harvester
  device_index: 0
  framerate: 60

tracking:
  head:
    tracker: area       # area | edge | tip | intensity | axis
    threshold: 0.0
  left:
    tracker: edge
    threshold: 0.01
  # ... abdomen, right, aux

# Optional: publish FlyState over ZeroMQ
zmq:
  address: tcp://*:5555
```

### Migrate a legacy ROS rig

```bash
python tools/convert_rig.py launch/thadsrig/ -o rigs/thadsrig/config.yaml
```

---

## Running

```bash
# Basic
uv run python -m kinefly --rig rigs/myrig/config.yaml

# With ZeroMQ publishing
uv run python -m kinefly --rig rigs/myrig/config.yaml --zmq tcp://*:5555

# With video recording
uv run python -m kinefly --rig rigs/myrig/config.yaml --record

# Verbose logging
uv run python -m kinefly --rig rigs/myrig/config.yaml -v
```

---

## Output

`FlyState` is emitted after every frame via the `EventBus`.

### Python callback

```python
from kinefly.core.events import EventBus
from kinefly.core.types import FlyState

bus = EventBus()

def on_state(state: FlyState) -> None:
    print(state.left.angles, state.right.angles)

bus.register(on_state)
```

### ZeroMQ subscriber

```python
import zmq, msgpack

ctx = zmq.Context()
sub = ctx.socket(zmq.SUB)
sub.connect("tcp://localhost:5555")
sub.setsockopt(zmq.SUBSCRIBE, b"")

while True:
    state = msgpack.unpackb(sub.recv(), raw=False)
    print(state["left"]["angles"])
```

### FlyState fields

| Field | Type | Description |
|---|---|---|
| `timestamp` | `float` | Seconds since epoch |
| `seq` | `int` | Frame sequence number |
| `head`, `abdomen`, `left`, `right`, `aux` | `BodyPartState` | Per-bodypart state |

Each `BodyPartState`:

| Field | Unit | Description |
|---|---|---|
| `angles` | radians | Detected edge / rotation angles |
| `radii` | pixels | Radial distance from hinge |
| `gradients` | — | Intensity gradients at detected edges |
| `intensity` | [0, 1] | Mean pixel intensity in ROI |
| `freq` | Hz | Wingbeat frequency (aux only) |

---

## Trackers

| Tracker | Key | Typical use |
|---|---|---|
| `AreaTracker` | `area` | Head, abdomen — phase-correlation rotation |
| `EdgeTrackerByIntensityProfile` | `edge` | Wings — radial intensity gradient edges |
| `TipTracker` | `tip` | Wing tip — farthest point from hinge |
| `IntensityTracker` | `intensity` | Aux region, wingbeat frequency |
| `AxisTracker` | `axis` | Body axis — two-point line |

---

## Hardware plugins

### PhidgetsAnalog (voltage output)

Add to your rig config:

```yaml
phidgets:
  serial: 0          # 0 = any device
  autorange: false
  channels:
    - enable: true
      coefficients: {l1: 5.0, r1: -5.0}   # L−R on channel 0
    - enable: true
      coefficients: {r1: 5.0}
```

Install: `uv sync --extra phidgets`

### LED panels (serial USB)

```yaml
ledpanels:
  port: /dev/ttyUSB0
  method: usb          # usb | voltage
  mode: velocity       # velocity | position
  pattern_id: 1
  coeff_usb:
    xl1: 1.0           # x velocity = left wing angle
    xr1: -1.0          #            − right wing angle
```

Install: `uv sync --extra ledpanels`

---

## Diagnostic tools

These tools require a running Kinefly instance with ZeroMQ enabled (`--zmq tcp://*:5555`).

```bash
# Single body-part: intensity + angle time-series
python tools/plot_tracker.py --address tcp://localhost:5555 --tracker left

# Both wings: angle + intensity
python tools/plot_wingdata.py --address tcp://localhost:5555

# Tip tracker: angle + radius
python tools/plot_tipdata.py --address tcp://localhost:5555 --tracker left
```

Install: `uv sync --extra plotting`

---

## Development

```bash
uv sync --extra dev

# Tests
uv run pytest

# Lint
uv run ruff check src/ tests/ tools/

# Format
uv run ruff format src/ tests/ tools/
```

---

## Project structure

```
src/kinefly/
├── __main__.py         # CLI entry point
├── fly.py              # Orchestrates all trackers, emits FlyState
├── core/
│   ├── types.py        # FlyState, BodyPartState dataclasses
│   ├── events.py       # EventBus (callbacks + ZeroMQ)
│   └── imaging.py      # Polar transforms, phase correlation
├── config/             # YAML loader, dataclass models
├── camera/             # CameraSource ABC, OpenCV, VideoFile, Harvester
├── trackers/           # area, edge, tip, intensity, axis, wingbeat
├── plugins/            # OutputPlugin base, Phidgets, LED panels
├── recording/          # FFmpeg video recorder
└── gui/                # Colors, handles, image windows

tools/
├── convert_rig.py      # Migrate legacy ROS .launch dirs to YAML
├── plot_tracker.py     # Live diagnostic: single body part
├── plot_wingdata.py    # Live diagnostic: both wings
└── plot_tipdata.py     # Live diagnostic: tip tracker

rigs/
└── example/
    └── config.yaml     # Annotated example rig config
```

---

## License

MIT
