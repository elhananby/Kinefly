"""CLI entry point for kinefly.

Wires up: config -> camera -> EventBus -> plugins -> Fly -> headless loop.
"""

from __future__ import annotations

import argparse
import logging
import signal
import sys
from pathlib import Path

logger = logging.getLogger(__name__)


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="kinefly",
        description="Kinefly: fly kinematics extraction from camera images.",
    )
    parser.add_argument(
        "--rig",
        "--config",
        metavar="PATH",
        type=Path,
        default=Path("rigs/example/config.yaml"),
        dest="rig",
        help="Path to rig config YAML (default: rigs/example/config.yaml)",
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        default=False,
        help="Run without GUI (headless frame loop, no window)",
    )
    parser.add_argument(
        "--record",
        action="store_true",
        help="Enable video recording",
    )
    parser.add_argument(
        "--zmq",
        action="store_true",
        default=False,
        help=(
            "Enable ZeroMQ FlyState publishing. "
            "Address is taken from the rig config's zmq.address field, "
            "or defaults to tcp://*:5555 if not set."
        ),
    )
    parser.add_argument(
        "--version",
        action="version",
        version=f"kinefly {_get_version()}",
    )
    parser.add_argument(
        "-v",
        "--verbose",
        action="store_true",
        help="Enable debug logging",
    )
    return parser


def _get_version() -> str:
    try:
        import importlib.metadata

        return importlib.metadata.version("kinefly")
    except Exception:
        return "unknown"


def _create_camera(config):
    """Instantiate the correct CameraSource from config."""
    source = config.camera.source.lower()
    if source == "opencv":
        from kinefly.camera.opencv import OpenCVCamera

        return OpenCVCamera(
            device_index=getattr(config.camera, "device_index", 0),
            framerate=config.camera.framerate,
        )
    elif source == "videofile":
        from kinefly.camera.videofile import VideoFileSource

        return VideoFileSource(
            path=getattr(config.camera, "path", ""),
        )
    elif source == "harvester":
        from kinefly.camera.harvester import HarvesterCamera

        return HarvesterCamera(
            framerate=config.camera.framerate,
        )
    else:
        raise ValueError(f"Unknown camera source: {source!r}")


def _load_gui_state(state_file: str) -> dict:
    """Load GUI state YAML (hinge positions, etc.). Returns {} if not found."""
    import yaml

    path = Path(state_file).expanduser()
    if not path.exists():
        logger.warning(
            "GUI state file not found: %s"
            " — hinge positions will use defaults; tracking may be inaccurate",
            path,
        )
        return {}
    with open(path) as f:
        return yaml.safe_load(f) or {}


def _build_fly_params(config, gui_state: dict) -> dict:
    """Merge rig config + GUI state into the params dict expected by Fly."""
    # Fly expects params['gui'] with hinge positions + params['left']['threshold'] etc.
    # If gui_state has the structure, use it; otherwise return minimal.
    params = gui_state.copy()

    # Overlay tracking thresholds/tracker types from rig config
    tracking = config.tracking
    for name in ("head", "abdomen", "left", "right"):
        bp_config = getattr(tracking, name, None)
        if bp_config is None:
            continue
        params.setdefault("gui", {}).setdefault(name, {})
        params.setdefault(name, {})
        params[name]["threshold"] = bp_config.threshold
        params[name]["tracker"] = bp_config.tracker
        params[name]["autozero"] = bp_config.autozero

    params.setdefault("gui", {})
    params["gui"]["windows"] = False  # no popup windows in headless mode
    aux = getattr(tracking, "aux", None)
    if aux is not None:
        params["wingbeat_min"] = aux.wingbeat_min
        params["wingbeat_max"] = aux.wingbeat_max

    return params


def main() -> int:
    parser = build_arg_parser()
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(name)s %(levelname)s %(message)s",
    )

    # Load rig config
    from kinefly.config.loader import load_rig_config

    try:
        config = load_rig_config(args.rig)
    except FileNotFoundError:
        logger.error("Rig config not found: %s", args.rig)
        return 1
    except Exception:
        logger.exception("Failed to load rig config: %s", args.rig)
        return 1

    # Create EventBus
    from kinefly.core.events import EventBus

    _DEFAULT_ZMQ_ADDRESS = "tcp://*:5555"
    zmq_enabled = args.zmq or (config.zmq is not None)
    if zmq_enabled:
        zmq_address = config.zmq.address if config.zmq else _DEFAULT_ZMQ_ADDRESS
    else:
        zmq_address = None
    bus = EventBus(zmq_address=zmq_address)

    # Register plugins
    plugins = []
    from dataclasses import asdict

    if config.phidgets:
        from kinefly.plugins.phidgets import PhidgetsPlugin

        plugin = PhidgetsPlugin()
        plugin.start(asdict(config.phidgets))
        bus.register(plugin.on_flystate)
        plugins.append(plugin)
    if config.ledpanels:
        from kinefly.plugins.ledpanels import LedPanelsPlugin

        plugin = LedPanelsPlugin()
        plugin.start(asdict(config.ledpanels))
        bus.register(plugin.on_flystate)
        plugins.append(plugin)

    # Load GUI state and create Fly
    gui_state = _load_gui_state(config.gui.state_file)
    fly_params = _build_fly_params(config, gui_state)

    from kinefly.fly import Fly

    fly = Fly(params=fly_params, event_bus=bus)

    # Create camera
    try:
        camera = _create_camera(config)
        camera.open()
    except Exception:
        logger.exception("Failed to open camera")
        for p in plugins:
            p.stop()
        bus.stop_zmq()
        return 1

    # Optional recorder
    recorder = None
    if args.record:
        import datetime

        from kinefly.recording.recorder import VideoRecorder

        recorder = VideoRecorder()
        out_dir = Path(config.recording.output_dir).expanduser()
        out_dir.mkdir(parents=True, exist_ok=True)
        ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        out_path = out_dir / f"kinefly_{ts}.mp4"
        w, h = camera.resolution
        recorder.start(w, h, camera.fps, str(out_path))
        logger.info("Recording to %s", out_path)

    # --- GUI mode ---
    if not args.headless:
        from kinefly.app import KineflyApp

        app = KineflyApp(
            config=config,
            camera=camera,
            fly=fly,
            bus=bus,
            plugins=plugins,
            recorder=recorder,
            state_file=config.gui.state_file,
        )
        return app.run()

    # --- Headless mode ---
    running = True

    def _sigint_handler(sig, frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, _sigint_handler)

    logger.info("Kinefly running headless. Press Ctrl-C to stop.")

    try:
        while running:
            try:
                frame, timestamp = camera.read()
            except StopIteration:
                logger.info("End of video source.")
                break
            except Exception:
                logger.exception("Camera read error")
                break

            fly.update(frame, timestamp)

            if recorder is not None:
                recorder.write_frame(frame)
    finally:
        camera.close()
        if recorder is not None:
            recorder.stop()
            logger.info("Recording saved to %s", out_path)
        for p in plugins:
            p.stop()
        bus.stop_zmq()
        logger.info("Kinefly stopped.")

    return 0


if __name__ == "__main__":
    sys.exit(main())
