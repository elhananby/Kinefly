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
