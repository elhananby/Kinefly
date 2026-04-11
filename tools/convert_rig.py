"""Convert a legacy Kinefly ROS launch directory to a new YAML rig config."""

import argparse
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

_SUBSTITUTION_RE = re.compile(r"\$\((?:env|optenv|find)\s+[^)]+\)")

_SKIP_PARAMS = frozenset([
    "parameterfile",
    "filenameBackground",
    "image_topic",
    "n_queue_images",
    "ExposureAuto",
    "GainAuto",
    "AcquisitionMode",
    "TriggerMode",
    "TriggerSource",
    "softwaretriggerrate",
    "frame_id",
    "mtu",
    "Acquire",
])

# Simple flat mapping: raw param name -> tuple of keys in output config dict
_KINEFLY_PARAM_MAP: dict[str, tuple[str, ...]] = {
    "n_edges_max": ("tracking", "n_edges_max"),
    "rc_background": ("tracking", "rc_background"),
    "scale_image": ("camera", "scale_image"),
    "use_gui": ("tracking", "use_gui"),
    "wingbeat_min": ("tracking", "wingbeat_min"),
    "wingbeat_max": ("tracking", "wingbeat_max"),
    # camera params
    "AcquisitionFrameRate": ("camera", "framerate"),
    "ExposureTimeAbs": ("camera", "exposure_time_us"),
    "Gain": ("camera", "gain"),
}

# Bodypart prefixes for tracking section
_BODYPART_PREFIXES = ("head", "abdomen", "left", "right", "aux")

# Phidgets coefficient key remapping: raw suffix -> output key
_PHIDGETS_COEFF_KEYS = frozenset(
    ["ha", "hr", "aa", "ar", "l1", "l2", "lr", "r1", "r2", "rr", "xi"]
)

# Phidgets top-level keys (non-channel)
_PHIDGETS_TOP_KEYS = frozenset(["autorange", "serial"])

# LED panels top-level keys
_LEDPANELS_TOP_KEYS = frozenset(["axis", "method", "mode", "pattern_id"])


def parse_launch_file(path: Path) -> tuple[dict[str, tuple[str, str]], list[str]]:
    """Parse a .launch XML file and return (flat {name: (type, value)} dict, warnings)."""
    params: dict[str, tuple[str, str]] = {}
    file_warnings: list[str] = []
    try:
        tree = ET.parse(path)
    except ET.ParseError as exc:
        raise ValueError(f"Cannot parse {path}: {exc}") from exc

    root = tree.getroot()

    # Scan for conditional attributes (if= / unless=)
    for elem in root.iter():
        if elem.get("if") is not None or elem.get("unless") is not None:
            attr = "if" if elem.get("if") is not None else "unless"
            val = elem.get(attr)
            tag = elem.get("name") or elem.tag
            file_warnings.append(
                f"Element '{tag}' in {path.name} has conditional attribute"
                f" '{attr}=\"{val}\"' — output may vary at runtime"
            )

    for elem in root.iter("param"):
        name = elem.get("name", "").strip()
        value = elem.get("value", "")
        type_hint = elem.get("type")
        if name:
            params[name] = (type_hint, value)

    for elem in root.iter("rosparam"):
        param_name = elem.get("param", "").strip()
        if not param_name:
            continue
        text = elem.text
        if not text or not text.strip():
            file_warnings.append(
                f"<rosparam param=\"{param_name}\"> in {path.name} has empty text — skipping"
            )
            continue
        try:
            parsed = yaml.safe_load(text)
        except yaml.YAMLError as exc:
            file_warnings.append(
                f"<rosparam param=\"{param_name}\"> in {path.name}"
                f" failed YAML parse: {exc} — skipping"
            )
            continue
        if parsed is None:
            file_warnings.append(
                f"<rosparam param=\"{param_name}\"> in {path.name} parsed as None — skipping"
            )
            continue
        if isinstance(parsed, dict):
            for k, v in parsed.items():
                params[f"{param_name}/{k}"] = ("rosparam", str(v))
        else:
            params[param_name] = ("rosparam", str(parsed))

    return params, file_warnings


def coerce_value(value: str, type_hint: str | None) -> object:
    """Convert string value to Python type based on ROS type hint."""
    if type_hint == "bool":
        return value.strip().lower() in ("true", "1", "yes")
    if type_hint == "int":
        return int(value)
    if type_hint == "double":
        return float(value)
    # "string" or None or anything else -> keep as str
    return value


def has_substitution(value: str) -> bool:
    """Return True if value contains $(env ...) or similar substitution."""
    return bool(_SUBSTITUTION_RE.search(value))


def set_nested(d: dict, keys: tuple[str, ...], value: object) -> None:
    """Set a value in a nested dict using a tuple of keys."""
    for key in keys[:-1]:
        d = d.setdefault(key, {})
    d[keys[-1]] = value


def _ensure_phidgets_channel(phidgets: dict, n: int) -> dict:
    """Ensure phidgets.channels list has an entry at index n."""
    channels = phidgets.setdefault("channels", [])
    while len(channels) <= n:
        channels.append({})
    return channels[n]


def _handle_phidgets_param(
    name: str,
    type_hint: str | None,
    raw_value: str,
    config: dict,
    warnings: list[str],
) -> bool:
    """
    Process a flystate2phidgetsanalog/* param and populate config["phidgets"].
    Returns True if the param was recognised and handled.
    """
    prefix = "flystate2phidgetsanalog/"
    if not name.startswith(prefix):
        return False

    key = name[len(prefix):]
    phidgets = config.setdefault("phidgets", {})
    value = coerce_value(raw_value, type_hint)

    # Top-level phidgets keys
    if key in _PHIDGETS_TOP_KEYS:
        phidgets[key] = value
        return True

    # Channel keys: v{n}enable or v{n}{coeff}
    m = re.fullmatch(r"v(\d+)(.+)", key)
    if m:
        n = int(m.group(1))
        suffix = m.group(2)
        channel = _ensure_phidgets_channel(phidgets, n)
        if suffix == "enable":
            channel["enable"] = value
        elif suffix == "0":
            channel.setdefault("coefficients", {})["offset"] = value
        elif suffix in _PHIDGETS_COEFF_KEYS:
            channel.setdefault("coefficients", {})[suffix] = value
        else:
            warnings.append(f"Unrecognised phidgets channel suffix '{suffix}' in param '{name}'")
        return True

    warnings.append(f"Unrecognised phidgets param '{name}'")
    return True  # still "handled" (it was a phidgets param)


def _handle_ledpanels_param(
    name: str,
    type_hint: str | None,
    raw_value: str,
    config: dict,
    warnings: list[str],
) -> bool:
    """
    Process a flystate2ledpanels/* param and populate config["ledpanels"].
    Returns True if the param was recognised and handled.
    """
    prefix = "flystate2ledpanels/"
    if not name.startswith(prefix):
        return False

    key = name[len(prefix):]
    ledpanels = config.setdefault("ledpanels", {})
    value = coerce_value(raw_value, type_hint)

    if key in _LEDPANELS_TOP_KEYS:
        ledpanels[key] = value
        return True

    # coeff_voltage/* or coeff_usb/*
    m = re.fullmatch(r"(coeff_voltage|coeff_usb)/(.+)", key)
    if m:
        section = m.group(1)
        subkey = m.group(2)
        ledpanels.setdefault(section, {})[subkey] = value
        return True

    warnings.append(f"Unrecognised ledpanels param '{name}'")
    return True  # still "handled"


def convert_rig_dir(launch_dir: Path) -> tuple[dict, list[str]]:
    """Convert a rig launch directory to new config dict.

    Returns (config_dict, warnings) where warnings lists params that need
    manual review (substitutions, unrecognised params).
    """
    config: dict = {
        "kinefly": {"version": 2},
    }
    warnings: list[str] = []

    # Collect all params from recognised launch files
    launch_files = [
        "params_kinefly.launch",
        "params_camera.launch",
        "params_phidgetsanalog.launch",
        "params_ledpanels.launch",
    ]

    all_params: dict[str, tuple[str | None, str]] = {}
    files_parsed = 0
    for fname in launch_files:
        fpath = launch_dir / fname
        if fpath.exists():
            file_params, file_warnings = parse_launch_file(fpath)
            all_params.update(file_params)
            warnings.extend(file_warnings)
            files_parsed += 1

    if files_parsed == 0:
        found_variants = sorted(launch_dir.glob("params_*.launch"))
        extra = (
            f" Found: {', '.join(p.name for p in found_variants)}."
            if found_variants
            else ""
        )
        warnings.append(
            f"No recognized launch files found in {launch_dir}. "
            "Expected params_kinefly.launch, params_camera.launch, etc. "
            f"Numbered variants (e.g. params_kinefly_1.launch) are not supported.{extra}"
        )

    for name, (type_hint, raw_value) in all_params.items():
        # 1. Skip ROS-specific params
        if name in _SKIP_PARAMS:
            continue

        # 2. Phidgets params
        if _handle_phidgets_param(name, type_hint, raw_value, config, warnings):
            continue

        # 3. LED panels params
        if _handle_ledpanels_param(name, type_hint, raw_value, config, warnings):
            continue

        # 4. Check for substitution expressions before coercing
        if has_substitution(raw_value):
            warnings.append(
                f"Param '{name}' contains substitution expression"
                f" '{raw_value}' — manual review needed"
            )
            # Still include raw string in output
            value: object = raw_value
        else:
            value = coerce_value(raw_value, type_hint)

        # 5. Bodypart-prefixed params (head/*, abdomen/*, left/*, right/*, aux/*)
        bodypart_handled = False
        for bp in _BODYPART_PREFIXES:
            if name.startswith(f"{bp}/"):
                subkey = name[len(bp) + 1:]
                set_nested(config, ("tracking", bp, subkey), value)
                bodypart_handled = True
                break

        if bodypart_handled:
            continue

        # 6. Flat mapping
        if name in _KINEFLY_PARAM_MAP:
            set_nested(config, _KINEFLY_PARAM_MAP[name], value)
            continue

        # 7. Unknown param
        warnings.append(f"Unrecognised param '{name}' — not mapped to new config")

    return config, warnings


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Convert legacy ROS launch rig to YAML config"
    )
    parser.add_argument("launch_dir", type=Path, help="Path to rig launch directory")
    parser.add_argument(
        "-o",
        "--output",
        type=Path,
        default=None,
        help="Output YAML path (default: stdout)",
    )
    args = parser.parse_args()

    config, warnings = convert_rig_dir(args.launch_dir)

    yaml_str = yaml.dump(config, default_flow_style=False, sort_keys=False)

    if warnings:
        print("# Warnings (manual review needed):", file=sys.stderr)
        for w in warnings:
            print(f"#   {w}", file=sys.stderr)

    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(yaml_str)
        print(f"Written to {args.output}", file=sys.stderr)
    else:
        print(yaml_str)


if __name__ == "__main__":
    main()
