"""Recursive dictionary merge utilities.

Ported from nodes/setdict.py. Provides preserve-or-overwrite merging of
nested dictionaries, used throughout Kinefly for merging default parameters
with user-supplied configuration.
"""

from __future__ import annotations

from typing import Any


def _set_dict(target: dict[str, Any], source: dict[str, Any], preserve: bool) -> None:
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
