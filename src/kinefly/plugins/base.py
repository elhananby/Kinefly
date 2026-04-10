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
