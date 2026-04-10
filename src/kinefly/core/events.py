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
