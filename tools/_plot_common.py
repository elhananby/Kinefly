#!/usr/bin/env python3
"""Shared utilities for Kinefly diagnostic plotting tools."""

from __future__ import annotations

import collections
import logging
import sys
from typing import Any

import numpy as np

logger = logging.getLogger(__name__)

WINDOW = 200  # Rolling window size in samples


class RollingBuffer:
    """Fixed-size rolling buffer backed by collections.deque."""

    def __init__(self, size: int = WINDOW) -> None:
        self._buf: collections.deque[float] = collections.deque([0.0] * size, maxlen=size)

    def push(self, v: float) -> None:
        self._buf.append(v)

    def array(self) -> np.ndarray:
        return np.array(self._buf)


def connect_subscriber(address: str) -> Any:
    """Connect a ZeroMQ SUB socket. Returns the socket or exits on ImportError."""
    try:
        import zmq
    except ImportError:
        logger.error("pyzmq not installed. Install with: pip install kinefly[zmq]")
        sys.exit(1)

    ctx = zmq.Context()
    sub = ctx.socket(zmq.SUB)
    sub.connect(address)
    sub.setsockopt(zmq.SUBSCRIBE, b"")
    sub.setsockopt(zmq.RCVTIMEO, 100)  # 100ms receive timeout
    return sub


def recv_state(sub: Any) -> dict | None:
    """Non-blocking receive. Returns state dict or None on timeout/error."""
    try:
        import zmq  # noqa: F401

        data = sub.recv()
    except Exception:
        return None

    try:
        import msgpack

        return msgpack.unpackb(data, raw=False)
    except Exception:
        logger.debug("Failed to deserialize msgpack frame")
        return None
