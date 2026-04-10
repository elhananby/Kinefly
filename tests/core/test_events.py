from kinefly.core.events import EventBus
from kinefly.core.types import FlyState


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
