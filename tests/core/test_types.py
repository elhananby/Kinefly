from kinefly.core.types import BodyPartState, FlyState


def test_bodypart_state_defaults():
    s = BodyPartState()
    assert s.angles == []
    assert s.gradients == []
    assert s.radii == []
    assert s.freq == 0.0
    assert s.intensity == 0.0


def test_bodypart_state_with_values():
    s = BodyPartState(angles=[0.5, 1.0], gradients=[-0.01], radii=[54.6], freq=200.0, intensity=0.45)
    assert s.angles == [0.5, 1.0]
    assert s.intensity == 0.45


def test_flystate_creation():
    state = FlyState(
        timestamp=1234567890.123,
        seq=42,
        head=BodyPartState(angles=[-0.05]),
        abdomen=BodyPartState(angles=[-0.37]),
        left=BodyPartState(angles=[0.82], gradients=[-0.008]),
        right=BodyPartState(angles=[0.60], gradients=[0.012]),
        aux=BodyPartState(intensity=0.67),
    )
    assert state.seq == 42
    assert state.left.angles[0] == 0.82
    assert state.aux.intensity == 0.67


def test_flystate_field_access_matches_ros_pattern():
    """Verify that field access patterns used throughout the old codebase still work.
    e.g. flystate.left.angles[0], flystate.head.radii[0], flystate.aux.intensity
    """
    state = FlyState(
        timestamp=0.0,
        seq=0,
        head=BodyPartState(angles=[0.1], radii=[50.0]),
        abdomen=BodyPartState(angles=[0.2], radii=[100.0]),
        left=BodyPartState(angles=[0.8, 0.3], gradients=[-0.01, 0.02]),
        right=BodyPartState(angles=[0.6], gradients=[0.01]),
        aux=BodyPartState(intensity=0.5, freq=195.0),
    )
    assert len(state.left.angles) == 2
    angle1_left = state.left.angles[0] if 0 < len(state.left.angles) else 0.0
    assert angle1_left == 0.8
    angle_head = state.head.angles[0] if 0 < len(state.head.angles) else 0.0
    assert angle_head == 0.1
    assert state.aux.intensity == 0.5
