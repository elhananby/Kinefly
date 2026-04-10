"""Data models replacing ROS message types (MsgFlystate, MsgState)."""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class BodyPartState:
    """State of a single body part. Replaces MsgState.

    Attributes:
        angles: Angular positions in radians. May contain 0, 1, or more entries
            depending on tracker type (e.g. edge tracker finds multiple wing edges).
        gradients: Intensity gradients at detected edges. Corresponds to angles list.
        radii: Radial positions in pixels from hinge point.
        freq: Detected oscillation frequency in Hz (e.g. wingbeat frequency).
        intensity: Mean pixel intensity in the ROI, normalized to [0, 1].
    """

    angles: list[float] = field(default_factory=list)
    gradients: list[float] = field(default_factory=list)
    radii: list[float] = field(default_factory=list)
    freq: float = 0.0
    intensity: float = 0.0


@dataclass
class FlyState:
    """Aggregate state of all tracked body parts. Replaces MsgFlystate.

    Attributes:
        timestamp: Time of the frame in seconds since epoch.
        seq: Frame sequence number.
        head: Head body part state.
        abdomen: Abdomen body part state.
        left: Left wing body part state.
        right: Right wing body part state.
        aux: Auxiliary region state (intensity/wingbeat frequency).
    """

    timestamp: float
    seq: int
    head: BodyPartState = field(default_factory=BodyPartState)
    abdomen: BodyPartState = field(default_factory=BodyPartState)
    left: BodyPartState = field(default_factory=BodyPartState)
    right: BodyPartState = field(default_factory=BodyPartState)
    aux: BodyPartState = field(default_factory=BodyPartState)
