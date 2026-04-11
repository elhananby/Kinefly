import numpy as np

from kinefly.fly import Fly
from kinefly.core.types import FlyState


def test_fly_init_default():
    """Fly instantiates with empty params without crashing."""
    fly = Fly(params={})
    assert fly is not None


def test_fly_get_flystate_returns_flystate():
    """get_flystate() returns a FlyState object."""
    fly = Fly(params={})
    state = fly.get_flystate()
    assert isinstance(state, FlyState)
    assert state.seq == 0


def test_fly_get_flystate_increments_seq():
    """Each call to update() increments the sequence counter."""
    fly = Fly(params={})
    img = np.zeros((100, 100), dtype=np.uint8)
    fly.update(img, timestamp=0.1)
    fly.update(img, timestamp=0.2)
    state = fly.get_flystate()
    assert state.seq == 2
