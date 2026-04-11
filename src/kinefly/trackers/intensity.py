"""IntensityTracker: track pixel intensity in an auxiliary region.

Ported from nodes/tracker_intensity.py. Used for the 'aux' area to monitor
intensity and detect wingbeat frequency via undersampling.
"""

from __future__ import annotations

import logging

import numpy as np

from kinefly.core.types import BodyPartState
from kinefly.trackers.base import IntensityTrackedBodypart
from kinefly.trackers.wingbeat import WingbeatDetector

logger = logging.getLogger(__name__)


###############################################################################
###############################################################################
# The 'aux' area to track intensity.
class IntensityTracker(IntensityTrackedBodypart):
    def __init__(self, name=None, params={}, color="white", bEqualizeHist=False):
        IntensityTrackedBodypart.__init__(self, name, params, color, bEqualizeHist)

        self.state = BodyPartState()
        self.state.intensity = 0.0
        self.state.freq = 0.0

        self.wingbeat = WingbeatDetector(0, 1000)

        self.set_params(params)

    # set_params()
    # Set the given params dict into this object.
    #
    def set_params(self, params):
        IntensityTrackedBodypart.set_params(self, params)

        self.imgRoiBackground = None
        self.iCount = 0
        self.state.intensity = 0.0

        self.wingbeat.set(self.params["wingbeat_min"], self.params["wingbeat_max"])

    # update_state()
    #
    def update_state(self):
        self.state.intensity = np.sum(self.imgRoiFgMasked).astype(np.float32) / self.mask.sum
        self.state.freq = self.wingbeat.freq_from_intensity(self.state.intensity, 1.0 / self.dt)

    # update()
    # Update all the internals given a foreground camera image.
    #
    def update(self, dt, image, bInvertColor):
        IntensityTrackedBodypart.update(self, dt, image, bInvertColor)

        if self.params["gui"][self.name]["track"]:
            self.update_state()

    # draw()
    # Draw the outline.
    #
    def draw(self, image):
        IntensityTrackedBodypart.draw(self, image)


# end class IntensityTracker
