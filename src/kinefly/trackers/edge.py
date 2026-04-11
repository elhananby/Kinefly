"""Edge tracker implementations using intensity profile gradients.

Ported from nodes/tracker_edge.py. Only the IntensityProfile variants are
included; the HoughTransform classes are omitted as they are not used in
the default configuration.
"""

from __future__ import annotations

import copy
import logging

import cv2
import numpy as np

from kinefly.core.types import BodyPartState
from kinefly.gui.imagewindow import ImageWindow
from kinefly.trackers.base import MotionTrackedBodypartPolar

logger = logging.getLogger(__name__)


###############################################################################
###############################################################################
# Find the N largest gradients in the horizontal intensity profile of an image.
#
class EdgeDetectorByIntensityProfile:
    def __init__(self, threshold=0.0, n_edges_max=100, sense=1):
        self.intensities = []
        self.diff = []
        self.set_params(threshold, n_edges_max, sense)

    def set_params(self, threshold, n_edges_max, sense):
        self.threshold = threshold
        self.n_edges_max = n_edges_max
        self.sense = sense

    # detect()
    # Get the horizontal pixel position of all vertical edge pairs exceeding a magnitude threshold.
    #
    def detect(self, image):
        axis = 0
        intensitiesRaw = np.sum(image, axis).astype(np.float32)
        intensitiesRaw /= 255.0 * image.shape[axis]  # Put into range [0,1]
        self.intensities = intensitiesRaw

        # Compute the intensity gradient.
        n = 2
        a = np.append(self.intensities[n:], self.intensities[-1] * np.ones(n))
        b = np.append(self.intensities[0] * np.ones(n), self.intensities[:-n])
        self.diff = b - a

        # Make copies for positive-going and negative-going edges.
        diffP = copy.copy(self.sense * self.diff)
        diffN = copy.copy(-self.sense * self.diff)

        # Threshold the positive and negative diffs.
        iZero = np.where(diffP < self.threshold)[0]
        diffP[iZero] = 0.0

        iZero = np.where(diffN < self.threshold)[0]
        diffN[iZero] = 0.0

        # Find positive-going edges, and negative-going edges, alternately P & N.
        edgesP = []  # Positive-going edges.
        edgesN = []  # Negative-going edges.
        absP = []
        absN = []
        nCount = self.n_edges_max + (
            self.n_edges_max % 2
        )  # Round up to the next multiple of 2 so that we look at both P and N diffs.
        q = 0  # Alternate between P & N:  0=P, 1=N
        diff_list = [diffP, diffN]
        edges_list = [edgesP, edgesN]
        abs_list = [absP, absN]
        iCount = 0

        # While there are edges to detect, put them in lists in order of decending strength.
        while ((0.0 < np.max(diffP)) or (0.0 < np.max(diffN))) and (iCount < nCount):
            # If there's an edge in this diff.
            if 0.0 < np.max(diff_list[q]):
                # Append the strongest edge to the list of edges.
                iMax = np.argmax(diff_list[q])
                edges_list[q].append(iMax)
                abs_list[q].append(diff_list[q][iMax])
                iCount += 1

                # Zero all the values associated with this edge.
                for i in range(iMax - 1, -1, -1):
                    if 0.0 < diff_list[q][i]:
                        diff_list[q][i] = 0.0
                    else:
                        break
                for i in range(iMax, len(diff_list[q])):
                    if 0.0 < diff_list[q][i]:
                        diff_list[q][i] = 0.0
                    else:
                        break

            q = 1 - q  # Go to the other list.

        (edgesMajor, edgesMinor) = self.SortEdgesOneEdge(edgesP, absP, edgesN, absN)

        # Combine & sort the two lists by decreasing absolute gradient.
        edges = copy.copy(edgesMajor)
        edges.extend(edgesMinor)
        if len(edges) > 0:
            nedges = np.array(edges)
            gradients = self.diff[nedges]
            s = np.argsort(np.abs(gradients))
            edges_sorted = nedges[s[::-1]]
            gradients_sorted = gradients[s[::-1]]
        else:
            edges_sorted = []
            gradients_sorted = []

        return (edges_sorted, gradients_sorted)

    # SortEdgesOneEdge()
    # Make sure that if there's just one edge, that it's in the major list.
    #
    def SortEdgesOneEdge(self, edgesP, absP, edgesN, absN):
        # If we have too many edges, then remove the weakest one.
        lP = len(edgesP)
        lN = len(edgesN)
        if self.n_edges_max < lP + lN:
            if (0 < lP) and (0 < lN):
                if absP[-1] < absN[-1]:
                    edgesP.pop()
                    absP.pop()
                else:
                    edgesN.pop()
                    absN.pop()
            elif 0 < lP:
                edgesP.pop()
                absP.pop()
            elif 0 < lN:
                edgesN.pop()
                absN.pop()

        # Sort the edges.
        if (len(edgesP) == 0) and (len(edgesN) > 0):
            edgesMajor = edgesN
            edgesMinor = edgesP
        else:
            edgesMajor = edgesP
            edgesMinor = edgesN

        return (edgesMajor, edgesMinor)

    # SortEdgesPairwise()
    # For each pair of (p,n) edges, the stronger edge of the pair is the major one.
    #
    def SortEdgesPairwise(self, edgesP, absP, edgesN, absN):
        edgesMajor = []
        edgesMinor = []
        abs_list = [absP, absN]  # noqa: F841
        iCount = 0

        m = max(len(edgesP), len(edgesN))
        for i in range(m):
            (absP1, edgeP1) = (absP[i], edgesP[i]) if (i < len(edgesP)) else (0.0, 0)
            (absN1, edgeN1) = (absN[i], edgesN[i]) if (i < len(edgesN)) else (0.0, 0)

            if (absP1 < absN1) and (iCount < self.n_edges_max):
                edgesMajor.append(edgeN1)
                iCount += 1
                if (0.0 < absP1) and (iCount < self.n_edges_max):
                    edgesMinor.append(edgeP1)
                    iCount += 1

            elif iCount < self.n_edges_max:
                edgesMajor.append(edgeP1)
                iCount += 1
                if (0.0 < absN1) and (iCount < self.n_edges_max):
                    edgesMinor.append(edgeN1)
                    iCount += 1

        return (edgesMajor, edgesMinor)

    def SortEdgesMax(self, edgesP, absP, edgesN, absN):
        # The P or N list with the 'stronger' edge is considered to be the "major" one.
        if np.max(absN) < np.max(absP):
            edgesMajor = edgesP
            edgesMinor = edgesN
        else:
            edgesMajor = edgesN
            edgesMinor = edgesP

        return (edgesMajor, edgesMinor)


# End class EdgeDetectorByIntensityProfile


###############################################################################
###############################################################################
# EdgeTrackerByIntensityProfile()
# Track radial bodypart edges using the gradient of the image intensity.
# i.e. Compute how the intensity changes with angle, and detect the locations
# of the greatest change.  Usually used for Wings.
#
class EdgeTrackerByIntensityProfile(MotionTrackedBodypartPolar):
    def __init__(self, name=None, params={}, color="white", bEqualizeHist=False):
        MotionTrackedBodypartPolar.__init__(self, name, params, color, bEqualizeHist)

        self.name = name
        self.detector = EdgeDetectorByIntensityProfile()
        self.state = BodyPartState()
        self.windowEdges = ImageWindow(False, self.name + "Edges")
        if params:
            self.set_params(params)

    # set_params()
    # Set the given params dict into this object.
    #
    def set_params(self, params):
        MotionTrackedBodypartPolar.set_params(self, params)

        self.imgRoiBackground = None
        self.iCount = 0

        self.state.intensity = 0.0
        self.state.angles = []
        self.state.gradients = []

        # Compute the 'handedness' of the head/abdomen and wing/wing axes.
        # self.sense specifies the direction of positive angles.
        matAxes = np.array(
            [
                [
                    self.params["gui"]["head"]["hinge"]["x"]
                    - self.params["gui"]["abdomen"]["hinge"]["x"],
                    self.params["gui"]["head"]["hinge"]["y"]
                    - self.params["gui"]["abdomen"]["hinge"]["y"],
                ],
                [
                    self.params["gui"]["right"]["hinge"]["x"]
                    - self.params["gui"]["left"]["hinge"]["x"],
                    self.params["gui"]["right"]["hinge"]["y"]
                    - self.params["gui"]["left"]["hinge"]["y"],
                ],
            ]
        )
        if self.name in ["left", "right"]:
            self.senseAxes = np.sign(np.linalg.det(matAxes))
            a = -1 if (self.name == "right") else 1
            self.sense = a * self.senseAxes
        else:
            self.sense = 1

        self.detector.set_params(params[self.name]["threshold"], params["n_edges_max"], self.sense)

    # update_state()
    #
    def update_state(self):
        imgNow = self.imgRoiFgMaskedPolarCropped
        self.windowEdges.set_image(imgNow)

        # Get the rotation & expansion between images.
        if imgNow is not None:
            # Pixel position and strength of the edges.
            (edges, gradients) = self.detector.detect(imgNow)

            anglePerPixel = (
                self.params["gui"][self.name]["angle_hi"]
                - self.params["gui"][self.name]["angle_lo"]
            ) / float(imgNow.shape[1])

            # Convert pixel to angle units, and put angle into the wing frame.
            self.state.angles = []
            self.state.gradients = []
            for i in range(len(edges)):
                edge = edges[i]
                gradient = gradients[i]
                angle_b = self.params["gui"][self.name]["angle_lo"] + edge * anglePerPixel
                angle_p = (self.transform_angle_p_from_b(angle_b) + np.pi) % (2 * np.pi) - np.pi
                self.state.angles.append(angle_p)
                self.state.gradients.append(gradient)

            self.state.intensity = np.mean(imgNow) / 255.0

    # update()
    # Update all the internals given a foreground camera image.
    #
    def update(self, dt, image, bInvertColor):
        MotionTrackedBodypartPolar.update(self, dt, image, bInvertColor)

        if self.params["gui"][self.name]["track"]:
            self.update_state()

    # draw()
    # Draw the outline.
    #
    def draw(self, image):
        MotionTrackedBodypartPolar.draw(self, image)

        if self.params["gui"][self.name]["track"]:
            # Draw the major and minor edges alternately, until the max number has been reached.
            bgra = self.bgra
            for i in range(len(self.state.angles)):
                angle_b = self.transform_angle_b_from_p(self.state.angles[i])
                angle_i = self.transform_angle_i_from_b(angle_b)

                x0 = self.ptHinge_i[0] + self.params["gui"][self.name]["radius_inner"] * np.cos(
                    angle_i
                )
                y0 = self.ptHinge_i[1] + self.params["gui"][self.name]["radius_inner"] * np.sin(
                    angle_i
                )
                x1 = self.ptHinge_i[0] + self.params["gui"][self.name]["radius_outer"] * np.cos(
                    angle_i
                )
                y1 = self.ptHinge_i[1] + self.params["gui"][self.name]["radius_outer"] * np.sin(
                    angle_i
                )
                cv2.line(image, (int(x0), int(y0)), (int(x1), int(y1)), bgra, 1)
                bgra = tuple(0.5 * np.array(bgra))

            self.windowEdges.show()

    def get_diagnostics(self) -> dict:
        """Return diagnostic data for plotting tools."""
        abscissa = list(
            np.linspace(
                self.params.get("gui", {}).get(self.name, {}).get("angle_lo", 0.0),
                self.params.get("gui", {}).get(self.name, {}).get("angle_hi", 1.0),
                max(1, len(self.detector.intensities)),
            )
        )
        return {
            "color": self.color,
            "title1": "Intensities",
            "title2": "Diffs",
            "abscissa": abscissa,
            "intensities": list(self.detector.intensities),
            "diffs": list(self.detector.diff),
            "angles": list(self.state.angles),
            "gradients": list(self.state.gradients),
        }


# end class EdgeTrackerByIntensityProfile
