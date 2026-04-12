"""Image processing utilities: polar transforms, phase correlation, window functions.

Ported from nodes/imageprocessing.py with minimal changes:
- Removed rospy and cv (old OpenCV) imports
- Fixed Python 2 tuple unpacking in function signatures
- All algorithms are preserved exactly as-is
"""

from __future__ import annotations

import copy

import cv2
import numpy as np


class TransformException(Exception):
    pass


def get_angle_from_points_i(pt1: np.ndarray, pt2: np.ndarray) -> float:
    x = pt2[0] - pt1[0]
    y = pt2[1] - pt1[1]
    return float(np.arctan2(y, x))


def get_intersection(
    pt1a: np.ndarray, pt1b: np.ndarray, pt2a: np.ndarray, pt2b: np.ndarray
) -> np.ndarray:
    """Intersection of two lines, given two points on each line."""
    x1, y1 = pt1a[0], pt1a[1]
    x2, y2 = pt1b[0], pt1b[1]
    x3, y3 = pt2a[0], pt2a[1]
    x4, y4 = pt2b[0], pt2b[1]

    den = x1 * y3 - x3 * y1 - x1 * y4 - x2 * y3 + x3 * y2 + x4 * y1 + x2 * y4 - x4 * y2
    if den != 0.0:
        x = (
            x1 * x3 * y2
            - x2 * x3 * y1
            - x1 * x4 * y2
            + x2 * x4 * y1
            - x1 * x3 * y4
            + x1 * x4 * y3
            + x2 * x3 * y4
            - x2 * x4 * y3
        ) / den
        y = (
            x1 * y2 * y3
            - x2 * y1 * y3
            - x1 * y2 * y4
            + x2 * y1 * y4
            - x3 * y1 * y4
            + x4 * y1 * y3
            + x3 * y2 * y4
            - x4 * y2 * y3
        ) / den
    else:
        x = x3
        y = y3

    return np.array([x, y])


def get_projection_onto_axis(
    pt_anywhere: np.ndarray, pt_axis_base: np.ndarray, pt_axis_head: np.ndarray
) -> np.ndarray:
    """Project the given point onto the axis defined by two points.

    Note: Original signature used Python 2 tuple unpacking:
        def get_projection_onto_axis(ptAnywhere, (ptAxisBase, ptAxisHead))
    """
    pt_b = pt_axis_head - pt_axis_base
    pt_m = pt_anywhere - pt_axis_base
    pt_axis = np.dot(pt_b, pt_m) / np.dot(pt_b, pt_b) * pt_b + pt_axis_base
    return pt_axis


def get_reflection_across_axis(
    pt_anywhere: np.ndarray, pt_axis_base: np.ndarray, pt_axis_head: np.ndarray
) -> np.ndarray:
    """Reflect a point across the axis defined by two points.

    Note: Original signature used Python 2 tuple unpacking:
        def get_reflection_across_axis(ptAnywhere, (ptAxisBase, ptAxisHead))
    """
    pt_axis = get_projection_onto_axis(pt_anywhere, pt_axis_base, pt_axis_head)
    pt_reflected = pt_anywhere + 2 * (pt_axis - pt_anywhere)
    return pt_reflected


def filter_median(data: np.ndarray, q: int = 1) -> np.ndarray:
    """Median filter with window radius q. q=1 gives window of 3, q=2 gives window of 5."""
    data2 = copy.copy(data)
    for i in range(q, len(data) - q):
        data2[i] = np.median(data[i - q : i + q + 1])

    try:
        data2[0:q] = data2[q]
        data2[len(data2) - q : len(data2)] = data2[-(q + 1)]
        return data2
    except IndexError:
        return data


def clip(x: float, lo: float, hi: float) -> float:
    return max(min(x, hi), lo)


def clip_pt(pt: tuple[int, int], shape: tuple[int, ...]) -> tuple[int, int]:
    """Clip a point (x, y) to image shape (yMax+1, xMax+1)."""
    return (int(clip(pt[0], 0, shape[1] - 1)), int(clip(pt[1], 0, shape[0] - 1)))


class PolarTransforms:
    def __init__(self) -> None:
        self._transforms: dict = {}

    def _get_transform_polar_log(self, i_0, j_0, i_n, j_n, nRho, dRho, nTheta, theta_0, theta_1):
        transform = self._transforms.get((i_0, j_0, i_n, j_n, nRho, nTheta, theta_0, theta_1))

        if transform is None:
            i_k = []
            j_k = []
            rho_k = []
            theta_k = []

            aspect = float(i_n) / float(j_n)
            dTheta = (theta_1 - theta_0) / nTheta
            for iRho in range(0, nRho):
                rho = np.exp(iRho * dRho)

                for iTheta in range(0, nTheta):
                    theta = theta_0 + iTheta * dTheta

                    i_c = rho * np.sin(theta)
                    j_c = rho * np.cos(theta)

                    if aspect >= 1.0:
                        i = i_0 + int(i_c * aspect)
                        j = j_0 + int(j_c)
                    else:
                        i = i_0 + int(i_c)
                        j = j_0 + int(j_c / aspect)

                    if (0 <= i < i_n) and (0 <= j < j_n):
                        i_k.append(i)
                        j_k.append(j)
                        rho_k.append(iRho)
                        theta_k.append(iTheta)

            transform = (
                (np.array(rho_k), np.array(theta_k)),
                (np.array(i_k), np.array(j_k)),
            )
            self._transforms[i_0, j_0, i_n, j_n, nRho, nTheta, theta_0, theta_1] = transform

        return transform

    def transform_polar_log(
        self,
        image,
        i_0,
        j_0,
        nRho=None,
        amplifyRho=1.0,
        nTheta=None,
        amplifyTheta=1.0,
        theta_0=0.0,
        theta_1=2.0 * np.pi,
        scale=0.0,
    ):
        (i_n, j_n) = image.shape[:2]

        i_c = max(i_0, i_n - i_0)
        j_c = max(j_0, j_n - j_0)
        d_c = (i_c**2 + j_c**2) ** 0.5
        d_s = min(i_0, i_n - i_0, j_0, j_n - j_0)
        d = scale * d_c + (1.0 - scale) * d_s

        if nRho is None:
            nRho = int(np.ceil(d * amplifyRho))

        if nTheta is None:
            nTheta = int(amplifyTheta * 2 * np.pi * np.sqrt((i_n**2 + j_n**2) / 2))

        dRho = np.log(d) / nRho

        (pt, ij) = self._get_transform_polar_log(
            i_0, j_0, i_n, j_n, nRho, dRho, nTheta, theta_0, theta_1
        )
        imgTransformed = np.zeros((nRho, nTheta) + image.shape[2:], dtype=image.dtype)
        imgTransformed[pt] = image[ij]

        return imgTransformed

    def _get_transform_polar_elliptical(
        self,
        i_0,
        j_0,
        i_n,
        j_n,
        r_axial_ortho,
        drStrip,
        angleEllipse,
        nRho,
        nTheta,
        theta_0,
        theta_1,
        rClip,
    ):
        (raxial, rortho) = r_axial_ortho
        nTheta = max(1, nTheta)
        transform = self._transforms.get(
            (i_0, j_0, i_n, j_n, nRho, drStrip, nTheta, theta_0, theta_1, rClip)
        )

        if transform is None:
            i_k = []
            j_k = []
            rho_k = []
            theta_k = []

            raxial_outer = raxial + drStrip
            rortho_outer = raxial + drStrip
            raxial_inner = raxial - drStrip
            rortho_inner = raxial - drStrip

            R = np.array(
                [
                    [np.cos(-angleEllipse), -np.sin(-angleEllipse)],
                    [np.sin(-angleEllipse), np.cos(-angleEllipse)],
                ]
            )
            dTheta = (theta_1 - theta_0) / nTheta

            for iTheta in range(0, nTheta):
                theta = theta_0 + iTheta * dTheta

                xy_e = np.array([raxial * np.cos(theta), rortho * np.sin(theta)])
                theta_e = np.arctan2(xy_e[1], xy_e[0])

                rho_e_inner = np.linalg.norm(
                    [raxial_inner * np.cos(theta), rortho_inner * np.sin(theta)]
                )
                rho_e_outer = np.linalg.norm(
                    [raxial_outer * np.cos(theta), rortho_outer * np.sin(theta)]
                )
                dRho = (rho_e_outer - rho_e_inner) / nRho

                for iRho in range(0, int(np.ceil(rClip * nRho))):
                    rho = rho_e_inner + iRho * dRho

                    i_e = rho * np.sin(theta_e)
                    j_e = rho * np.cos(theta_e)

                    ij = R.dot([i_e, j_e])

                    i = int(i_0 + ij[0])
                    j = int(j_0 + ij[1])

                    if (0 <= i < i_n) and (0 <= j < j_n):
                        i_k.append(i)
                        j_k.append(j)
                        rho_k.append(iRho)
                        theta_k.append(iTheta)

            transform = (
                (np.array(rho_k), np.array(theta_k)),
                (np.array(i_k), np.array(j_k)),
            )
            self._transforms[i_0, j_0, i_n, j_n, nRho, drStrip, nTheta, theta_0, theta_1, rClip] = (
                transform
            )

        return transform

    def transform_polar_elliptical(
        self,
        image,
        i_0,
        j_0,
        raxial=None,
        rortho=None,
        dradiusStrip=None,
        nRho=None,
        amplifyRho=1.0,
        rClip=0.0,
        angleEllipse=0.0,
        theta_0=-np.pi,
        theta_1=np.pi,
        nTheta=None,
        amplifyTheta=1.0,
    ):
        (i_n, j_n) = image.shape[:2]
        if raxial is None:
            raxial = i_n / 2
        if rortho is None:
            rortho = j_n / 2

        if dradiusStrip is None:
            dradiusStrip = raxial - 5

        raxial_outer = raxial + dradiusStrip
        rortho_outer = raxial + dradiusStrip
        raxial_inner = raxial - dradiusStrip
        rortho_inner = raxial - dradiusStrip

        d_e_raxial_outer = raxial_outer
        d_e_rortho_outer = rortho_outer
        d_e_wedge_outer = np.linalg.norm(
            [raxial_outer * np.cos(theta_0), rortho_outer * np.sin(theta_0)]
        )
        if np.abs(theta_0) >= np.pi / 2.0:
            d_e_min_outer = min(d_e_raxial_outer, d_e_wedge_outer, d_e_rortho_outer)
        else:
            d_e_min_outer = min(d_e_raxial_outer, d_e_wedge_outer)

        d_e_raxial_inner = raxial_inner
        d_e_rortho_inner = rortho_inner
        d_e_wedge_inner = np.linalg.norm(
            [raxial_inner * np.cos(theta_0), rortho_inner * np.sin(theta_0)]
        )
        if np.abs(theta_0) >= np.pi / 2.0:
            d_e_min_inner = min(d_e_raxial_inner, d_e_wedge_inner, d_e_rortho_inner)
        else:
            d_e_min_inner = min(d_e_raxial_inner, d_e_wedge_inner)

        d = d_e_min_outer - d_e_min_inner

        xy_e = np.array([raxial * np.cos(theta_0), rortho * np.sin(theta_0)])
        theta_0e = np.arctan2(xy_e[1], xy_e[0])
        xy_e = np.array([raxial * np.cos(theta_1), rortho * np.sin(theta_1)])
        theta_1e = np.arctan2(xy_e[1], xy_e[0])

        if nRho is None:
            nRho = int(np.ceil(d * amplifyRho))

        if nTheta is None:
            circumference = 2 * np.pi * np.sqrt(raxial**2 + rortho**2)
            fraction_wedge = np.abs(theta_1e - theta_0e) / (2 * np.pi)
            nTheta = int(amplifyTheta * circumference * fraction_wedge)

        (pt, ij) = self._get_transform_polar_elliptical(
            i_0,
            j_0,
            i_n,
            j_n,
            (raxial, rortho),
            dradiusStrip,
            angleEllipse,
            nRho,
            nTheta,
            theta_0,
            theta_1,
            rClip,
        )
        imgTransformed = np.zeros((nRho, nTheta) + image.shape[2:], dtype=image.dtype)
        if len(pt[0]) > 0:
            imgTransformed[pt] = image[ij]
        else:
            raise TransformException()

        return imgTransformed


class PhaseCorrelation:
    """Compute the coordinate shift between two images via FFT phase correlation."""

    def get_shift(self, imgA: np.ndarray, imgB: np.ndarray) -> np.ndarray:
        rv = np.array([0.0, 0.0])
        if imgA is not None and imgB is not None and imgA.shape == imgB.shape:
            A = cv2.dft(imgA)
            B = cv2.dft(imgB)
            AB = cv2.mulSpectrums(A, B, flags=0, conjB=True)
            normAB = cv2.norm(AB)
            if normAB != 0.0:
                crosspower = AB / normAB
                shift = cv2.idft(crosspower)
                shift0 = np.roll(shift, int(shift.shape[0] / 2), 0)
                shift00 = np.roll(shift0, int(shift.shape[1] / 2), 1)

                kShift = np.argmax(shift00)
                (iShift, jShift) = np.unravel_index(kShift, shift00.shape)

                w = 7
                r = int((w - 1) / 2)
                i0 = int(clip(iShift - r, 0, shift00.shape[0] - 1))
                i1 = int(clip(iShift + r, 0, shift00.shape[0] - 1)) + 1
                j0 = int(clip(jShift - r, 0, shift00.shape[1] - 1))
                j1 = int(clip(jShift + r, 0, shift00.shape[1] - 1)) + 1
                peak = shift00[i0:i1].T[j0:j1].T
                moments = cv2.moments(peak, binaryImage=False)

                if moments["m00"] != 0.0:
                    iShiftSubpixel = moments["m01"] / moments["m00"] + float(i0)
                    jShiftSubpixel = moments["m10"] / moments["m00"] + float(j0)
                else:
                    iShiftSubpixel = float(shift.shape[0]) / 2.0
                    jShiftSubpixel = float(shift.shape[1]) / 2.0

                iShiftSubpixel -= float(shift.shape[0]) / 2.0
                jShiftSubpixel -= float(shift.shape[1]) / 2.0

                height = float(shift00.shape[0])
                width = float(shift00.shape[1])
                iShiftSubpixel = ((iShiftSubpixel + height / 2.0) % height) - height / 2.0
                jShiftSubpixel = ((jShiftSubpixel + width / 2.0) % width) - width / 2.0

                rv = np.array([iShiftSubpixel, jShiftSubpixel])

        return rv


class WindowFunctions:
    """Create 2D window functions for image processing."""

    def create_hanning(self, shape: tuple[int, int]) -> np.ndarray:
        (height, width) = shape
        wfn = np.ones(shape, dtype=np.float32)
        if height > 1 and width > 1:
            for i in range(width):
                for j in range(height):
                    x = 2 * np.pi * i / (width - 1)
                    y = 2 * np.pi * j / (height - 1)
                    wfn[j][i] = 0.5 * (1 - np.cos(x)) * 0.5 * (1 - np.cos(y))
        return wfn

    def create_tukey(self, shape: tuple[int, int], feathering: float = 0.25) -> np.ndarray:
        (height, width) = shape
        alpha = feathering if feathering > 0 else 0.25
        wfn = np.ones(shape, dtype=np.float32)
        if height > 1 and width > 1:
            for i in range(width):
                for j in range(height):
                    y = np.pi * (2 * j / (alpha * (height - 1)) - 1)

                    if 0 <= i <= (alpha * (width - 1)) / 2:
                        x = np.pi * (2 * i / (alpha * (width - 1)) - 1)
                    elif (alpha * (width - 1)) / 2 < i <= (width - 1) * (1 - alpha / 2):
                        x = 0.0
                    elif (width - 1) * (1 - alpha / 2) < i <= width - 1:
                        x = np.pi * (2 * i / (alpha * (width - 1)) - 2 / alpha + 1)

                    if 0 <= j <= (alpha * (height - 1)) / 2:
                        y = np.pi * (2 * j / (alpha * (height - 1)) - 1)
                    elif (alpha * (height - 1)) / 2 < j <= (height - 1) * (1 - alpha / 2):
                        y = 0.0
                    elif (height - 1) * (1 - alpha / 2) < j <= height - 1:
                        y = np.pi * (2 * j / (alpha * (height - 1)) - 2 / alpha + 1)

                    wfnx = 0.5 * (1 + np.cos(x))
                    wfny = 0.5 * (1 + np.cos(y))
                    wfn[j][i] = wfnx * wfny
        return wfn
