"""
Detector slot: image -> target pixel. Any detector with this call works (plug in
your own, e.g. a neural network):

    detect(image) -> Detection | None

Two simple ones:
  CentreDetector   always the image centre: the bearing is then the camera axis
                   (checks the attitude chain: it must follow the heading).
  ColourDetector   the largest blob of one colour, picked by clicking on the
                   object in the live window. A still object should keep the
                   same world bearing while the drone turns.
"""
import math
from dataclasses import dataclass

import cv2
import numpy as np


@dataclass
class Detection:
    u: float            # pixel, in the image given to detect()
    v: float
    conf: float
    box: tuple | None = None    # x, y, w, h


class CentreDetector:
    def __call__(self, image):
        h, w = image.shape[:2]
        return Detection(w / 2, h / 2, 1.0)


class ColourDetector:
    """
    Tracks one coloured object, picked by clicking on it.

    Colour: hue within hue_tol of the sample, and saturation / brightness within
    sat_tol / val_tol of it (a dark blue shirt and the bright blue sky share a
    hue; flight 2026-10-31 locked onto the sky). Blobs larger than max_frac of
    the searched image are ignored (sky, lawn).
    Object: the blob under the click starts the track; after that the blob
    nearest to the last position with a similar size wins, not the largest.
    After lost_after frames without it, any matching blob may start a new track.
    The DJI Fly overlays (top bar, record button on the right edge, joystick
    circles) sit at the edges, so only the inner part of the image is searched.
    """

    def __init__(self, hue_tol=10, sat_tol=70, val_tol=70, min_sat=60, min_area=30, max_frac=0.25,
                 gate=0.15, lost_after=15, margin=(0.12, 0.06)):
        self.hsv = None
        self.hue_tol, self.sat_tol, self.val_tol, self.min_sat = hue_tol, sat_tol, val_tol, min_sat
        self.min_area, self.max_frac = min_area, max_frac
        self.gate = gate                # fraction of the image width the target may jump between frames
        self.lost_after = lost_after
        self.margin = margin            # (top/bottom, left/right) fractions ignored
        self.track = None               # (u, v, area) of the target in the last frame it was found
        self.missed = 0

    def _blobs(self, image):
        """Matching blobs as (u, v, area, box) in full-image pixels."""
        h, w = image.shape[:2]
        my, mx = int(h * self.margin[0]), int(w * self.margin[1])
        hsv = cv2.cvtColor(image[my:h - my, mx:w - mx], cv2.COLOR_BGR2HSV).astype(int)
        dh = np.abs(hsv[..., 0] - int(self.hsv[0]))
        dh = np.minimum(dh, 180 - dh)       # hue wraps around (red sits at both ends)
        mask = ((dh <= self.hue_tol) & (np.abs(hsv[..., 1] - self.hsv[1]) <= self.sat_tol)
                & (np.abs(hsv[..., 2] - self.hsv[2]) <= self.val_tol) & (hsv[..., 1] >= self.min_sat))
        mask = cv2.morphologyEx(mask.astype(np.uint8), cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        n, _, stats, centroids = cv2.connectedComponentsWithStats(mask)
        too_big = self.max_frac * mask.size
        return [(centroids[k][0] + mx, centroids[k][1] + my, int(stats[k, cv2.CC_STAT_AREA]),
                 (int(stats[k, 0] + mx), int(stats[k, 1] + my), int(stats[k, 2]), int(stats[k, 3])))
                for k in range(1, n) if self.min_area <= stats[k, cv2.CC_STAT_AREA] <= too_big]

    def pick(self, image, u, v):
        """Take the target colour from a 5x5 patch around pixel (u, v); the blob there starts the track."""
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        patch = hsv[max(v - 2, 0):v + 3, max(u - 2, 0):u + 3].reshape(-1, 3)
        self.hsv = np.median(patch, axis=0)
        blobs = self._blobs(image)
        near = min(blobs, key=lambda b: (b[0] - u) ** 2 + (b[1] - v) ** 2, default=None)
        self.track = (near[0], near[1], near[2]) if near else None
        self.missed = 0
        return self.hsv

    def __call__(self, image):
        if self.hsv is None:
            return None
        blobs = self._blobs(image)
        best = None
        if self.track is not None:
            u0, v0, a0 = self.track
            gate = self.gate * image.shape[1]
            ok = [b for b in blobs if math.hypot(b[0] - u0, b[1] - v0) <= gate and 0.25 <= b[2] / a0 <= 4]
            best = min(ok, key=lambda b: math.hypot(b[0] - u0, b[1] - v0), default=None)
            if best is None:
                self.missed += 1
                if self.missed <= self.lost_after:
                    return None
                self.track = None           # lost for good: any matching blob may start a new track
        if best is None:
            best = max(blobs, key=lambda b: b[2], default=None)
            if best is None:
                return None
        self.track, self.missed = (best[0], best[1], best[2]), 0
        return Detection(best[0], best[1], min(1.0, best[2] / 500), best[3])
