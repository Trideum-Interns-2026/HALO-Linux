"""Pure box logic for Perception: candidate decoding, shape checks, track gate, smoothing.

No ROS, no Triton: everything here is plain Python/numpy so it is unit-testable on its own.
A box is (x_min, y_min, x_max, y_max) in frame pixels; a candidate is (confidence, box).
"""

import math

import cv2
import numpy as np

PERSON_ROW = 4              # rows 0-3 = box (cx, cy, w, h), then 80 class scores; person = class 0
NMS_IOU = 0.7               # AGENTS.md: multiple people = threshold + NMS at IoU 0.7
MAX_PRE_NMS = 200           # best guesses kept before NMS; the model emits 8400 per frame
MAX_CANDIDATES = 10         # candidates returned per frame
SMOOTHER_RESET_GAP_S = 0.5  # a longer gap between updates restarts the smoother
MIN_DT_S = 1e-3             # smaller (or negative) time steps are treated as "same instant"


# ---- geometry ---------------------------------------------------------------------------------

def box_area(box):
    """Area of a box in px^2 (0 for an empty or inverted box)."""
    return max(0.0, box[2] - box[0]) * max(0.0, box[3] - box[1])


def iou(a, b):
    """Intersection over union of two boxes, 0.0 when they do not overlap."""
    iw = min(a[2], b[2]) - max(a[0], b[0])
    ih = min(a[3], b[3]) - max(a[1], b[1])
    if iw <= 0.0 or ih <= 0.0:
        return 0.0
    inter = iw * ih
    union = box_area(a) + box_area(b) - inter
    return inter / union if union > 0.0 else 0.0


def clip_box(box, fw, fh):
    """Keep a box inside a frame of fw x fh pixels."""
    return (min(max(box[0], 0.0), float(fw)), min(max(box[1], 0.0), float(fh)),
            min(max(box[2], 0.0), float(fw)), min(max(box[3], 0.0), float(fh)))


def is_plausible_box(box, fh, min_height_frac, min_aspect, max_aspect):
    """True when a box can be a person: tall enough and with a person-like height/width ratio.

    Standing people are about 2-3 times taller than wide, a running person about 1.5-2.5; tree
    canopies and bushes are usually wider or round. Limits are parameters so they can be tuned.
    Exactly at a limit still counts as plausible.
    """
    w = box[2] - box[0]
    h = box[3] - box[1]
    if w <= 0.0 or h <= 0.0 or fh <= 0:
        return False
    if h < min_height_frac * fh:
        return False
    return min_aspect <= h / w <= max_aspect


# ---- decoding ---------------------------------------------------------------------------------

def decode_candidates(raw, scale, pad_x, pad_y, fw, fh, min_confidence,
                      nms_iou=NMS_IOU, max_candidates=MAX_CANDIDATES):
    """All person boxes scoring >= min_confidence, best first, as [(conf, box), ...].

    `raw` is the model output (1, 84, 8400). Boxes are mapped back from the 640 letterbox to
    frame pixels, clipped to the frame, and overlapping ones are merged with NMS.
    """
    preds = raw[0]
    scores = preds[PERSON_ROW]
    idx = np.flatnonzero(scores >= min_confidence)
    if idx.size == 0:
        return []
    if idx.size > MAX_PRE_NMS:
        idx = idx[np.argpartition(scores[idx], -MAX_PRE_NMS)[-MAX_PRE_NMS:]]
    idx = idx[np.argsort(-scores[idx])]
    boxes, confs = [], []
    for i in idx:
        cx, cy, w, h = (float(v) for v in preds[0:4, i])
        box = clip_box(((cx - w / 2 - pad_x) / scale, (cy - h / 2 - pad_y) / scale,
                        (cx + w / 2 - pad_x) / scale, (cy + h / 2 - pad_y) / scale), fw, fh)
        boxes.append(box)
        confs.append(float(scores[i]))
    xywh = [[b[0], b[1], b[2] - b[0], b[3] - b[1]] for b in boxes]
    # scores are already >= min_confidence; NMSBoxes' own threshold is strict (>), so pass 0
    keep = cv2.dnn.NMSBoxes(xywh, confs, 0.0, float(nms_iou))
    order = sorted((int(k) for k in np.array(keep).flatten()), key=lambda k: -confs[k])
    return [(confs[k], boxes[k]) for k in order[:max_candidates]]


# ---- track gate -------------------------------------------------------------------------------

class TrackGate:
    """Decides which candidate to publish, so a tree that scores 0.6 once never gets through.

    - To START a track the best candidate must score >= start_confidence in `confirm_frames`
      frames in a row, each overlapping the previous one (IoU >= match_iou).
    - Once tracking, it keeps following the candidate that overlaps the last box, even at a
      lower confidence (>= the caller's keep threshold), instead of jumping to a louder one.
    - After more than `max_missed` frames without a match the track is dropped.
    Returns None when nothing should be published (no "valid" flag, per AGENTS.md).
    """

    def __init__(self, start_confidence, confirm_frames, max_missed, match_iou):
        self.start_confidence = float(start_confidence)
        self.confirm_frames = max(1, int(confirm_frames))
        self.max_missed = max(0, int(max_missed))
        self.match_iou = float(match_iou)
        self.reset()

    def reset(self):
        self._box = None       # last box of the current track or pending track
        self._hits = 0         # consecutive confirming frames so far
        self._missed = 0       # consecutive frames without a match while tracking
        self.confirmed = False

    def update(self, candidates):
        """Feed one frame's candidates [(conf, box)], best first; return (conf, box) or None."""
        if self.confirmed:
            match = self._best_overlap(candidates)
            if match is not None:
                self._box, self._missed = match[1], 0
                return match
            self._missed += 1
            if self._missed > self.max_missed:
                self.reset()
            return None
        strong = [c for c in candidates if c[0] >= self.start_confidence]
        if not strong:
            self.reset()
            return None
        best = strong[0]
        if self._box is not None and iou(self._box, best[1]) >= self.match_iou:
            self._hits += 1
        else:
            self._hits = 1
        self._box = best[1]
        if self._hits >= self.confirm_frames:
            self.confirmed, self._missed = True, 0
            return best
        return None

    def _best_overlap(self, candidates):
        best, best_iou = None, 0.0
        for cand in candidates:
            overlap = iou(self._box, cand[1])
            if overlap >= self.match_iou and overlap > best_iou:
                best, best_iou = cand, overlap
        return best


# ---- smoothing and extrapolation --------------------------------------------------------------

def _to_state(box):
    return [(box[0] + box[2]) / 2, (box[1] + box[3]) / 2, box[2] - box[0], box[3] - box[1]]


def _from_state(s):
    w, h = max(0.0, s[2]), max(0.0, s[3])
    return (s[0] - w / 2, s[1] - h / 2, s[0] + w / 2, s[1] + h / 2)


class BoxSmoother:
    """Alpha-beta filter on (center x, center y, width, height) with a velocity per value.

    Smooths frame-to-frame jitter without the lag a plain moving average has on a moving
    person: the velocity term carries the box along. alpha=1 returns the measurement as is.
    Time comes from the message stamps (seconds), never from the wall clock.
    """

    def __init__(self, alpha, beta, reset_gap_s=SMOOTHER_RESET_GAP_S):
        self.alpha = min(max(float(alpha), 0.0), 1.0)
        self.beta = min(max(float(beta), 0.0), 1.0)
        self.reset_gap_s = float(reset_gap_s)
        self.reset()

    def reset(self):
        self._pos = None
        self._vel = [0.0] * 4
        self._t = None

    def update(self, t_s, box):
        """Fold one measurement taken at t_s into the filter and return the smoothed box."""
        meas = _to_state(box)
        if self._pos is None or t_s - self._t > self.reset_gap_s or t_s < self._t:
            self._pos, self._vel, self._t = meas, [0.0] * 4, t_s
            return tuple(box)
        dt = max(t_s - self._t, MIN_DT_S)
        for k in range(4):
            pred = self._pos[k] + self._vel[k] * dt
            resid = meas[k] - pred
            self._pos[k] = pred + self.alpha * resid
            self._vel[k] += self.beta * resid / dt
        self._t = t_s
        return _from_state(self._pos)


def extrapolate_box(prev, last, t_s, max_horizon_s):
    """Box at time t_s from two timed boxes prev=(t, box), last=(t, box), moved at constant speed.

    Used by the overlay to draw the newest box on a newer camera image. The look-ahead is capped
    at max_horizon_s so a stale detection never flies off. With no usable `prev` (None, or the
    same instant) the last box is returned unchanged.
    """
    t_last, box_last = last
    if prev is None or not math.isfinite(t_s):
        return tuple(box_last)
    t_prev, box_prev = prev
    span = t_last - t_prev
    if span < MIN_DT_S:
        return tuple(box_last)
    ahead = min(max(t_s - t_last, 0.0), max_horizon_s)
    return tuple(c + (c - p) / span * ahead for c, p in zip(box_last, box_prev))


# ---- timing -----------------------------------------------------------------------------------

class StageTimer:
    """Collects per-stage durations and frame counts, and summarizes them as one log line."""

    def __init__(self):
        self.reset()

    def reset(self):
        self._sums, self._maxes, self._counts = {}, {}, {}
        self.frames = 0

    def record(self, name, seconds):
        self._sums[name] = self._sums.get(name, 0.0) + seconds
        self._maxes[name] = max(self._maxes.get(name, 0.0), seconds)
        self._counts[name] = self._counts.get(name, 0) + 1

    def count_frame(self):
        self.frames += 1

    def summary(self, window_s):
        """e.g. '30.1 fps | preprocess 2.1/3.0 ms, triton 12.0/20.4 ms' (mean/max per stage)."""
        fps = self.frames / window_s if window_s > 0 else 0.0
        parts = [f"{n} {self._sums[n] / self._counts[n] * 1000:.1f}/{self._maxes[n] * 1000:.1f} ms"
                 for n in sorted(self._sums)]
        return f"{fps:.1f} fps (mean/max) | " + ", ".join(parts)
