"""Logic tests for perception.box_logic: no ROS, no Triton."""
import numpy as np
import pytest

from perception.box_logic import (
    BoxSmoother,
    StageTimer,
    TrackGate,
    box_area,
    clip_box,
    decode_candidates,
    extrapolate_box,
    iou,
    is_plausible_box,
)

# ---------- geometry ----------


@pytest.mark.parametrize("a, b, expected", [
    ((0, 0, 10, 10), (0, 0, 10, 10), 1.0),        # identical
    ((0, 0, 10, 10), (20, 20, 30, 30), 0.0),      # apart
    ((0, 0, 10, 10), (10, 0, 20, 10), 0.0),       # touching edge only
    ((0, 0, 10, 10), (5, 0, 15, 10), 50 / 150),   # half overlap
    ((0, 0, 0, 0), (0, 0, 0, 0), 0.0),            # empty boxes
])
def test_iou(a, b, expected):
    assert iou(a, b) == pytest.approx(expected)


def test_box_area_and_clip():
    assert box_area((0, 0, 4, 5)) == 20
    assert box_area((4, 5, 0, 0)) == 0                   # inverted
    assert clip_box((-5, -5, 2000, 1100), 1920, 1080) == (0.0, 0.0, 1920.0, 1080.0)


@pytest.mark.parametrize("box, ok", [
    ((0, 0, 100, 250), True),        # standing person, 2.5
    ((0, 0, 100, 120), True),        # exactly min_aspect 1.2 counts
    ((0, 0, 100, 119), False),       # just under: squat, tree-like
    ((0, 0, 100, 500), True),        # exactly max_aspect 5 counts
    ((0, 0, 100, 501), False),       # a pole
    ((0, 0, 0, 100), False),         # zero width
])
def test_is_plausible_box(box, ok):
    assert is_plausible_box(box, 1080, 0.03, 1.2, 5.0) == ok


def test_is_plausible_box_min_height():
    assert not is_plausible_box((0, 0, 10, 32.3), 1080, 0.03, 1.2, 5.0)   # a hair too small
    assert is_plausible_box((0, 0, 10, 32.4), 1080, 0.03, 1.2, 5.0)
    assert not is_plausible_box((0, 0, 10, 30), 0, 0.03, 1.2, 5.0)        # bad frame height


# ---------- decode ----------


def make_raw(*guesses):
    """guesses: (index, cx, cy, w, h, score) in 640-letterbox pixels."""
    raw = np.zeros((1, 84, 8400), dtype=np.float32)
    for i, cx, cy, w, h, score in guesses:
        raw[0, 0:4, i] = [cx, cy, w, h]
        raw[0, 4, i] = score
    return raw


def test_decode_candidates_sorted_best_first_and_nms_merges_duplicates():
    raw = make_raw((1, 100, 100, 40, 100, 0.9),
                   (2, 102, 100, 40, 100, 0.8),        # same person, slightly shifted -> merged
                   (3, 400, 300, 60, 120, 0.6))        # another object
    cands = decode_candidates(raw, 1.0, 0, 0, 640, 640, 0.5)
    assert [round(c[0], 2) for c in cands] == [0.9, 0.6]


def test_decode_candidates_threshold_is_inclusive_and_empty_is_empty():
    raw = make_raw((1, 100, 100, 40, 100, 0.5))
    assert len(decode_candidates(raw, 1.0, 0, 0, 640, 640, 0.5)) == 1
    assert decode_candidates(raw, 1.0, 0, 0, 640, 640, 0.51) == []
    assert decode_candidates(make_raw(), 1.0, 0, 0, 640, 640, 0.1) == []


def test_decode_candidates_maps_back_to_frame_pixels():
    # 1920x1080 -> scale 1/3, pad_y 140; box centre (320, 290) size 40x100 in the letterbox
    cands = decode_candidates(make_raw((5, 320, 290, 40, 100, 0.9)),
                              1 / 3, 0, 140, 1920, 1080, 0.5)
    assert cands[0][1] == pytest.approx((900, 300, 1020, 600))


def test_decode_candidates_caps_the_count():
    raw = make_raw(*[(i, 20 + 30 * (i % 20), 20 + 50 * (i // 20), 10, 20, 0.9) for i in range(60)])
    assert len(decode_candidates(raw, 1.0, 0, 0, 640, 640, 0.5, max_candidates=4)) == 4


def test_decode_candidates_many_guesses_stay_fast_and_correct():
    raw = np.zeros((1, 84, 8400), dtype=np.float32)
    raw[0, 4, :] = 0.6                                  # 8400 guesses over the threshold
    raw[0, 0:4, :] = np.array([[300], [300], [50], [120]])
    raw[0, 4, 77] = 0.95
    cands = decode_candidates(raw, 1.0, 0, 0, 640, 640, 0.5)
    assert cands[0][0] == pytest.approx(0.95)


# ---------- track gate ----------

A = (100.0, 100.0, 200.0, 350.0)
A_NEAR = (105.0, 100.0, 205.0, 350.0)
FAR = (800.0, 100.0, 900.0, 350.0)


def gate(confirm=3, missed=2):
    return TrackGate(start_confidence=0.7, confirm_frames=confirm, max_missed=missed,
                     match_iou=0.2)


def test_gate_needs_consecutive_strong_overlapping_frames():
    g = gate()
    assert g.update([(0.9, A)]) is None
    assert g.update([(0.9, A_NEAR)]) is None
    assert g.update([(0.9, A)]) == (0.9, A)
    assert g.confirmed


def test_gate_weak_or_jumping_candidates_never_confirm():
    g = gate()
    assert g.update([(0.6, A)]) is None                 # a tree at 0.6 is below the start score
    assert g.update([(0.6, A)]) is None
    assert g.update([(0.6, A)]) is None
    assert not g.confirmed
    g = gate()
    g.update([(0.9, A)])
    g.update([(0.9, FAR)])                              # jumped: the count restarts
    assert g.update([(0.9, A)]) is None
    assert not g.confirmed


def test_gate_a_gap_before_confirming_resets_the_count():
    g = gate()
    g.update([(0.9, A)])
    g.update([(0.9, A)])
    assert g.update([]) is None
    assert g.update([(0.9, A)]) is None                 # starts again at 1 of 3


def test_gate_confirm_frames_one_publishes_at_once():
    assert gate(confirm=1).update([(0.9, A)]) == (0.9, A)


def test_gate_follows_the_overlapping_box_at_lower_confidence_not_a_louder_one():
    g = gate(confirm=1)
    g.update([(0.9, A)])
    # a louder box elsewhere and the person at a low score: stay on the person
    assert g.update([(0.95, FAR), (0.55, A_NEAR)]) == (0.55, A_NEAR)


def test_gate_drops_after_too_many_misses_and_publishes_nothing_meanwhile():
    g = gate(confirm=1, missed=2)
    g.update([(0.9, A)])
    assert g.update([]) is None and g.confirmed         # miss 1
    assert g.update([(0.9, FAR)]) is None and g.confirmed   # miss 2: no overlap
    assert g.update([]) is None and not g.confirmed     # miss 3 > max_missed -> dropped
    assert g.update([(0.9, FAR)]) == (0.9, FAR)         # re-acquires normally


def test_gate_recovers_when_the_person_reappears_within_the_limit():
    g = gate(confirm=1, missed=2)
    g.update([(0.9, A)])
    g.update([])
    assert g.update([(0.6, A_NEAR)]) == (0.6, A_NEAR)


# ---------- smoother ----------


def test_smoother_first_measurement_passes_through():
    assert BoxSmoother(0.5, 0.2).update(0.0, A) == A


def test_smoother_alpha_one_beta_zero_is_the_measurement():
    s = BoxSmoother(1.0, 0.0)
    s.update(0.0, A)
    assert s.update(0.1, A_NEAR) == pytest.approx(A_NEAR)


def test_smoother_reduces_jitter_on_a_still_box():
    s = BoxSmoother(0.5, 0.1)
    s.update(0.0, A)
    out = s.update(0.033, (A[0] + 10, A[1], A[2] + 10, A[3]))   # 10 px jitter
    assert A[0] < out[0] < A[0] + 10


def test_smoother_tracks_constant_motion_without_steady_lag():
    s = BoxSmoother(0.7, 0.3)
    box = A
    for k in range(60):                                           # 30 Hz, 300 px/s to the right
        box = (A[0] + 10 * k, A[1], A[2] + 10 * k, A[3])
        out = s.update(k / 30, box)
    assert out[0] == pytest.approx(box[0], abs=1.0)


@pytest.mark.parametrize("t2", [0.6, 5.0, -1.0])                  # long gap, or time going back
def test_smoother_restarts_on_a_gap_or_backwards_time(t2):
    s = BoxSmoother(0.5, 0.2)
    s.update(0.0, A)
    assert s.update(t2, FAR) == FAR


def test_smoother_reset():
    s = BoxSmoother(0.5, 0.2)
    s.update(0.0, A)
    s.reset()
    assert s.update(0.01, FAR) == FAR


# ---------- extrapolation ----------


def test_extrapolate_moves_at_constant_velocity():
    prev, last = (0.0, (0, 0, 10, 10)), (1.0, (10, 0, 20, 10))      # 10 px/s right
    assert extrapolate_box(prev, last, 1.05, 0.5) == pytest.approx((10.5, 0, 20.5, 10))


def test_extrapolate_is_capped_and_never_goes_backwards():
    prev, last = (0.0, (0, 0, 10, 10)), (1.0, (10, 0, 20, 10))
    assert extrapolate_box(prev, last, 9.0, 0.2) == pytest.approx((12, 0, 22, 10))   # cap 0.2 s
    assert extrapolate_box(prev, last, 0.5, 0.2) == pytest.approx((10, 0, 20, 10))   # older image


@pytest.mark.parametrize("prev", [None, (1.0, (0, 0, 5, 5))])    # no history / same instant
def test_extrapolate_without_usable_history_returns_the_last_box(prev):
    assert extrapolate_box(prev, (1.0, (10, 0, 20, 10)), 1.1, 0.5) == (10, 0, 20, 10)


def test_extrapolate_non_finite_time_returns_the_last_box():
    prev, last = (0.0, (0, 0, 10, 10)), (1.0, (10, 0, 20, 10))
    assert extrapolate_box(prev, last, float("nan"), 0.5) == (10, 0, 20, 10)


# ---------- timer ----------


def test_stage_timer_summary_and_reset():
    t = StageTimer()
    t.record("triton", 0.010)
    t.record("triton", 0.030)
    t.count_frame()
    t.count_frame()
    text = t.summary(window_s=1.0)
    assert "2.0 fps" in text and "triton 20.0/30.0 ms" in text
    t.reset()
    assert t.frames == 0 and t.summary(1.0).startswith("0.0 fps")
    assert t.summary(0.0).startswith("0.0 fps")                      # no divide by zero
