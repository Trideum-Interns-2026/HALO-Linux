"""Unit tests for the detector's pre- and post-processing math.

No ROS, no sim, no Triton needed: these only exercise the pure-numpy
_letterbox and _decode steps.
"""
import numpy as np
import pytest
from perception.yolo_person_detector import YoloPersonDetector as Det

# (frame width, frame height, expected scale, expected pad_x, expected pad_y)
FRAME_SIZES = [
    (1920, 1080, 1 / 3, 0, 140),    # landscape 16:9 -> grey bars top and bottom
    (1280, 720, 0.5, 0, 140),       # same shape, different size -> same bars
    (480, 640, 1.0, 80, 0),         # portrait -> grey bars on the sides
    (640, 640, 1.0, 0, 0),          # already square -> no padding at all
]
GREY = 114 / 255


def make_raw(*people):
    """Build a fake Triton output (1, 84, 8400) from (index, cx, cy, w, h, score) tuples."""
    raw = np.zeros((1, 84, 8400), dtype=np.float32)
    for i, cx, cy, w, h, score in people:
        raw[0, 0:4, i] = [cx, cy, w, h]
        raw[0, 4, i] = score
    return raw


# ---------- _letterbox ----------

@pytest.mark.parametrize("fw, fh, scale, pad_x, pad_y", FRAME_SIZES)
def test_letterbox_scale_and_padding(fw, fh, scale, pad_x, pad_y):
    """Any frame size comes out as a 640x640 blob with the right scale and padding."""
    frame = np.full((fh, fw, 3), 200, dtype=np.uint8)
    blob, got_scale, (got_pad_x, got_pad_y) = Det._letterbox(frame)
    assert blob.shape == (1, 3, 640, 640)
    assert blob.dtype == np.float32
    assert blob.flags["C_CONTIGUOUS"]                          # Triton needs contiguous memory
    assert got_scale == pytest.approx(scale)
    assert (got_pad_x, got_pad_y) == (pad_x, pad_y)
    # Image content lands just inside the padding...
    assert blob[0, 0, pad_y + 1, pad_x + 1] == pytest.approx(200 / 255)
    # ...and the padding itself (if any) is YOLO grey
    if pad_y:
        assert blob[0, 0, 0, 320] == pytest.approx(GREY)
    if pad_x:
        assert blob[0, 0, 320, 0] == pytest.approx(GREY)


def test_letterbox_flips_bgr_to_rgb():
    """A pure-blue OpenCV (BGR) frame must reach the model as blue in RGB order."""
    frame = np.zeros((640, 640, 3), dtype=np.uint8)
    frame[:, :, 0] = 255                                        # BGR: channel 0 is blue
    blob, _, _ = Det._letterbox(frame)
    assert blob[0, 0].max() == 0.0                              # R
    assert blob[0, 1].max() == 0.0                              # G
    assert blob[0, 2].min() == 1.0                              # B


# ---------- _decode ----------

@pytest.mark.parametrize("fw, fh, scale, pad_x, pad_y", FRAME_SIZES)
def test_round_trip(fw, fh, scale, pad_x, pad_y):
    """Frame box -> letterbox coords (using _letterbox's own numbers) -> _decode -> same box."""
    _, scale, (pad_x, pad_y) = Det._letterbox(np.zeros((fh, fw, 3), dtype=np.uint8))
    x1, y1, x2, y2 = 0.4 * fw, 0.2 * fh, 0.6 * fw, 0.8 * fh     # a person-ish box in the frame
    cx = (x1 + x2) / 2 * scale + pad_x
    cy = (y1 + y2) / 2 * scale + pad_y
    w = (x2 - x1) * scale
    h = (y2 - y1) * scale
    det = Det._decode(make_raw((42, cx, cy, w, h, 0.9)), scale, pad_x, pad_y, fw, fh)
    assert det is not None
    assert det[1:] == pytest.approx((x1, y1, x2, y2), abs=0.5)  # within half a pixel


def test_decode_known_box_1080p():
    """Hand-checked example: letterbox (320,290) 40x100 -> frame (900,300)-(1020,600)."""
    det = Det._decode(make_raw((123, 320, 290, 40, 100, 0.9)), 1 / 3, 0, 140, 1920, 1080)
    assert det == pytest.approx((0.9, 900, 300, 1020, 600))


def test_decode_picks_most_confident_person():
    """Two people in view -> only the higher score is returned."""
    raw = make_raw((10, 100, 300, 40, 100, 0.6),
                   (500, 320, 290, 40, 100, 0.9))
    conf, x_min, *_ = Det._decode(raw, 1 / 3, 0, 140, 1920, 1080)
    assert conf == pytest.approx(0.9)
    assert x_min == pytest.approx(900)


def test_decode_clips_box_to_frame():
    """A box hanging off the left and bottom edges is clipped to the frame."""
    # cx=10, w=100 -> x_min would be -120 px; cy=480, h=100 -> y_max would be 1170 px
    det = Det._decode(make_raw((7, 10, 480, 100, 100, 0.9)), 1 / 3, 0, 140, 1920, 1080)
    _, x_min, _, _, y_max = det
    assert x_min == 0.0
    assert y_max == 1080.0


@pytest.mark.parametrize("score, detected", [
    (0.0, False),     # empty scene
    (0.49, False),    # just under the threshold
    (0.5, True),      # exactly the threshold counts as a detection
    (0.97, True),
])
def test_decode_threshold(score, detected):
    """Only scores >= 0.5 count as a person."""
    det = Det._decode(make_raw((0, 320, 320, 50, 50, score)), 1.0, 0, 0, 640, 640)
    assert (det is not None) == detected
