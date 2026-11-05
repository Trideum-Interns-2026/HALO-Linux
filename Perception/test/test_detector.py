"""Unit tests for the detector's pre- and post-processing math."""
import numpy as np
import pytest
from perception.yolo_person_detector import YoloPersonDetector as Det


def test_letterbox_1080p():
    """A 1920x1080 frame shrinks by 1/3 and gets 140 px grey bars top and bottom."""
    frame = np.zeros((1080, 1920, 3), dtype=np.uint8)
    blob, scale, (pad_x, pad_y) = Det._letterbox(frame)
    assert blob.shape == (1, 3, 640, 640)
    assert blob.dtype == np.float32
    assert scale == pytest.approx(1 / 3)
    assert (pad_x, pad_y) == (0, 140)
    assert blob[0, 0, 0, 0] == pytest.approx(114 / 255)   # top bar is grey padding


def test_decode_maps_box_back_to_frame():
    """A box placed in letterbox coords comes back as the original frame box."""
    raw = np.zeros((1, 84, 8400), dtype=np.float32)
    # Frame box (900,300)-(1020,600) -> letterbox center (320,290), size 40x100
    raw[0, 0:4, 123] = [320, 290, 40, 100]
    raw[0, 4, 123] = 0.9                                   # person score
    conf, x_min, y_min, x_max, y_max = Det._decode(raw, 1 / 3, 0, 140, 1920, 1080)
    assert conf == pytest.approx(0.9)
    assert (x_min, y_min, x_max, y_max) == pytest.approx((900, 300, 1020, 600))


def test_decode_no_person_returns_none():
    """All scores below 0.5 means no detection."""
    raw = np.zeros((1, 84, 8400), dtype=np.float32)
    assert Det._decode(raw, 1 / 3, 0, 140, 1920, 1080) is None
