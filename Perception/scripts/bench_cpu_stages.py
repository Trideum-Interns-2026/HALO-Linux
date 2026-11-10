#!/usr/bin/env python3
"""Time the detector's CPU stages on synthetic 1080p frames: no sim, no GPU, no Triton.

Shows how much of the per-frame budget (33 ms at 30 Hz) goes to preprocessing and decoding, so
what is left for Triton is known. Triton's own time is logged by the running node with
`-p timing_report_s:=5.0` ("triton mean/max ms").

    python3 Perception/scripts/bench_cpu_stages.py [frames]
"""
import statistics
import sys
import time

import numpy as np

from perception.box_logic import decode_candidates
from perception.yolo_person_detector import YoloPersonDetector

WIDTH, HEIGHT = 1920, 1080       # sim camera size; only used to build test frames here
BUDGET_30HZ_MS = 1000 / 30


def time_ms(fn, frames):
    samples = []
    for _ in range(frames):
        t = time.perf_counter()
        fn()
        samples.append((time.perf_counter() - t) * 1000)
    return statistics.mean(samples), max(samples)


def main():
    frames = int(sys.argv[1]) if len(sys.argv) > 1 else 100
    frame = np.random.randint(0, 255, (HEIGHT, WIDTH, 3), dtype=np.uint8)
    raw = np.random.rand(1, 84, 8400).astype(np.float32) * 0.4   # no person: all scores < 0.4
    raw[0, 4, 5] = 0.9
    raw[0, 0:4, 5] = [320, 320, 60, 150]
    _, scale, (pad_x, pad_y) = YoloPersonDetector._letterbox(frame)
    stages = {
        "letterbox": lambda: YoloPersonDetector._letterbox(frame),
        "decode": lambda: decode_candidates(raw, scale, pad_x, pad_y, WIDTH, HEIGHT, 0.5),
    }
    total = 0.0
    for name, fn in stages.items():
        mean, worst = time_ms(fn, frames)
        total += mean
        print(f"{name:10s} mean {mean:6.2f} ms   max {worst:6.2f} ms")
    print(f"CPU total  mean {total:6.2f} ms of the {BUDGET_30HZ_MS:.1f} ms budget at 30 Hz "
          f"({total / BUDGET_30HZ_MS:.0%}); the rest is for Triton")


if __name__ == "__main__":
    main()
