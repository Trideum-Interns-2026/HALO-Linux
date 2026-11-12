"""YOLO person detector node.

Subscribes to the drone's camera, sends each frame to the shared Triton
YOLO server, and publishes the most confident person as a
perception_msgs/Detection.

Topics
    in   /camera/front/image_raw   sensor_msgs/Image (reliable, depth 1; see image_reliability)
    out  /perception/detection     perception_msgs/Detection (only when a person is seen)

Parameters (all optional, see declare_parameter below for defaults)
    triton_url, model_name, min_confidence, keep_confidence, confirm_frames, max_missed_frames,
    match_iou, min_box_height_frac, min_aspect, max_aspect, smooth_box, smoothing_alpha,
    smoothing_beta, async_inference, timing_report_s, triton_timeout_s, image_topic,
    detection_topic, image_reliability

Frames are processed on a worker thread that only ever keeps the newest one, so a slow Triton
call drops frames instead of queuing them (steady rate, no growing lag).
"""

import os
import threading
import time

import cv2
import numpy as np
import rclpy
import tritonclient.grpc as grpcclient
from cv_bridge import CvBridge
from perception_msgs.msg import Detection
from rclpy.node import Node
from sensor_msgs.msg import Image
from tritonclient.utils import InferenceServerException

from perception.box_logic import (
    BoxSmoother, StageTimer, TrackGate, clip_box, decode_candidates, is_plausible_box)
from perception.image_qos import IMAGE_RELIABILITY, image_qos

INPUT_SIZE = 640        # fixed by config.pbtxt: dims [3, 640, 640]
MIN_CONFIDENCE = 0.7    # score needed to START a track (trees scored ~0.6 at 0.5)
KEEP_CONFIDENCE = 0.5   # score that keeps an already confirmed track alive
CONFIRM_FRAMES = 3      # consecutive overlapping frames before the first Detection
MAX_MISSED_FRAMES = 5   # frames without a match before the track is dropped
MATCH_IOU = 0.2         # overlap with the previous box that counts as "the same person"
MIN_BOX_HEIGHT_FRAC = 0.03   # box height / frame height below this is not a person
MIN_ASPECT = 1.2        # box height / width: a person is taller than wide
MAX_ASPECT = 6.0
SMOOTHING_ALPHA = 0.7   # position gain of the box smoother (1 = no smoothing)
SMOOTHING_BETA = 0.3    # velocity gain
IMAGE_TOPIC = "/camera/front/image_raw"        # default input (sim camera bridge)
DETECTION_TOPIC = "/perception/detection"      # default output


class YoloPersonDetector(Node):
    """ROS2 node: camera frames in, person detections out."""

    def __init__(self, **kwargs):
        super().__init__("yolo_person_detector", **kwargs)

        # Settings come from ROS parameters so they can change per launch
        # without editing code. Triton's URL defaults to the container's env var.
        self.triton_url = self.declare_parameter(
            "triton_url", os.environ.get("HALO_TRITON_URL", "triton:8001")).value
        self.model_name = self.declare_parameter("model_name", "yolo_person").value
        self.min_confidence = self.declare_parameter("min_confidence", MIN_CONFIDENCE).value
        self.keep_confidence = min(
            self.declare_parameter("keep_confidence", KEEP_CONFIDENCE).value, self.min_confidence)
        confirm_frames = self.declare_parameter("confirm_frames", CONFIRM_FRAMES).value
        max_missed = self.declare_parameter("max_missed_frames", MAX_MISSED_FRAMES).value
        match_iou = self.declare_parameter("match_iou", MATCH_IOU).value
        self.min_box_height_frac = self.declare_parameter(
            "min_box_height_frac", MIN_BOX_HEIGHT_FRAC).value
        self.min_aspect = self.declare_parameter("min_aspect", MIN_ASPECT).value
        self.max_aspect = self.declare_parameter("max_aspect", MAX_ASPECT).value
        self.smooth_box = self.declare_parameter("smooth_box", True).value
        alpha = self.declare_parameter("smoothing_alpha", SMOOTHING_ALPHA).value
        beta = self.declare_parameter("smoothing_beta", SMOOTHING_BETA).value
        self.async_inference = self.declare_parameter("async_inference", True).value
        # 0 = no timing log; otherwise one summary line every this many seconds
        self.timing_report_s = self.declare_parameter("timing_report_s", 0.0).value
        self.triton_timeout_s = self.declare_parameter("triton_timeout_s", 0.1).value
        # Topic names are parameters so the same node runs on a real camera (SITL values here).
        self.image_topic = self.declare_parameter("image_topic", IMAGE_TOPIC).value
        self.detection_topic = self.declare_parameter("detection_topic", DETECTION_TOPIC).value
        self.image_reliability = self.declare_parameter(
            "image_reliability", IMAGE_RELIABILITY).value

        # Converts ROS Image <-> numpy; made once, reused every frame
        self.bridge = CvBridge()
        # gRPC client for the shared Triton server (connects on first infer)
        self.triton = grpcclient.InferenceServerClient(url=self.triton_url)

        self.gate = TrackGate(self.min_confidence, confirm_frames, max_missed, match_iou)
        self.smoother = BoxSmoother(alpha, beta)
        self.timer = StageTimer()
        self._timer_start = time.monotonic()

        # Latest-frame-only hand-off to the worker thread (see _worker)
        self._cond = threading.Condition()
        self._pending = None
        self._stop = False
        self._worker_thread = None
        if self.async_inference:
            self._worker_thread = threading.Thread(target=self._worker, daemon=True)
            self._worker_thread.start()

        # Input: camera frames, depth 1 so a slow frame never queues up behind older ones.
        callback = self.on_image_async if self.async_inference else self.on_image
        self.sub = self.create_subscription(
            Image, self.image_topic, callback, self.image_qos(self.image_reliability))
        # Output: one Detection per frame that contains a person
        self.pub = self.create_publisher(Detection, self.detection_topic, 10)

        self.get_logger().info(
            f"Ready: triton={self.triton_url} model={self.model_name} "
            f"min_confidence={self.min_confidence} keep={self.keep_confidence} "
            f"confirm={confirm_frames} smooth={self.smooth_box} async={self.async_inference} "
            f"timeout={self.triton_timeout_s}s "
            f"image={self.image_topic}({self.image_reliability})")

    # Kept as a method so callers and tests can still use YoloPersonDetector.image_qos(...)
    image_qos = staticmethod(image_qos)

    def on_image_async(self, msg):
        """Subscription callback: park the newest frame for the worker, replacing any older one."""
        with self._cond:
            self._pending = msg
            self._cond.notify()

    def _worker(self):
        """Process the newest waiting frame, one at a time; frames that arrive meanwhile are
        overwritten, so the node always works on the most recent image."""
        while True:
            with self._cond:
                while self._pending is None and not self._stop:
                    self._cond.wait()
                if self._stop:
                    return
                msg, self._pending = self._pending, None
            try:
                self.on_image(msg)
            except Exception as e:  # never let one bad frame kill the worker
                self.get_logger().error(
                    f"Frame failed, skipping: {e}", throttle_duration_sec=5.0)

    def shutdown(self):
        """Stop the worker thread (call before destroy_node)."""
        with self._cond:
            self._stop = True
            self._cond.notify_all()
        if self._worker_thread is not None:
            self._worker_thread.join(timeout=2.0)

    def on_image(self, msg):
        """Handle one camera frame: run YOLO and publish the tracked person, if any."""
        t0 = time.perf_counter()
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        blob, scale, (pad_x, pad_y) = self._letterbox(frame)
        t1 = time.perf_counter()
        try:
            raw = self._infer(blob)
        except InferenceServerException as e:
            # Triton slow or down: skip this frame instead of crashing the node
            self.get_logger().warn(
                f"Triton inference failed, skipping frame: {e}", throttle_duration_sec=5.0)
            return
        t2 = time.perf_counter()
        fh, fw = frame.shape[:2]
        cands = decode_candidates(raw, scale, pad_x, pad_y, fw, fh, self.keep_confidence)
        cands = [c for c in cands if is_plausible_box(
            c[1], fh, self.min_box_height_frac, self.min_aspect, self.max_aspect)]
        det = self.gate.update(cands)
        if not self.gate.confirmed:
            self.smoother.reset()
        self._record_timing(t0, t1, t2, time.perf_counter())
        if det is None:
            return                           # no confirmed person -> publish nothing

        conf, box = det
        if self.smooth_box:
            stamp_s = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
            box = clip_box(self.smoother.update(stamp_s, box), fw, fh)
        out = Detection()
        out.header = msg.header              # timestamp = camera capture time
        out.confidence = float(conf)         # float() because ROS rejects numpy floats
        out.x_min, out.y_min, out.x_max, out.y_max = (float(v) for v in box)
        self.pub.publish(out)

    def _record_timing(self, t0, t1, t2, t3):
        """Add one frame's stage durations; log a summary every timing_report_s seconds."""
        if self.timing_report_s <= 0:
            return
        self.timer.record("preprocess", t1 - t0)
        self.timer.record("triton", t2 - t1)
        self.timer.record("decode", t3 - t2)
        self.timer.record("total", t3 - t0)
        self.timer.count_frame()
        now = time.monotonic()
        window = now - self._timer_start
        if window >= self.timing_report_s:
            self.get_logger().info(f"timing: {self.timer.summary(window)}")
            self.timer.reset()
            self._timer_start = now

    @staticmethod
    def _letterbox(frame):
        """Fit a camera frame into the 640x640 input the Triton model expects.

        Returns the model-ready blob plus the scale and padding used, which
        are needed later to map the model's boxes back onto the real frame.
        """
        size = INPUT_SIZE
        fh, fw = frame.shape[:2]
        scale = min(size / fw, size / fh)        # smaller ratio: both sides fit, no distortion
        nw, nh = int(round(fw * scale)), int(round(fh * scale))
        pad_x = (size - nw) // 2                 # center the image; one of these is always 0
        pad_y = (size - nh) // 2
        resized = cv2.resize(frame, (nw, nh))    # note: resize takes (width, height)
        # 114 = the grey padding YOLO was trained with
        canvas = np.full((size, size, 3), 114, dtype=np.uint8)
        canvas[pad_y:pad_y + nh, pad_x:pad_x + nw] = resized
        blob = canvas[:, :, ::-1]                # BGR -> RGB: model trained on RGB
        blob = blob.transpose(2, 0, 1)           # HWC -> CHW: model wants channels first
        blob = blob.astype(np.float32) / 255.0   # 0-255 -> 0.0-1.0
        blob = blob[None]                        # add batch dimension -> (1, 3, 640, 640)
        return np.ascontiguousarray(blob), scale, (pad_x, pad_y)

    def _infer(self, blob):
        """Send one letterboxed blob to Triton and return the model's raw output."""
        inp = grpcclient.InferInput("images", list(blob.shape), "FP32")
        inp.set_data_from_numpy(blob)
        out = grpcclient.InferRequestedOutput("output0")
        result = self.triton.infer(
            self.model_name, [inp], outputs=[out], client_timeout=self.triton_timeout_s)
        return result.as_numpy("output0")        # (1, 84, 8400)

    @staticmethod
    def _decode(raw, scale, pad_x, pad_y, fw, fh, min_confidence=KEEP_CONFIDENCE):
        """The single most confident person, mapped back to frame pixels.

        Returns (conf, x_min, y_min, x_max, y_max), or None if no person scores
        min_confidence. The node itself uses decode_candidates plus the track gate; this is the
        plain best-guess view, kept for tests and quick checks.
        """
        cands = decode_candidates(raw, scale, pad_x, pad_y, fw, fh, min_confidence)
        if not cands:
            return None
        conf, box = cands[0]
        return (conf, *box)


def main():
    rclpy.init()
    node = YoloPersonDetector()
    try:
        rclpy.spin(node)          # blocks here, calling on_image as frames arrive
    except KeyboardInterrupt:
        pass                      # Ctrl+C: exit quietly instead of a traceback
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
