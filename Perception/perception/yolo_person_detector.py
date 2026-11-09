"""YOLO person detector node.

Subscribes to the drone's camera, sends each frame to the shared Triton
YOLO server, and publishes the most confident person as a
perception_msgs/Detection.

Topics
    in   /camera/front/image_raw   sensor_msgs/Image (sensor-data QoS)
    out  /perception/detection     perception_msgs/Detection (only when a person is seen)

Parameters (all optional, see declare_parameter below for defaults)
    triton_url, model_name, min_confidence, triton_timeout_s
"""

import os

import cv2
import numpy as np
import rclpy
import tritonclient.grpc as grpcclient
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from tritonclient.utils import InferenceServerException

from perception_msgs.msg import Detection

INPUT_SIZE = 640        # fixed by config.pbtxt: dims [3, 640, 640]
MIN_CONFIDENCE = 0.5    # default threshold; below it a frame counts as "no person"
PERSON_ROW = 4          # rows 0-3 = box, then 80 class scores; person is class 0


class YoloPersonDetector(Node):
    """ROS2 node: camera frames in, person detections out."""

    def __init__(self):
        super().__init__("yolo_person_detector")

        # Settings come from ROS parameters so they can change per launch
        # without editing code. Triton's URL defaults to the container's env var.
        self.triton_url = self.declare_parameter(
            "triton_url", os.environ.get("HALO_TRITON_URL", "triton:8001")).value
        self.model_name = self.declare_parameter("model_name", "yolo_person").value
        self.min_confidence = self.declare_parameter("min_confidence", MIN_CONFIDENCE).value
        self.triton_timeout_s = self.declare_parameter("triton_timeout_s", 0.1).value

        # Converts ROS Image <-> numpy; made once, reused every frame
        self.bridge = CvBridge()
        # gRPC client for the shared Triton server (connects on first infer)
        self.triton = grpcclient.InferenceServerClient(url=self.triton_url)

        # Input: camera frames. Sensor-data QoS (best effort, small queue)
        # drops stale frames instead of building up lag.
        self.sub = self.create_subscription(
            Image, "/camera/front/image_raw", self.on_image, qos_profile_sensor_data)
        # Output: one Detection per frame that contains a person
        self.pub = self.create_publisher(Detection, "/perception/detection", 10)

        self.get_logger().info(
            f"Ready: triton={self.triton_url} model={self.model_name} "
            f"min_confidence={self.min_confidence} timeout={self.triton_timeout_s}s")

    def on_image(self, msg):
        """Handle one camera frame: run YOLO and publish the best person, if any."""
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        blob, scale, (pad_x, pad_y) = self._letterbox(frame)
        try:
            raw = self._infer(blob)
        except InferenceServerException as e:
            # Triton slow or down: skip this frame instead of crashing the node
            self.get_logger().warn(
                f"Triton inference failed, skipping frame: {e}", throttle_duration_sec=5.0)
            return
        fh, fw = frame.shape[:2]
        det = self._decode(raw, scale, pad_x, pad_y, fw, fh, self.min_confidence)
        if det is None:
            return                           # no person -> publish nothing

        conf, x_min, y_min, x_max, y_max = det
        out = Detection()
        out.header = msg.header              # timestamp = camera capture time
        out.confidence = float(conf)         # float() because ROS rejects numpy floats
        out.x_min = float(x_min)
        out.y_min = float(y_min)
        out.x_max = float(x_max)
        out.y_max = float(y_max)
        self.pub.publish(out)

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
    def _decode(raw, scale, pad_x, pad_y, fw, fh, min_confidence=MIN_CONFIDENCE):
        """Pick the most confident person box and map it back to frame pixels.

        Returns (conf, x_min, y_min, x_max, y_max), or None if no person.
        """
        preds = raw[0]                    # drop batch dim -> (84, 8400)
        scores = preds[PERSON_ROW]        # person score for all 8400 guesses
        best = int(np.argmax(scores))     # index of the single most confident guess
        conf = float(scores[best])
        if conf < min_confidence:         # nobody confidently there -> no detection
            return None
        cx, cy, w, h = preds[0:4, best]   # box center + size, in 640x640 letterbox pixels
        # Undo the letterbox: remove the padding, then undo the resize
        x_min = (cx - w / 2 - pad_x) / scale
        y_min = (cy - h / 2 - pad_y) / scale
        x_max = (cx + w / 2 - pad_x) / scale
        y_max = (cy + h / 2 - pad_y) / scale
        # Keep the box inside the real frame
        x_min, x_max = max(0.0, x_min), min(float(fw), x_max)
        y_min, y_max = max(0.0, y_min), min(float(fh), y_max)
        return conf, x_min, y_min, x_max, y_max


def main():
    rclpy.init()
    node = YoloPersonDetector()
    try:
        rclpy.spin(node)          # blocks here, calling on_image as frames arrive
    except KeyboardInterrupt:
        pass                      # Ctrl+C: exit quietly instead of a traceback
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
