"""Subscribes to a camera image topic and detects a person with a YOLO
model (Ultralytics YOLOv8n by default), filtered to the COCO "person" class.

Same publish contract as person_detector.py (HOG-based), so either can be
picked per launch file and the tracker consumes both the same way: centroid
+ bbox height on ~/person/position (PointStamped: x=u, y=v,
z=bbox_height_px), a distance estimate (meters) on ~/person/distance, and an
annotated debug image (960px wide) on ~/person/debug_image — the latter
only while something is subscribed to it, since drawing and encoding it
isn't free. View it with `ros2 run perception debug_image_viewer`.

Two inference backends, same output either way:

  local   Ultralytics in-process. Runs on the NVIDIA GPU when PyTorch can see
          one, CPU otherwise. What the WSL/laptop dev container uses.
  triton  Sends the frame to a shared NVIDIA Triton server (the single
          TensorRT engine on the DGX Spark that every developer's container
          shares — see infra/triton/). Triton batches frames from whichever
          containers have one ready, so five sims cost one model in GPU
          memory and one batched pass instead of five.

`backend` defaults to "auto": triton when HALO_TRITON_URL is set in the
environment (docker-compose.spark.yml sets it), local otherwise, so the
non-Spark setups behave exactly as before.

If Triton is slow or unreachable the frame is SKIPPED — nothing is
published for it — rather than re-publishing the previous detection. The
tracker's Kalman filter already predicts through gaps using message stamps
and drops the target after its own timeout; a stale box with a fresh stamp
would look like a real measurement and hide the outage.
"""

import os
import threading
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float32

COCO_PERSON_CLASS = 0
# Ultralytics' letterbox padding colour and its default NMS IoU threshold —
# matched so the Triton path produces the same boxes the local path would.
LETTERBOX_FILL = 114
NMS_IOU = 0.7


class InferenceUnavailable(Exception):
    """Triton did not return a usable result in time for this frame."""


class YoloPersonDetector(Node):
    """Runs frame capture and YOLO inference on separate threads.

    The ROS callback (on_image) only does the cheap cv_bridge decode and
    hands the frame off — it never runs inference itself, so it's never
    blocked waiting on a slow YOLO pass. A background thread takes whatever
    the newest handed-off frame is and runs detection + publishing. The
    handoff is a single slot, not a queue: a frame that arrives mid-inference
    replaces the pending one, so the worker always processes the latest
    frame instead of working through a stale backlog.
    """

    def __init__(self):
        super().__init__("yolo_person_detector")

        self.declare_parameter("image_topic", "/camera/front/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/front/camera_info")
        # Baked into the image by the Dockerfile, so no download at runtime.
        self.declare_parameter("model_path", "/opt/yolo/yolov8n.pt")
        self.declare_parameter("min_confidence", 0.5)
        # YOLO resizes frames to this before inference. Smaller = faster but
        # loses people who are far away.
        self.declare_parameter("inference_size", 640)
        # "" picks the GPU when PyTorch sees one; "cpu" or "cuda:0" to force.
        # Local backend only.
        self.declare_parameter("device", "")
        # "auto" | "local" | "triton" — see module docstring.
        self.declare_parameter("backend", "auto")
        # host:port of Triton's gRPC endpoint. Defaults to HALO_TRITON_URL.
        self.declare_parameter("triton_url", os.environ.get("HALO_TRITON_URL", ""))
        self.declare_parameter("triton_model", "yolo_person")
        # Per-request deadline. Comfortably above a batched TensorRT pass
        # (a few ms) but below the 15 Hz camera period, so a stalled server
        # costs one frame, not a growing backlog.
        self.declare_parameter("triton_timeout_s", 0.1)
        # Real-world height of the target person, in meters — matches
        # Perception/models/standing_person.sdf (~1.9m). YOLO's boxes fit the
        # person tightly, so unlike HOG no margin correction is needed.
        self.declare_parameter("person_height_m", 1.9)
        self.declare_parameter("publish_debug_image", True)
        # Debug images are shrunk to this width before publishing: a raw
        # 1080p frame is ~6MB, which is most of what makes rqt_image_view
        # lag. 0 publishes full resolution.
        self.declare_parameter("debug_image_width", 960)

        image_topic = self.get_parameter("image_topic").value
        camera_info_topic = self.get_parameter("camera_info_topic").value
        self.min_confidence = float(self.get_parameter("min_confidence").value)
        self.inference_size = int(self.get_parameter("inference_size").value)
        self.person_height_m = float(self.get_parameter("person_height_m").value)
        self.publish_debug_image = bool(self.get_parameter("publish_debug_image").value)
        self.debug_image_width = int(self.get_parameter("debug_image_width").value)
        self.bridge = CvBridge()
        self.focal_length_px = None  # populated once camera_info arrives

        backend = self.get_parameter("backend").value
        self.triton_url = self.get_parameter("triton_url").value
        if backend == "auto":
            backend = "triton" if self.triton_url else "local"
        self.backend = backend
        if backend == "triton":
            self._init_triton()
            where = f"Triton at {self.triton_url} (model '{self.triton_model}')"
        elif backend == "local":
            self._init_local()
            where = self.device
        else:
            raise ValueError(f"backend must be auto, local or triton (got '{backend}')")

        # Rate limit for "Triton unavailable" warnings: one per few seconds,
        # the rest show up as a count in the stats line.
        self._last_unavailable_warn = 0.0

        # Only the newest frame matters for a live tracker, so depth 1 — but
        # RELIABLE, not BEST_EFFORT. A 1080p frame is ~6MB, which Fast DDS
        # splits into ~100 fragments; best-effort loses the whole frame if
        # any one fragment is dropped, and measured in the sim that cut a
        # 15 Hz camera to ~2 Hz with multi-second gaps. Reliable repairs lost
        # fragments, and KEEP_LAST 1 still means a slow reader only ever
        # gets the newest frame, never a backlog. The same profile is used
        # for the debug image publishers, for the same reasons.
        image_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.image_sub = self.create_subscription(Image, image_topic, self.on_image, image_qos)
        self.camera_info_sub = self.create_subscription(
            CameraInfo, camera_info_topic, self.on_camera_info, 10
        )
        self.position_pub = self.create_publisher(PointStamped, "~/person/position", 10)
        self.distance_pub = self.create_publisher(Float32, "~/person/distance", 10)
        self.debug_image_pub = (
            self.create_publisher(Image, "~/person/debug_image", image_qos)
            if self.publish_debug_image
            else None
        )

        # Single-slot handoff between the ROS callback thread (producer) and
        # the inference worker thread (consumer). The condition guards both
        # the slot and the shutdown flag.
        self._cond = threading.Condition()
        self._pending = None  # (frame, header) or None
        self._shutdown = False
        self._worker = threading.Thread(target=self._inference_loop, daemon=True)
        self._worker.start()

        # Pipeline stats, logged every few seconds so a slow stage shows up
        # in the node's output: frames received from the camera, frames
        # replaced in the slot before the worker got to them, frames
        # processed, and time spent in inference / debug-image publishing.
        # Guarded by self._cond.
        self._stats = self._empty_stats()
        self.create_timer(5.0, self._log_stats)

        self.get_logger().info(
            f"Subscribed to '{image_topic}' for YOLO person detection on {where}"
        )

    # Backends are imported lazily so the Spark image doesn't need to load
    # PyTorch just to talk to Triton, and the laptop image doesn't need
    # tritonclient installed at all.
    def _init_local(self):
        import torch
        from ultralytics import YOLO

        self.device = self.get_parameter("device").value or (
            "cuda:0" if torch.cuda.is_available() else "cpu"
        )
        self.model = YOLO(self.get_parameter("model_path").value)

    def _init_triton(self):
        import tritonclient.grpc as grpcclient

        self._grpcclient = grpcclient
        self.triton_model = self.get_parameter("triton_model").value
        self.triton_timeout_s = float(self.get_parameter("triton_timeout_s").value)
        self.triton = grpcclient.InferenceServerClient(url=self.triton_url)
        # Not fatal if the server isn't up yet — every frame retries, and
        # the skip path below reports it. Just log so a bad URL is obvious.
        try:
            if not self.triton.is_model_ready(self.triton_model):
                self.get_logger().warn(
                    f"Triton model '{self.triton_model}' not ready at {self.triton_url}; "
                    "frames will be skipped until it is"
                )
        except Exception as exc:  # noqa: BLE001 — any transport error
            self.get_logger().warn(f"Triton not reachable at {self.triton_url}: {exc}")

    def on_camera_info(self, msg: CameraInfo):
        # K is the row-major 3x3 intrinsic matrix; K[4] is fy in pixels, the
        # vertical focal length that matches a height measurement.
        self.focal_length_px = msg.k[4]

    def on_image(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as exc:
            self.get_logger().warn(f"cv_bridge conversion failed: {exc}")
            return

        with self._cond:
            self._stats["received"] += 1
            if self._pending is not None:
                self._stats["dropped"] += 1
            self._pending = (frame, msg.header)  # replaces any unprocessed frame
            self._cond.notify()

    def _inference_loop(self):
        while True:
            with self._cond:
                while self._pending is None and not self._shutdown:
                    self._cond.wait()
                if self._shutdown:
                    return
                frame, header = self._pending
                self._pending = None

            try:
                self._process(frame, header)
            except Exception:
                if not self.context.ok():
                    return  # Ctrl+C shut ROS down mid-frame; nothing to publish to
                raise

    @staticmethod
    def _empty_stats():
        return {"received": 0, "dropped": 0, "processed": 0, "skipped": 0,
                "infer_s": 0.0, "infer_max_s": 0.0, "debug_s": 0.0,
                "since": time.monotonic()}

    def _log_stats(self):
        with self._cond:
            s, self._stats = self._stats, self._empty_stats()
        elapsed = time.monotonic() - s["since"]
        n = max(s["processed"], 1)
        self.get_logger().info(
            f"camera in {s['received'] / elapsed:.1f} Hz, "
            f"processed {s['processed'] / elapsed:.1f} Hz, "
            f"dropped {s['dropped']} stale, "
            f"skipped {s['skipped']} (inference unavailable) | "
            f"inference avg {1000 * s['infer_s'] / n:.0f} ms "
            f"(max {1000 * s['infer_max_s']:.0f}), "
            f"debug image avg {1000 * s['debug_s'] / n:.0f} ms"
        )

    def destroy_node(self):
        with self._cond:
            self._shutdown = True
            self._cond.notify()
        self._worker.join(timeout=2.0)
        super().destroy_node()

    def _process(self, frame, header):
        t0 = time.monotonic()
        try:
            detection = self._detect_person(frame)
        except InferenceUnavailable as exc:
            # Skip the frame: publish nothing, count it, warn occasionally.
            # See the module docstring for why not "reuse last detection".
            with self._cond:
                self._stats["skipped"] += 1
            now = time.monotonic()
            if now - self._last_unavailable_warn > 5.0:
                self._last_unavailable_warn = now
                self.get_logger().warn(f"skipping frame, Triton unavailable: {exc}")
            return
        t_infer = time.monotonic() - t0
        distance_m = None

        if detection is not None:
            (cx, cy), (_x1, y1, _x2, y2), _score = detection
            h = y2 - y1
            point = PointStamped()
            point.header = header
            point.point.x = float(cx)
            point.point.y = float(cy)
            point.point.z = float(h)
            self.position_pub.publish(point)

            if self.focal_length_px is not None and h > 0:
                # Pinhole model: apparent_height_px / focal_length_px ==
                # real_height_m / distance_m, solved for distance_m.
                distance_m = self.person_height_m * self.focal_length_px / h
                self.distance_pub.publish(Float32(data=float(distance_m)))

        if (
            self.debug_image_pub is not None
            and self.debug_image_pub.get_subscription_count() > 0
        ):
            self._publish_debug_image(frame, detection, distance_m, header)

        with self._cond:
            self._stats["processed"] += 1
            self._stats["infer_s"] += t_infer
            self._stats["infer_max_s"] = max(self._stats["infer_max_s"], t_infer)
            self._stats["debug_s"] += time.monotonic() - t0 - t_infer

    def _detect_person(self, frame):
        """Returns ((cx, cy), (x1, y1, x2, y2), score) in frame pixels for the
        most confident person, or None. Same shape from either backend."""
        if self.backend == "triton":
            return self._detect_person_triton(frame)
        return self._detect_person_local(frame)

    def _detect_person_triton(self, frame):
        blob, scale, (pad_x, pad_y) = self._letterbox(frame)

        inputs = [self._grpcclient.InferInput("images", list(blob.shape), "FP32")]
        inputs[0].set_data_from_numpy(blob)
        outputs = [self._grpcclient.InferRequestedOutput("output0")]
        try:
            result = self.triton.infer(
                self.triton_model, inputs, outputs=outputs,
                client_timeout=self.triton_timeout_s,
            )
            raw = result.as_numpy("output0")
        except Exception as exc:  # noqa: BLE001 — timeouts, refused, server errors
            raise InferenceUnavailable(str(exc).splitlines()[0]) from exc
        if raw is None:
            raise InferenceUnavailable("empty response")

        # raw: [1, 84, 8400] → per-anchor rows of cx, cy, w, h, 80 class scores.
        pred = raw[0].T
        scores = pred[:, 4 + COCO_PERSON_CLASS]
        keep = scores >= self.min_confidence
        if not keep.any():
            return None
        boxes_cxcywh = pred[keep, :4]
        scores = scores[keep]

        # NMS in letterbox space, then map the winner back to frame pixels.
        xywh = boxes_cxcywh.copy()
        xywh[:, 0] -= xywh[:, 2] / 2
        xywh[:, 1] -= xywh[:, 3] / 2
        idx = cv2.dnn.NMSBoxes(xywh.tolist(), scores.tolist(), self.min_confidence, NMS_IOU)
        if len(idx) == 0:
            return None
        idx = np.asarray(idx).reshape(-1)
        best = idx[np.argmax(scores[idx])]
        x, y, w, h = xywh[best]
        fh, fw = frame.shape[:2]
        x1 = float(np.clip((x - pad_x) / scale, 0, fw))
        y1 = float(np.clip((y - pad_y) / scale, 0, fh))
        x2 = float(np.clip((x + w - pad_x) / scale, 0, fw))
        y2 = float(np.clip((y + h - pad_y) / scale, 0, fh))
        return ((x1 + x2) / 2.0, (y1 + y2) / 2.0), (x1, y1, x2, y2), float(scores[best])

    def _letterbox(self, frame):
        """Ultralytics-style preprocessing: scale to fit inference_size,
        centre on a grey canvas, BGR→RGB, CHW, float32 [0,1], batch of 1.
        Returns (blob, scale, (pad_x, pad_y)) so boxes can be mapped back."""
        size = self.inference_size
        fh, fw = frame.shape[:2]
        scale = min(size / fh, size / fw)
        nw, nh = int(round(fw * scale)), int(round(fh * scale))
        resized = cv2.resize(frame, (nw, nh), interpolation=cv2.INTER_LINEAR)
        pad_x, pad_y = (size - nw) // 2, (size - nh) // 2
        canvas = np.full((size, size, 3), LETTERBOX_FILL, dtype=np.uint8)
        canvas[pad_y:pad_y + nh, pad_x:pad_x + nw] = resized
        blob = canvas[:, :, ::-1].transpose(2, 0, 1).astype(np.float32) / 255.0
        return np.ascontiguousarray(blob[None]), scale, (pad_x, pad_y)

    def _detect_person_local(self, frame):
        results = self.model.predict(
            frame,
            imgsz=self.inference_size,
            classes=[COCO_PERSON_CLASS],
            conf=self.min_confidence,
            device=self.device,
            verbose=False,
        )[0]
        if len(results.boxes) == 0:
            return None

        best = max(results.boxes, key=lambda b: float(b.conf[0]))
        x1, y1, x2, y2 = (float(v) for v in best.xyxy[0])
        return ((x1 + x2) / 2.0, (y1 + y2) / 2.0), (x1, y1, x2, y2), float(best.conf[0])

    def _shrink_for_debug(self, image):
        if self.debug_image_width <= 0 or image.shape[1] <= self.debug_image_width:
            return image
        scale = self.debug_image_width / image.shape[1]
        return cv2.resize(image, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)

    def _publish_debug_image(self, frame, detection, distance_m, header):
        debug = frame.copy()
        if detection is not None:
            (cx, cy), (x1, y1, x2, y2), score = detection
            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
            cv2.rectangle(debug, (x1, y1), (x2, y2), (0, 255, 0), 2)
            label = f"person conf={score:.2f}"
            if distance_m is not None:
                label += f"  dist={distance_m:.2f}m"
            cv2.putText(
                debug,
                label,
                (x1, max(0, y1 - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )
            cv2.circle(debug, (int(cx), int(cy)), 4, (0, 255, 0), -1)

        try:
            debug_msg = self.bridge.cv2_to_imgmsg(
                self._shrink_for_debug(debug), encoding="bgr8"
            )
        except CvBridgeError as exc:
            self.get_logger().warn(f"cv_bridge conversion failed for debug image: {exc}")
            return
        debug_msg.header = header
        self.debug_image_pub.publish(debug_msg)


def main(args=None):
    rclpy.init(args=args)
    node = YoloPersonDetector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass  # Ctrl+C: rclpy's signal handler has already shut ROS down
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
