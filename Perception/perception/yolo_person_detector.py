import rclpy
import cv2
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO


class YoloPersonDetector(Node):
    def __init__(self):
        super().__init__("yolo_person_detector")
        self.model = YOLO
        self.bridge = CvBridge()
        self.sub = self.create_subscription(
            Image, "/camera/front/image_raw", self.on_image, 10)
        
    def on_image(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.get_logger().info(f"decoded frame shape: {frame.shape}")

        blob, scale, (pad_x, pad_y) = self._letterbox(frame)
        self.get_logger().info(f"blob {blob.shape} {blob.dtype} scale={scale:.3f} pad=({pad_x},{pad_y})")

    def _letterbox(self,frame):
        size = 640
        fh, fw = frame.shape[:2]
        scale = min(size / fw, size / fh)
        nw, nh = int(round(fw * scale)), int(round(fh * scale))

        pad_x = (size -nw) //2
        pad_y = (size -nh) //2

        resized = cv2.resize(frame, (nw, nh))
        canvas = np.full((size, size, 3), 114, dtype=np.uint8)
        canvas[pad_y:nh + pad_y, pad_x:nw + pad_x] = resized

        blob = canvas[:,:,::-1]
        blob = blob.transpose(2, 0, 1)
        blob = blob.astype(np.float32) /255.0
        blob = blob[None]

        return np.ascontiguousarray(blob), scale, (pad_x, pad_y)


        



def main():
    rclpy.init()
    node = YoloPersonDetector()
    rclpy.spin(node)
    rclpy.shutdown()