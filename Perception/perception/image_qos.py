"""QoS for the camera image, shared by the nodes that read it (no Triton import here)."""

from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

IMAGE_RELIABILITY = "reliable"   # the sim bridge publishes reliable


def image_qos(reliability):
    """QoS for the camera image: keep only the newest frame.

    Reliable by default: a 1920x1080 frame (6 MB) is fragmented over UDP, and a best-effort
    reader loses the whole frame when one fragment is lost (measured: about 4 Hz instead of
    15 Hz in the sim). A reliable reader cannot connect to a best-effort publisher, so a camera
    driver that publishes best-effort needs image_reliability:=best_effort.
    """
    policy = (ReliabilityPolicy.BEST_EFFORT if str(reliability).lower() == "best_effort"
              else ReliabilityPolicy.RELIABLE)
    return QoSProfile(reliability=policy, history=HistoryPolicy.KEEP_LAST, depth=1)
