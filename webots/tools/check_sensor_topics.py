"""Wait for one valid simulated Go2 lidar scan and camera frame."""

import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan


class SensorCheck(Node):
    def __init__(self) -> None:
        super().__init__("go2_sensor_check")
        self.scan = None
        self.image = None
        self.create_subscription(LaserScan, "/scan", self._on_scan, 10)
        self.create_subscription(Image, "/camera/image_raw", self._on_image, 10)

    def _on_scan(self, message: LaserScan) -> None:
        self.scan = message

    def _on_image(self, message: Image) -> None:
        self.image = message


def main() -> int:
    rclpy.init()
    node = SensorCheck()
    deadline = time.monotonic() + 10.0
    try:
        while time.monotonic() < deadline and (node.scan is None or node.image is None):
            rclpy.spin_once(node, timeout_sec=0.25)
        if node.scan is None or node.image is None:
            print("FAIL: timed out waiting for /scan and /camera/image_raw")
            return 1
        scan_ok = len(node.scan.ranges) == 360 and node.scan.range_max == 12.0
        image_ok = (
            node.image.width == 320
            and node.image.height == 240
            and node.image.encoding == "bgra8"
            and len(node.image.data) == 320 * 240 * 4
        )
        camera_array = np.frombuffer(node.image.data, dtype=np.uint8).reshape(
            node.image.height, node.image.width, 4
        )
        output_path = Path(
            "/mnt/c/Users/drmma/Documents/ChatGPT/ROS2 Competition/"
            "webots/test-results/go2_camera.png"
        )
        output_path.parent.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(output_path), camera_array)
        print(
            f"scan: samples={len(node.scan.ranges)}, "
            f"fov={node.scan.angle_max - node.scan.angle_min:.3f} rad"
        )
        print(
            f"camera: {node.image.width}x{node.image.height}, "
            f"encoding={node.image.encoding}, bytes={len(node.image.data)}"
        )
        print(f"camera snapshot: {output_path}")
        print("PASS" if scan_ok and image_ok else "FAIL: invalid sensor metadata")
        return 0 if scan_ok and image_ok else 2
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
