"""Exchange velocity commands and simulated sensors with Webots over UDP."""

import json
import math
import socket
import struct
import time
import zlib

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, Imu, LaserScan
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


CAMERA_MAGIC = b"GO2CAM1"
CAMERA_HEADER = struct.Struct("!7sIHHHH")


class CmdVelBridge(Node):
    def __init__(self) -> None:
        super().__init__("go2_webots_cmd_vel_bridge")
        self.declare_parameter("host", "127.0.0.1")
        self.declare_parameter("port", 15000)
        self.declare_parameter("topic", "/cmd_vel")
        self._host = str(self.get_parameter("host").value)
        self._port = int(self.get_parameter("port").value)
        topic = str(self.get_parameter("topic").value)
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.setblocking(False)
        self._camera_frames = {}
        self._latest_pose = None
        self.create_subscription(Twist, topic, self._on_twist, 10)
        self._scan_publisher = self.create_publisher(LaserScan, "/scan", 10)
        self._raw_scan_publisher = self.create_publisher(LaserScan, "/scan_raw", 10)
        self._imu_publisher = self.create_publisher(Imu, "/imu", 20)
        self._odom_publisher = self.create_publisher(Odometry, "/odom", 20)
        self._tf_broadcaster = TransformBroadcaster(self)
        self._static_tf_broadcaster = StaticTransformBroadcaster(self)
        self._image_publisher = self.create_publisher(Image, "/camera/image_raw", 5)
        self._camera_info_publisher = self.create_publisher(
            CameraInfo, "/camera/camera_info", 5
        )
        self.create_timer(0.005, self._receive_packets)
        self.create_timer(1.0, self._send_hello)
        self._publish_static_transforms()
        self._send_hello()
        self.get_logger().info(
            f"Bridge udp://{self._host}:{self._port}: {topic} -> Webots, "
            "Webots -> /scan, /scan_raw, /camera/image_raw, /imu and /odom"
        )

    def _send(self, payload: bytes) -> None:
        self._socket.sendto(payload, (self._host, self._port))

    def _send_hello(self) -> None:
        self._send(b'{"type":"hello"}')

    def _on_twist(self, message: Twist) -> None:
        payload = json.dumps(
            {"vx": message.linear.x, "vy": message.linear.y, "wz": message.angular.z},
            separators=(",", ":"),
        ).encode("utf-8")
        self._send(payload)

    def _receive_packets(self) -> None:
        for _ in range(64):
            try:
                payload, _ = self._socket.recvfrom(65535)
            except BlockingIOError:
                break
            if payload.startswith(CAMERA_MAGIC):
                self._receive_camera_chunk(payload)
            else:
                self._receive_json(payload)
        self._discard_stale_camera_frames()

    def _receive_json(self, payload: bytes) -> None:
        try:
            message = json.loads(payload.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError):
            return
        message_type = message.get("type")
        if message_type == "state":
            self._publish_state(message)
            return
        if message_type not in ("scan", "scan_raw"):
            return

        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        # State and scan are separate UDP datagrams. Re-stamp the latest pose
        # at the scan time so costmaps never receive a scan outside the TF
        # cache, even when camera packets temporarily delay state delivery.
        self._publish_latest_transform(scan.header.stamp)
        scan.header.frame_id = str(message.get("frame_id", "lidar_link"))
        scan.angle_min = float(message["angle_min"])
        scan.angle_max = float(message["angle_max"])
        scan.range_min = float(message["range_min"])
        scan.range_max = float(message["range_max"])
        # Webots orders a lidar layer from +FOV/2 to -FOV/2 (clockwise), as
        # reflected by the negative angle_increment in the official
        # webots_ros2 Ros2Lidar plugin.  ROS consumers here use increasing
        # angles, so reverse the samples instead of publishing a mirrored scan.
        scan.ranges = [float(value) for value in reversed(message["ranges"])]
        if len(scan.ranges) > 1:
            scan.angle_increment = (scan.angle_max - scan.angle_min) / (
                len(scan.ranges) - 1
            )
        scan.scan_time = 0.1
        scan.time_increment = scan.scan_time / max(1, len(scan.ranges))
        publisher = self._raw_scan_publisher if message_type == "scan_raw" else self._scan_publisher
        publisher.publish(scan)

    def _publish_state(self, message) -> None:
        stamp = self.get_clock().now().to_msg()
        position = [float(value) for value in message["position"]]
        orientation = [float(value) for value in message["orientation"]]
        self._latest_pose = (position, orientation)
        linear = [float(value) for value in message["linear_velocity"]]
        angular = [float(value) for value in message["angular_velocity"]]

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z = position
        (
            odom.pose.pose.orientation.x,
            odom.pose.pose.orientation.y,
            odom.pose.pose.orientation.z,
            odom.pose.pose.orientation.w,
        ) = orientation
        odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z = linear
        odom.twist.twist.angular.x, odom.twist.twist.angular.y, odom.twist.twist.angular.z = angular
        self._odom_publisher.publish(odom)

        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = "base_link"
        imu.orientation = odom.pose.pose.orientation
        imu.angular_velocity = odom.twist.twist.angular
        imu.linear_acceleration_covariance[0] = -1.0
        self._imu_publisher.publish(imu)

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = "odom"
        transform.child_frame_id = "base_link"
        transform.transform.translation.x = position[0]
        transform.transform.translation.y = position[1]
        transform.transform.translation.z = position[2]
        transform.transform.rotation = odom.pose.pose.orientation
        self._tf_broadcaster.sendTransform(transform)

    def _publish_latest_transform(self, stamp) -> None:
        if self._latest_pose is None:
            return
        position, orientation = self._latest_pose
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = "odom"
        transform.child_frame_id = "base_link"
        transform.transform.translation.x = position[0]
        transform.transform.translation.y = position[1]
        transform.transform.translation.z = position[2]
        (
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w,
        ) = orientation
        self._tf_broadcaster.sendTransform(transform)

    def _publish_static_transforms(self) -> None:
        stamp = self.get_clock().now().to_msg()
        transforms = []
        for child, xyz, quaternion in (
            ("lidar_link", (0.08, 0.0, 0.13), (0.0, 0.0, 0.0, 1.0)),
            (
                "front_camera_optical_frame",
                (0.45, 0.0, 0.05),
                (-0.5, 0.5, -0.5, 0.5),
            ),
        ):
            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = "base_link"
            transform.child_frame_id = child
            transform.transform.translation.x = xyz[0]
            transform.transform.translation.y = xyz[1]
            transform.transform.translation.z = xyz[2]
            transform.transform.rotation.x = quaternion[0]
            transform.transform.rotation.y = quaternion[1]
            transform.transform.rotation.z = quaternion[2]
            transform.transform.rotation.w = quaternion[3]
            transforms.append(transform)
        self._static_tf_broadcaster.sendTransform(transforms)

    def _receive_camera_chunk(self, payload: bytes) -> None:
        if len(payload) < CAMERA_HEADER.size:
            return
        magic, frame_id, chunk_index, chunk_count, width, height = CAMERA_HEADER.unpack_from(
            payload
        )
        if magic != CAMERA_MAGIC or chunk_count == 0 or chunk_index >= chunk_count:
            return
        frame = self._camera_frames.setdefault(
            frame_id,
            {
                "created": time.monotonic(),
                "width": width,
                "height": height,
                "chunks": [None] * chunk_count,
            },
        )
        if len(frame["chunks"]) != chunk_count:
            self._camera_frames.pop(frame_id, None)
            return
        frame["chunks"][chunk_index] = payload[CAMERA_HEADER.size :]
        if all(chunk is not None for chunk in frame["chunks"]):
            try:
                image_data = zlib.decompress(b"".join(frame["chunks"]))
            except zlib.error:
                self._camera_frames.pop(frame_id, None)
                return
            self._publish_camera(image_data, int(frame["width"]), int(frame["height"]))
            self._camera_frames.pop(frame_id, None)

    def _publish_camera(self, data: bytes, width: int, height: int) -> None:
        expected_size = width * height * 4
        if len(data) != expected_size:
            return
        stamp = self.get_clock().now().to_msg()
        image = Image()
        image.header.stamp = stamp
        image.header.frame_id = "front_camera_optical_frame"
        image.height = height
        image.width = width
        image.encoding = "bgra8"
        image.is_bigendian = 0
        image.step = width * 4
        image.data = data
        self._image_publisher.publish(image)

        horizontal_fov = math.radians(80.0)
        focal_length = width / (2.0 * math.tan(horizontal_fov / 2.0))
        info = CameraInfo()
        info.header = image.header
        info.height = height
        info.width = width
        info.distortion_model = "plumb_bob"
        info.d = [0.0] * 5
        info.k = [
            focal_length,
            0.0,
            width / 2.0,
            0.0,
            focal_length,
            height / 2.0,
            0.0,
            0.0,
            1.0,
        ]
        info.p = [
            focal_length,
            0.0,
            width / 2.0,
            0.0,
            0.0,
            focal_length,
            height / 2.0,
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
        ]
        self._camera_info_publisher.publish(info)

    def _discard_stale_camera_frames(self) -> None:
        now = time.monotonic()
        stale = [
            frame_id
            for frame_id, frame in self._camera_frames.items()
            if now - frame["created"] > 0.5
        ]
        for frame_id in stale:
            self._camera_frames.pop(frame_id, None)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CmdVelBridge()
    try:
        rclpy.spin(node)
    finally:
        node._socket.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
