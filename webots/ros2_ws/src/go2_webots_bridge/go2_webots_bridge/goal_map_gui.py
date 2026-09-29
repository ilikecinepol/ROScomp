"""Small OpenGL-free SLAM map and goal selector for WSLg."""

import math
import sys

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from PyQt5.QtCore import QPoint, QPointF, QRectF, Qt, QTimer
from PyQt5.QtGui import QColor, QImage, QPainter, QPen
from PyQt5.QtWidgets import QApplication, QLabel, QVBoxLayout, QWidget
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


class MapCanvas(QWidget):
    def __init__(self, send_goal, set_status) -> None:
        super().__init__()
        self.setMinimumSize(900, 650)
        self.setMouseTracking(True)
        self._send_goal = send_goal
        self._set_status = set_status
        self._image = None
        self._grid = None
        self._resolution = 0.05
        self._origin = (0.0, 0.0, 0.0)
        self._robot = None
        self._drag_start = None
        self._drag_now = None

    def set_map(self, message: OccupancyGrid) -> None:
        width, height = message.info.width, message.info.height
        grid = np.asarray(message.data, dtype=np.int16).reshape((height, width))
        pixels = np.empty((height, width, 3), dtype=np.uint8)
        pixels[grid < 0] = (70, 70, 70)
        free = grid == 0
        pixels[free] = (238, 238, 238)
        occupied = grid > 0
        shade = np.clip(238 - grid[occupied] * 2, 20, 220).astype(np.uint8)
        pixels[occupied] = np.column_stack((shade, shade, shade))
        image = QImage(
            pixels.data, width, height, width * 3, QImage.Format_RGB888
        ).copy().mirrored(False, True)
        q = message.info.origin.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self._image = image
        self._grid = grid
        self._resolution = message.info.resolution
        self._origin = (
            message.info.origin.position.x,
            message.info.origin.position.y,
            yaw,
        )
        self.update()

    def set_robot(self, x: float, y: float, yaw: float) -> None:
        self._robot = (x, y, yaw)
        self.update()

    def _image_rect(self) -> QRectF:
        if self._image is None:
            return QRectF()
        scale = min(self.width() / self._image.width(), self.height() / self._image.height())
        width = self._image.width() * scale
        height = self._image.height() * scale
        return QRectF((self.width() - width) / 2, (self.height() - height) / 2, width, height)

    def _screen_to_world(self, point: QPointF):
        rect = self._image_rect()
        if self._image is None or not rect.contains(point):
            return None
        px = (point.x() - rect.left()) * self._image.width() / rect.width()
        py = (point.y() - rect.top()) * self._image.height() / rect.height()
        gx = px * self._resolution
        gy = (self._image.height() - 1 - py) * self._resolution
        ox, oy, yaw = self._origin
        return (
            ox + math.cos(yaw) * gx - math.sin(yaw) * gy,
            oy + math.sin(yaw) * gx + math.cos(yaw) * gy,
        )

    def _world_to_screen(self, x: float, y: float):
        if self._image is None:
            return None
        ox, oy, yaw = self._origin
        dx, dy = x - ox, y - oy
        gx = (math.cos(yaw) * dx + math.sin(yaw) * dy) / self._resolution
        gy = (-math.sin(yaw) * dx + math.cos(yaw) * dy) / self._resolution
        rect = self._image_rect()
        return QPointF(
            rect.left() + gx * rect.width() / self._image.width(),
            rect.top() + (self._image.height() - 1 - gy) * rect.height() / self._image.height(),
        )

    def _is_free(self, world) -> bool:
        if self._grid is None:
            return False
        ox, oy, yaw = self._origin
        dx, dy = world[0] - ox, world[1] - oy
        gx = int((math.cos(yaw) * dx + math.sin(yaw) * dy) / self._resolution)
        gy = int((-math.sin(yaw) * dx + math.cos(yaw) * dy) / self._resolution)
        if gx < 0 or gy < 0 or gy >= self._grid.shape[0] or gx >= self._grid.shape[1]:
            return False
        return 0 <= int(self._grid[gy, gx]) < 50

    def mousePressEvent(self, event) -> None:
        if event.button() == Qt.LeftButton:
            self._drag_start = QPointF(event.pos())
            self._drag_now = QPointF(event.pos())

    def mouseMoveEvent(self, event) -> None:
        if self._drag_start is not None:
            self._drag_now = QPointF(event.pos())
            self.update()

    def mouseReleaseEvent(self, event) -> None:
        if event.button() != Qt.LeftButton or self._drag_start is None:
            return
        start = self._screen_to_world(self._drag_start)
        end = self._screen_to_world(QPointF(event.pos()))
        self._drag_start = None
        self._drag_now = None
        self.update()
        if start is None or end is None:
            self._set_status("Цель должна находиться внутри карты")
            return
        if not self._is_free(start):
            self._set_status("Выберите светлую свободную клетку")
            return
        yaw = math.atan2(end[1] - start[1], end[0] - start[0])
        self._send_goal(start[0], start[1], yaw)

    def paintEvent(self, _event) -> None:
        painter = QPainter(self)
        painter.fillRect(self.rect(), QColor(35, 35, 35))
        if self._image is None:
            painter.setPen(Qt.white)
            painter.drawText(self.rect(), Qt.AlignCenter, "Ожидание карты /map…")
            return
        painter.drawImage(self._image_rect(), self._image)
        if self._robot is not None:
            x, y, yaw = self._robot
            center = self._world_to_screen(x, y)
            if center is not None:
                painter.setPen(QPen(QColor(0, 110, 255), 3))
                painter.setBrush(QColor(0, 110, 255))
                painter.drawEllipse(center, 7, 7)
                tip = QPointF(center.x() + 24 * math.cos(yaw), center.y() - 24 * math.sin(yaw))
                painter.drawLine(center, tip)
        if self._drag_start is not None and self._drag_now is not None:
            painter.setPen(QPen(QColor(255, 80, 0), 4))
            painter.drawLine(self._drag_start, self._drag_now)


class GoalMapNode(Node):
    def __init__(self, canvas: MapCanvas, status: QLabel) -> None:
        super().__init__("go2_goal_map_gui")
        self._canvas = canvas
        self._status = status
        self._goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self.create_subscription(OccupancyGrid, "/map", self._on_map, 10)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

    def _on_map(self, message: OccupancyGrid) -> None:
        self._canvas.set_map(message)

    def send_goal(self, x: float, y: float, yaw: float) -> None:
        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "map"
        message.pose.position.x = x
        message.pose.position.y = y
        message.pose.orientation.z = math.sin(yaw / 2.0)
        message.pose.orientation.w = math.cos(yaw / 2.0)
        self._goal_pub.publish(message)
        self._status.setText(f"Цель отправлена: x={x:.2f}, y={y:.2f}")

    def update_robot(self) -> None:
        try:
            transform = self._tf_buffer.lookup_transform("map", "base_link", Time())
        except TransformException:
            return
        p = transform.transform.translation
        q = transform.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self._canvas.set_robot(p.x, p.y, yaw)


def main(args=None) -> None:
    rclpy.init(args=args)
    app = QApplication(sys.argv)
    window = QWidget()
    window.setWindowTitle("Go2 SLAM — выбор цели")
    status = QLabel("ЛКМ: нажмите на свободную клетку и протяните стрелку направления")
    status.setStyleSheet("font-size: 16px; padding: 8px;")
    canvas = MapCanvas(lambda *_: None, status.setText)
    node = GoalMapNode(canvas, status)
    canvas._send_goal = node.send_goal
    layout = QVBoxLayout(window)
    layout.addWidget(status)
    layout.addWidget(canvas)
    timer = QTimer()
    timer.timeout.connect(lambda: (rclpy.spin_once(node, timeout_sec=0.0), node.update_robot()))
    timer.start(20)
    def place_on_primary_screen() -> None:
        screen = app.screenAt(QPoint(0, 0)) or app.primaryScreen()
        area = screen.availableGeometry()
        width = min(1100, max(640, area.width() - 80))
        height = min(800, max(480, area.height() - 80))
        window.setGeometry(
            area.left() + max(20, (area.width() - width) // 2),
            area.top() + max(20, (area.height() - height) // 2),
            width,
            height,
        )
        window.setWindowState(Qt.WindowNoState)
        window.showNormal()
        window.raise_()
        window.activateWindow()

    # WSLg can restore a Linux window at coordinates belonging to a monitor
    # that is no longer connected. Re-apply a visible geometry after the
    # native Windows window has been created.
    window.setWindowFlag(Qt.WindowStaysOnTopHint, True)
    place_on_primary_screen()
    QTimer.singleShot(300, place_on_primary_screen)
    QTimer.singleShot(1200, place_on_primary_screen)
    try:
        app.exec_()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
