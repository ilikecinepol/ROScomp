"""Forward RViz's standard /goal_pose topic to Nav2 NavigateToPose."""

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node


class GoalPoseRelay(Node):
    def __init__(self) -> None:
        super().__init__("go2_goal_pose_relay")
        self._client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.create_subscription(PoseStamped, "/goal_pose", self._on_goal, 10)
        self.get_logger().info("Ready: /goal_pose -> Nav2 /navigate_to_pose")

    def _on_goal(self, pose: PoseStamped) -> None:
        if not self._client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error("Nav2 action server is not ready")
            return
        goal = NavigateToPose.Goal()
        goal.pose = pose
        self.get_logger().info(
            f"Sending map goal: x={pose.pose.position.x:.2f}, "
            f"y={pose.pose.position.y:.2f}"
        )
        future = self._client.send_goal_async(goal)
        future.add_done_callback(self._goal_response)

    def _goal_response(self, future) -> None:
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error("Nav2 rejected the RViz goal")
            return
        self.get_logger().info("Nav2 accepted the RViz goal")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GoalPoseRelay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
