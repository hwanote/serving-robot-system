"""Check frontier approach goals using the Nav2 planner action."""

import math

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import Path
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from visualization_msgs.msg import Marker


class FrontierPathChecker(Node):
    """Validate frontier goals without commanding robot motion."""

    def __init__(self):
        super().__init__("frontier_path_checker")

        self.declare_parameter(
            "goals_topic", "/mapping/frontier_goals"
        )
        self.declare_parameter(
            "path_topic", "/mapping/frontier_valid_path"
        )
        self.declare_parameter(
            "planner_action", "/compute_path_to_pose"
        )
        self.declare_parameter("planner_id", "")

        goals_topic = self.get_parameter("goals_topic").value
        path_topic = self.get_parameter("path_topic").value
        planner_action = self.get_parameter("planner_action").value

        self.planner_id = self.get_parameter("planner_id").value

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.goals_sub = self.create_subscription(
            Marker,
            goals_topic,
            self.goals_callback,
            qos,
        )

        self.path_pub = self.create_publisher(
            Path,
            path_topic,
            qos,
        )

        self.action_client = ActionClient(
            self,
            ComputePathToPose,
            planner_action,
        )

        self.frame_id = "map"
        self.candidates = []
        self.signature = None
        self.completed_signature = None
        self.active_signature = None

        self.busy = False
        self.current_index = 0
        self.success_count = 0
        self.last_status = None

        self.timer = self.create_timer(1.0, self.tick)

        self.get_logger().info("Frontier Path Checker started")
        self.get_logger().info(f"Goals topic: {goals_topic}")
        self.get_logger().info(f"Planner action: {planner_action}")
        self.get_logger().info(f"Valid path topic: {path_topic}")
        self.get_logger().info(
            "Planning only: no NavigateToPose or cmd_vel"
        )

    def report_status(self, message):
        if message != self.last_status:
            self.get_logger().info(message)
            self.last_status = message

    def clear_path(self):
        path = Path()
        path.header.frame_id = self.frame_id
        path.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(path)

    def goals_callback(self, msg):
        if msg.action == Marker.DELETE:
            self.candidates = []
            self.signature = None
            self.completed_signature = None
            self.clear_path()
            self.report_status("No frontier goals available")
            return

        if msg.action != Marker.ADD:
            return

        if msg.type != Marker.SPHERE_LIST:
            return

        frame_id = msg.header.frame_id

        if not frame_id:
            self.get_logger().warn(
                "Ignoring frontier goals without a frame_id"
            )
            return

        candidates = []
        seen = set()

        for point in msg.points:
            if not (
                math.isfinite(point.x)
                and math.isfinite(point.y)
            ):
                continue

            cell_key = (round(point.x, 3), round(point.y, 3))

            if cell_key in seen:
                continue

            seen.add(cell_key)
            candidates.append((point.x, point.y))

        signature = (
            frame_id,
            tuple(
                (round(x, 3), round(y, 3))
                for x, y in candidates
            ),
        )

        if signature == self.signature:
            return

        self.frame_id = frame_id
        self.candidates = candidates
        self.signature = signature
        self.completed_signature = None

        # Old paths must not remain visible after goals change.
        self.clear_path()

        self.report_status(
            f"Received {len(candidates)} unique frontier goals"
        )

    def tick(self):
        if self.busy:
            return

        if not self.candidates:
            self.report_status("Waiting for frontier goals...")
            return

        if self.signature == self.completed_signature:
            return

        if not self.action_client.server_is_ready():
            self.report_status(
                "Nav2 Planner unavailable; waiting for "
                "/compute_path_to_pose"
            )
            return

        self.busy = True
        self.active_signature = self.signature
        self.current_index = 0
        self.success_count = 0

        self.report_status(
            "Nav2 Planner available. Starting path validation."
        )

        self.send_next_goal()

    def send_next_goal(self):
        if self.active_signature != self.signature:
            self.busy = False
            return

        if self.current_index >= len(self.candidates):
            self.completed_signature = self.active_signature
            self.busy = False

            self.report_status(
                f"Validation finished: "
                f"{self.success_count}/"
                f"{len(self.candidates)} paths valid"
            )
            return

        x, y = self.candidates[self.current_index]

        request = ComputePathToPose.Goal()

        pose = PoseStamped()
        pose.header.frame_id = self.frame_id
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.w = 1.0

        request.goal = pose
        request.planner_id = self.planner_id

        # Nav2 obtains the current robot pose using TF.
        request.use_start = False

        self.get_logger().info(
            f"Checking goal {self.current_index + 1}/"
            f"{len(self.candidates)}: ({x:.2f}, {y:.2f})"
        )

        future = self.action_client.send_goal_async(request)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        try:
            handle = future.result()
        except Exception as exc:
            self.get_logger().error(
                f"Planner goal request failed: {exc}"
            )
            self.advance()
            return

        if self.active_signature != self.signature:
            self.busy = False
            return

        if not handle.accepted:
            self.get_logger().warn(
                "Planner rejected the goal"
            )
            self.advance()
            return

        future = handle.get_result_async()
        future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        if self.active_signature != self.signature:
            self.busy = False
            return

        try:
            response = future.result()
            result = response.result
        except Exception as exc:
            self.get_logger().error(
                f"Planner result failed: {exc}"
            )
            self.advance()
            return

        success = (
            response.status == GoalStatus.STATUS_SUCCEEDED
            and result.error_code == ComputePathToPose.Result.NONE
            and len(result.path.poses) > 0
        )

        if success:
            length = self.path_length(result.path)

            self.success_count += 1

            self.get_logger().info(
                f"VALID PATH: goal "
                f"{self.current_index + 1}, "
                f"length={length:.2f} m, "
                f"poses={len(result.path.poses)}"
            )

            # Visualize the first successful route.
            if self.success_count == 1:
                self.path_pub.publish(result.path)

        else:
            self.get_logger().warn(
                f"INVALID PATH: goal "
                f"{self.current_index + 1}, "
                f"status={response.status}, "
                f"error_code={result.error_code}, "
                f"message={result.error_msg}"
            )

        self.advance()

    def advance(self):
        if self.active_signature != self.signature:
            self.busy = False
            return

        self.current_index += 1
        self.send_next_goal()

    @staticmethod
    def path_length(path):
        total = 0.0

        poses = path.poses

        for previous, current in zip(poses, poses[1:]):
            a = previous.pose.position
            b = current.pose.position

            total += math.hypot(
                b.x - a.x,
                b.y - a.y,
            )

        return total


def main(args=None):
    rclpy.init(args=args)

    node = FrontierPathChecker()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
