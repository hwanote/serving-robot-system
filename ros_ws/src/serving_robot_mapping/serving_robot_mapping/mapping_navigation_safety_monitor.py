"""Read-only ROS adapter for Mapping navigation safety.

Never publishes velocity, sends navigation goals,
or clears collision latches.
"""

import json
import math
import time

from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from tf2_ros import TransformException

from serving_robot_mapping.mapping_navigation_safety import (
    MappingNavigationSafety,
    MappingSafetySnapshot,
)


class MappingNavigationSafetyMonitor:
    """Re-evaluate live safety evidence on every call."""

    def __init__(
        self,
        node,
        tf_buffer,
        robot_id="robot_01",
        lidar_only=False,
    ):
        if robot_id != "robot_01":
            raise ValueError(
                "Mapping policy currently supports robot_01 only"
            )

        if type(lidar_only) is not bool:
            raise ValueError("lidar_only must be bool")

        self.lidar_only = lidar_only
        self.node = node
        self.tf_buffer = tf_buffer
        self.received = {}
        self.subscriptions = []

        self.slam_client = node.create_client(
            GetState,
            "/slam_toolbox/get_state",
        )
        self.slam_future = None
        self.slam_request_at = None
        self.slam_reply_at = None
        self.slam_is_active = False

        self.tf_valid_since = None
        self.last_tf_ages = (None, None)
        self._last_sim_value = None
        self._last_sim_progress = None
        self.last_snapshot = None
        self.last_result = (False, "NOT_EVALUATED")

        normal = QoSProfile(depth=10)
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        root = f"/{robot_id}"
        self.collision_topic = f"{root}/collision_latched"
        self.obstacle_topic = f"{root}/obstacle_hold"

        self._watch(
            LaserScan,
            f"{root}/scan",
            "scan",
            qos_profile_sensor_data,
            lambda msg: True,
        )
        self._watch(
            Odometry,
            f"{root}/odom",
            "odom",
            qos_profile_sensor_data,
            lambda msg: True,
        )
        self._watch(
            OccupancyGrid,
            "/map",
            "map",
            latched,
            lambda msg: (
                msg.info.width > 0
                and msg.info.height > 0
            ),
        )
        self._watch(
            String,
            f"{root}/health",
            "health",
            normal,
            lambda msg: json.loads(msg.data),
        )
        if not self.lidar_only:
            self._watch(
                String,
                f"{root}/person_limit",
                "person_limit",
                normal,
                lambda msg: json.loads(msg.data),
            )
        self._watch(
            Bool,
            self.collision_topic,
            "collision",
            latched,
            lambda msg: bool(msg.data),
        )
        self._watch(
            Bool,
            self.obstacle_topic,
            "obstacle",
            latched,
            lambda msg: bool(msg.data),
        )

    def _watch(
        self,
        msg_type,
        topic,
        name,
        qos,
        convert,
    ):
        def callback(msg):
            try:
                value = convert(msg)
            except (TypeError, ValueError, KeyError):
                value = None

            self.received[name] = (
                time.monotonic(),
                value,
            )

        self.subscriptions.append(
            self.node.create_subscription(
                msg_type,
                topic,
                callback,
                qos,
            )
        )

    def _age(self, name, now):
        item = self.received.get(name)
        if item is None:
            return None
        return now - item[0]

    def _value(self, name):
        item = self.received.get(name)
        return None if item is None else item[1]

    def _poll_slam(self, now):
        if self.slam_future is not None:
            if self.slam_future.done():
                try:
                    reply = self.slam_future.result()
                    self.slam_is_active = (
                        reply is not None
                        and reply.current_state.label == "active"
                    )
                    self.slam_reply_at = now
                except Exception:
                    self.slam_is_active = False
                    self.slam_reply_at = None

                self.slam_future = None

            elif now - self.slam_request_at > 3.0:
                self.slam_future.cancel()
                self.slam_future = None
                self.slam_is_active = False
                self.slam_reply_at = None

        if (
            self.slam_future is None
            and (
                self.slam_request_at is None
                or now - self.slam_request_at >= 2.0
            )
            and self.slam_client.service_is_ready()
        ):
            try:
                self.slam_future = self.slam_client.call_async(
                    GetState.Request()
                )
                self.slam_request_at = now
            except Exception:
                self.slam_is_active = False
                self.slam_reply_at = None

    def _clock_progress(self, now):
        """Reject a paused, unavailable or backward ROS clock."""
        sim_now = (
            self.node.get_clock().now().nanoseconds * 1e-9
        )

        if not math.isfinite(sim_now) or sim_now <= 0:
            self._last_sim_value = None
            self._last_sim_progress = None
            self.tf_valid_since = None
            return False, sim_now

        previous = self._last_sim_value

        if (
            previous is None
            or sim_now < previous - 0.001
        ):
            self._last_sim_value = sim_now
            self._last_sim_progress = None
            self.tf_valid_since = None
            return False, sim_now

        if sim_now > previous + 0.001:
            self._last_sim_value = sim_now
            self._last_sim_progress = now

        fresh = (
            self._last_sim_progress is not None
            and now - self._last_sim_progress <= 2.0
        )

        if not fresh:
            self.tf_valid_since = None

        return fresh, sim_now

    def _check_tf(self, now):
        self.last_tf_ages = (None, None)
        valid = False

        try:
            map_odom = self.tf_buffer.lookup_transform(
                "map",
                "odom",
                Time(),
            )
            odom_base = self.tf_buffer.lookup_transform(
                "odom",
                "base_footprint",
                Time(),
            )
            map_base = self.tf_buffer.lookup_transform(
                "map",
                "base_footprint",
                Time(),
            )

            sim_now = (
                self.node.get_clock().now().nanoseconds
                * 1e-9
            )

            def age(tf):
                stamp = tf.header.stamp
                stamp_s = (
                    stamp.sec
                    + stamp.nanosec * 1e-9
                )
                return sim_now - stamp_s

            self.last_tf_ages = (
                age(map_odom),
                age(odom_base),
            )

            position = map_base.transform.translation
            orientation = map_base.transform.rotation

            quat_norm_sq = (
                orientation.x ** 2
                + orientation.y ** 2
                + orientation.z ** 2
                + orientation.w ** 2
            )

            valid = (
                sim_now > 0
                and all(
                    math.isfinite(value)
                    and -0.30 <= value <= 0.50
                    for value in self.last_tf_ages
                )
                and all(
                    math.isfinite(value)
                    for value in (
                        position.x,
                        position.y,
                        position.z,
                        orientation.x,
                        orientation.y,
                        orientation.z,
                        orientation.w,
                    )
                )
                and 0.5 < quat_norm_sq < 1.5
            )

        except (TransformException, ValueError, TypeError):
            pass

        if valid:
            if self.tf_valid_since is None:
                self.tf_valid_since = now
        else:
            self.tf_valid_since = None

        return (
            valid
            and now - self.tf_valid_since >= 1.0
        )

    def evaluate(self):
        """Return (allowed, reason) using current ROS evidence."""

        now = time.monotonic()

        self._poll_slam(now)
        clock_fresh, sim_now = self._clock_progress(now)
        tf_stable = self._check_tf(now) and clock_fresh

        # A cached False cannot authorize movement when the
        # corresponding safety publisher is missing or duplicated.
        collision = (
            self._value("collision")
            if self.node.count_publishers(
                self.collision_topic
            ) == 1
            else None
        )

        obstacle = (
            self._value("obstacle")
            if self.node.count_publishers(
                self.obstacle_topic
            ) == 1
            else None
        )

        # Health's JSON 'time' is ROS simulation time.
        # Require both a fresh simulation stamp and a bounded
        # wall-clock receipt age, even when Gazebo runs slowly.
        health_publishers = self.node.count_publishers(
            "/robot_01/health"
        )
        health = self._value("health")
        health_sim_age = None

        if (
            health_publishers == 1
            and isinstance(health, dict)
            and clock_fresh
        ):
            try:
                stamp = health["time"]
                if isinstance(stamp, bool):
                    raise TypeError("Boolean timestamp")

                stamp = float(stamp)

                if math.isfinite(stamp):
                    health_sim_age = sim_now - stamp
            except (KeyError, TypeError, ValueError):
                pass

        self.last_snapshot = MappingSafetySnapshot(
            slam_active=(
                self.slam_is_active
                and self.slam_reply_at is not None
                and now - self.slam_reply_at <= 5.0
            ),
            tf_stable=tf_stable,
            scan_age_s=(
                self._age("scan", now)
                if self._value("scan") is True
                else None
            ),
            odom_age_s=(
                self._age("odom", now)
                if self._value("odom") is True
                else None
            ),
            map_age_s=(
                self._age("map", now)
                if self._value("map") is True
                else None
            ),
            map_odom_tf_age_s=self.last_tf_ages[0],
            odom_base_tf_age_s=self.last_tf_ages[1],
            health_age_s=(
                self._age("health", now)
                if health_publishers == 1
                else None
            ),
            health_sim_age_s=health_sim_age,
            health=(
                health if health_publishers == 1 else None
            ),
            person_limit_age_s=self._age(
                "person_limit",
                now,
            ),
            person_limit=self._value("person_limit"),
            collision_latched=collision,
            obstacle_hold=obstacle,
        )

        self.last_result = MappingNavigationSafety.evaluate(
            self.last_snapshot
        , lidar_only=self.lidar_only)

        return self.last_result

    def is_safe(self):
        """Callback suitable for a future Nav2 safety gate."""
        return self.evaluate()[0] is True
