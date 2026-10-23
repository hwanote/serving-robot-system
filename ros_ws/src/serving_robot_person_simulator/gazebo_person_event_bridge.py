"""Simulation-only bridge from Gazebo poses to PersonSafetyEvent."""
import json
import math
import re
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from serving_robot_interfaces.msg import PersonSafetyEvent
from serving_robot_person_simulator.pose_cli import PoseStream


class GazeboPersonEventBridge(Node):
    def __init__(self):
        super().__init__('gazebo_person_event_bridge')

        self.stream = PoseStream('cafe', 'test_person_', 'robot_01')
        self.roster = None
        self.status_received = None
        self.last_state = None

        self.pub = self.create_publisher(
            PersonSafetyEvent,
            '/robot_01/person_safety_event_lab',
            10
        )
        self.sub = self.create_subscription(
            String, '/person_simulator/status',
            self.on_status, 10
        )
        self.timer = self.create_timer(0.1, self.tick)

    def on_status(self, msg):
        try:
            status = json.loads(msg.data)
            people = status['people']

            if (
                status.get('error') is not None
                or status.get('robot_pose_valid') is not True
                or status.get('gazebo_pose_stream_alive') is not True
                or not isinstance(people, list)
                or len(people) > 8
            ):
                raise ValueError('manager not ready')

            names = set()
            for person in people:
                name = person['name']
                if (
                    re.fullmatch(r'test_person_0[1-8]', name) is None
                    or person.get('position') is None
                    or name in names
                ):
                    raise ValueError('invalid person status')
                names.add(name)

            self.roster = names
            self.status_received = time.monotonic()

        except (ValueError, TypeError, KeyError):
            self.roster = None
            self.status_received = None

    def tick(self):
        now = time.monotonic()
        state = 'SENSOR_STALE'
        distance = 0.0
        confidence = 0.0

        if (
            self.stream.alive()
            and self.roster is not None
            and self.status_received is not None
            and now - self.status_received <= 3.0
        ):
            poses = self.stream.snapshot()

            def fresh(name):
                entry = poses.get(name)
                if entry is None or now - entry[1] > 1.5:
                    return None
                return entry[0]

            robot = fresh('robot_01')
            known_people = {
                name: fresh(name) for name in self.roster
            }

            stray_people = any(
                name.startswith('test_person_')
                and re.fullmatch(r'test_person_0[1-8]', name)
                and name not in self.roster
                and now - timestamp <= 1.5
                for name, (_, timestamp) in poses.items()
            )

            if (
                robot is not None
                and all(
                    p is not None for p in known_people.values()
                )
                and not stray_people
            ):
                nearest = min(
                    (
                        math.dist(robot, p)
                        for p in known_people.values()
                    ),
                    default=10000.0
                )

                # 로봇 중심에서 사람 표면까지의 근사 거리
                distance = max(0.0, nearest - 0.22)

                if distance <= 0.80:
                    state = 'STOP'
                elif distance <= 1.50:
                    state = 'SLOW'
                else:
                    state = 'NORMAL'

                confidence = 1.0

        event = PersonSafetyEvent()
        event.header.stamp = self.get_clock().now().to_msg()
        event.header.frame_id = 'world'
        event.robot_id = 'robot_01'
        event.state = state
        event.confidence = confidence
        event.distance_m = float(distance)
        self.pub.publish(event)

        if state != self.last_state:
            self.get_logger().info(
                f'person event: {state} '
                f'(distance={distance:.2f}m)'
            )
            self.last_state = state

    def close(self):
        self.stream.close()


def main():
    rclpy.init()
    node = None
    try:
        node = GazeboPersonEventBridge()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
