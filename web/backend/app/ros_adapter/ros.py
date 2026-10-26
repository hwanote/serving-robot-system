"""rclpy node for one robot: subscribes mission_status / health / alarm, reads TF
map->base_footprint, publishes mission_cmd (all JSON in std_msgs/String).
Every robot gets its own rclpy context, so its own ROS_DOMAIN_ID (the robots'
TF frames are not prefixed, so they must not share one ROS graph)."""
import json
import math
import threading


class RosAdapter:
    def __init__(self, robot, domain=None, use_sim_time=True):
        self.robot, self.domain, self.use_sim_time = robot, domain, use_sim_time

    def start(self, sink):
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from rclpy.parameter import Parameter
        from rclpy.time import Time
        from std_msgs.msg import String
        from tf2_ros import Buffer, TransformException, TransformListener

        name = self.robot
        ctx = Context()
        rclpy.init(context=ctx, domain_id=self.domain)
        node = Node('backend_' + name, context=ctx, parameter_overrides=[
            Parameter('use_sim_time', Parameter.Type.BOOL, self.use_sim_time)])
        self.node, self._String = node, String
        r = '/%s/' % name

        def parse(cb):
            def handler(msg):
                try:
                    cb(name, json.loads(msg.data))
                except (ValueError, TypeError) as e:
                    node.get_logger().warning('bad JSON: %s' % e)
            return handler

        node.create_subscription(String, r + 'mission_status', parse(sink.on_status), 10)
        node.create_subscription(String, r + 'health', parse(sink.on_health), 10)
        node.create_subscription(String, r + 'alarm', parse(sink.on_alarm), 10)
        self.pub = node.create_publisher(String, r + 'mission_cmd', 10)
        buf = Buffer()
        TransformListener(buf, node)

        clock = {'last': None}

        def pose_tick():
            # Gazebo was restarted: simulation time went back. TF drops such data as
            # "from the past", so forget what it knows and start over.
            now = node.get_clock().now().nanoseconds
            if clock['last'] is not None and now < clock['last'] - 1_000_000_000:
                node.get_logger().warning('clock went back (%.1f s -> %.1f s): resetting TF'
                                          % (clock['last'] / 1e9, now / 1e9))
                buf.clear()
            clock['last'] = now
            try:
                tf = buf.lookup_transform('map', 'base_footprint', Time())
            except TransformException:
                return
            q, p = tf.transform.rotation, tf.transform.translation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            sink.on_pose(name, p.x, p.y, yaw)

        node.create_timer(0.2, pose_tick)
        ex = SingleThreadedExecutor(context=ctx)
        ex.add_node(node)
        threading.Thread(target=ex.spin, daemon=True).start()
        self._ex, self._ctx = ex, ctx

    def stop(self):
        try:
            self._ex.shutdown()
            self._ctx.try_shutdown()
        except Exception:
            pass

    def send_cmd(self, cmd):
        self.pub.publish(self._String(data=json.dumps(cmd)))
