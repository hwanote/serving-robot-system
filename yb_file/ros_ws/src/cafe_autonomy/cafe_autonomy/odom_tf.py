"""Broadcast odom TF directly from Gazebo's independent per-robot odometry."""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class OdomTF(Node):
    def __init__(self):
        super().__init__('odom_tf')
        self.declare_parameter('robot_id','robot_01')
        self.robot=self.get_parameter('robot_id').value
        self.tf=TransformBroadcaster(self)
        self.create_subscription(Odometry,'odom',self.cb,10)
    def cb(self,msg):
        t=TransformStamped();t.header=msg.header
        t.header.frame_id=self.robot+'/odom'
        t.child_frame_id=self.robot+'/base_footprint'
        t.transform.translation.x=msg.pose.pose.position.x
        t.transform.translation.y=msg.pose.pose.position.y
        t.transform.translation.z=0.0
        t.transform.rotation=msg.pose.pose.orientation
        self.tf.sendTransform(t)

def main():
    rclpy.init();n=OdomTF()
    try:rclpy.spin(n)
    finally:n.destroy_node();rclpy.shutdown()
