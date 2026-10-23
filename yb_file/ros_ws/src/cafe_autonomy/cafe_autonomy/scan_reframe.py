"""Canonicalize scan frame without unsupported SDF gz_frame_id metadata."""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
class ScanReframe(Node):
    def __init__(self):
        super().__init__('scan_reframe')
        self.declare_parameter('robot_id','robot_01')
        robot=self.get_parameter('robot_id').value
        self.pub=self.create_publisher(LaserScan,'scan',10)
        self.create_subscription(LaserScan,'scan_raw',self.cb,10)
        self.frame=robot+'/laser_link'
    def cb(self,msg):
        msg.header.frame_id=self.frame
        self.pub.publish(msg)
def main():
    rclpy.init();n=ScanReframe()
    try:rclpy.spin(n)
    finally:n.destroy_node();rclpy.shutdown()
