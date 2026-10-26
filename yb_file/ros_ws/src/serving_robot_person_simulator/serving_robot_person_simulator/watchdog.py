"""Independent heartbeat timeout watchdog for person-model commands only.

On manager death, independently publishes zero ROS person commands. No robot
velocity topics are referenced, and the manager can be restarted separately.
"""
import time
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Empty

class Watchdog(Node):
    def __init__(self):
        super().__init__('person_simulator_watchdog')
        self.last_beat=time.monotonic()
        self.had_beat=False
        self.stale_reported=False
        self.person_cmd_pubs=[self.create_publisher(Twist,
            f'/model/test_person_{i:02d}/cmd_vel',10) for i in range(1,9)]
        self.create_subscription(Empty,'/person_simulator/heartbeat',self.heartbeat,10)
        self.create_timer(0.2,self.tick)

    def heartbeat(self,_):
        self.last_beat=time.monotonic()
        self.had_beat=True
        self.stale_reported=False

    def tick(self):
        if time.monotonic()-self.last_beat<=0.8:
            return
        # No manager commands recently: zero every owned pedestrian topic.
        for pub in self.person_cmd_pubs:
            pub.publish(Twist())
        if self.had_beat and not self.stale_reported:
            self.get_logger().error('Manager heartbeat stale: all person commands forced to zero')
            self.stale_reported=True

def main():
    rclpy.init()
    node=Watchdog()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__=='__main__': main()
