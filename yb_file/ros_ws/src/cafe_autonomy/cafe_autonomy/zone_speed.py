"""Apply the same Slow Zone policy to all commands, including final alignment."""
from pathlib import Path
import yaml
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Float64
from tf2_ros import Buffer,TransformListener
from .policy import in_bounds

class ZoneSpeed(Node):
    def __init__(self):
        super().__init__('zone_speed')
        self.robot=self.declare_parameter('robot_id','robot_01').value
        self.zones=yaml.safe_load(Path(self.declare_parameter('zones_file','').value).read_text()).get('slow',{})
        self.tf=Buffer();self.listener=TransformListener(self.tf,self);self.pub=self.create_publisher(Float64,'zone_speed_ratio',10)
        self.create_timer(.2,self.tick)
    def tick(self):
        ratio=1.
        try:
            t=self.tf.lookup_transform('map',self.robot+'/base_footprint',Time());p=(t.transform.translation.x,t.transform.translation.y)
            for zone in self.zones.values():
                if in_bounds(p,zone['bounds']):ratio=min(ratio,float(zone.get('speed_ratio',.4)))
        except Exception:ratio=0.
        self.pub.publish(Float64(data=max(0.,min(1.,ratio))))

def main():
    rclpy.init();n=ZoneSpeed()
    try:rclpy.spin(n)
    finally:n.destroy_node();rclpy.shutdown()
