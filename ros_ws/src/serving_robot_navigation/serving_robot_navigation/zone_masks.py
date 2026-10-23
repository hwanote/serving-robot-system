"""Read-only Nav2 semantic masks; unreserved exclusive zones stay prohibited."""
from pathlib import Path
import json,math,time
import cv2,numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.msg import CostmapFilterInfo
from std_msgs.msg import String
from .semantic_inputs import zones,contains

class Masks(Node):
    def __init__(self):
        super().__init__("zone_masks")
        import yaml
        path=Path(self.declare_parameter("map_yaml","").value)
        self.meta=yaml.safe_load(path.read_text())
        image=cv2.imread(str(path.parent/self.meta["image"]),0)
        if image is None:raise ValueError("Part 1 map image is unreadable")
        self.height,self.width=image.shape
        self.zones=zones(self.declare_parameter("zones_file","").value)
        self.grants=[];self.grant_time=0.;self.previous=None
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
        self.outputs=[]
        for name in ("keepout","slow"):
            self.outputs.append((self.create_publisher(OccupancyGrid,name+"_mask",qos),self.create_publisher(CostmapFilterInfo,name+"_filter_info",qos),name))
        self.create_subscription(String,"zone_permission",self.permission,10)
        self.create_timer(.2,self.publish);self.publish()
    def permission(self,msg):
        try:self.grants=json.loads(msg.data)["granted"];self.grant_time=time.monotonic()
        except (ValueError,KeyError,TypeError):self.grants=[]
    def publish(self):
        grants=tuple(sorted(self.grants)) if time.monotonic()-self.grant_time<1. else ()
        if grants==self.previous:return
        self.previous=grants
        ox,oy,angle=self.meta["origin"];res=self.meta["resolution"]
        yy,xx=np.mgrid[:self.height,:self.width]
        x=ox+(xx+.5)*res*math.cos(angle)-(yy+.5)*res*math.sin(angle)
        y=oy+(xx+.5)*res*math.sin(angle)+(yy+.5)*res*math.cos(angle)
        for pub,ipub,name in self.outputs:
            mask=np.zeros((self.height,self.width),dtype=np.int8)
            for zone in self.zones:
                keep=zone["type"]=="no_go" or zone["type"]=="exclusive" and zone["id"] not in grants
                if name=="keepout" and not keep or name=="slow" and zone["type"]!="slow":continue
                inside=np.array([contains(zone,float(a),float(b)) for a,b in zip(x.flat,y.flat)]).reshape(x.shape)
                mask[inside]=100 if name=="keepout" else round(100*(1-float(zone.get("speed_ratio",.4))))
            msg=OccupancyGrid();msg.header.frame_id="map";msg.header.stamp=self.get_clock().now().to_msg()
            msg.info.width=self.width;msg.info.height=self.height;msg.info.resolution=res
            msg.info.origin.position.x=ox;msg.info.origin.position.y=oy;msg.info.origin.orientation.z=math.sin(angle/2);msg.info.origin.orientation.w=math.cos(angle/2)
            msg.data=mask.flatten().tolist()
            info=CostmapFilterInfo();info.header=msg.header;info.type=0 if name=="keepout" else 1
            info.filter_mask_topic=self.get_namespace()+"/"+name+"_mask";info.base=0. if name=="keepout" else 100.;info.multiplier=1. if name=="keepout" else -1.
            pub.publish(msg);ipub.publish(info)

def main():
    rclpy.init();node=Masks()
    try:rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
