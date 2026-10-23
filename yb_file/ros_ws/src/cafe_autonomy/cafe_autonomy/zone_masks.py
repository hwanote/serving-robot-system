"""Separate Nav2 filter masks; never changes source SLAM occupancy pixels."""
from pathlib import Path
import math,yaml
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.msg import CostmapFilterInfo

class Masks(Node):
    def __init__(self):
        super().__init__('zone_masks')
        mp=Path(self.declare_parameter('map_yaml','').value)
        zones=yaml.safe_load(Path(self.declare_parameter('zones_file','').value).read_text())
        meta=yaml.safe_load(mp.read_text());im=cv2.imread(str(mp.parent/meta['image']),0)
        h,w=im.shape;res=meta['resolution'];ox,oy,angle=meta['origin']
        xx,yy=np.meshgrid(np.arange(w),np.arange(h));xx=(xx+.5)*res;yy=(yy+.5)*res
        mx=ox+xx*math.cos(angle)-yy*math.sin(angle);my=oy+xx*math.sin(angle)+yy*math.cos(angle)
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
        self.pubs=[];self.msgs=[]
        for typ,key in [(0,'no_go'),(1,'slow')]:
            mask=np.zeros((h,w),dtype=np.int8)
            for zone in zones.get(key,{}).values():
                bounds=zone['bounds'] if isinstance(zone,dict) else zone
                inside=(mx>=bounds[0])&(mx<=bounds[1])&(my>=bounds[2])&(my<=bounds[3])
                mask[inside]=100 if typ==0 else int(round(100*(1-float(zone.get('speed_ratio',.4)))))
            prefix='keepout' if typ==0 else 'slow'
            grid=OccupancyGrid();grid.header.frame_id='map';grid.info.resolution=res;grid.info.width=w;grid.info.height=h
            grid.info.origin.position.x=ox;grid.info.origin.position.y=oy;grid.info.origin.orientation.z=math.sin(angle/2);grid.info.origin.orientation.w=math.cos(angle/2)
            grid.data=mask.flatten().tolist()
            info=CostmapFilterInfo();info.header.frame_id='map';info.type=typ;info.filter_mask_topic=self.get_namespace()+'/'+prefix+'_mask';info.base=0. if typ==0 else 100.;info.multiplier=1. if typ==0 else -1.
            for name,msg,cls in [(prefix+'_mask',grid,OccupancyGrid),(prefix+'_filter_info',info,CostmapFilterInfo)]:
                self.pubs.append(self.create_publisher(cls,name,qos));self.msgs.append(msg)
        self.publish()
    def publish(self):
        for pub,msg in zip(self.pubs,self.msgs):msg.header.stamp=self.get_clock().now().to_msg();pub.publish(msg)

def main():
    rclpy.init();n=Masks()
    try:rclpy.spin(n)
    finally:n.destroy_node();rclpy.shutdown()
