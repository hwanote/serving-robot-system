"""State-aware bringup with bounded requests; never sends STARTUP to active nodes."""
import time
import rclpy
from rclpy.node import Node
from lifecycle_msgs.srv import GetState,ChangeState

class Guard(Node):
    def __init__(self):
        super().__init__('lifecycle_guard')
        self.names=self.declare_parameter('nodes',['map_server','amcl','planner_server','controller_server','behavior_server','bt_navigator','velocity_smoother','collision_monitor']).value
        self.lifecycle_clients={name:(self.create_client(GetState,name+'/get_state'),self.create_client(ChangeState,name+'/change_state')) for name in self.names}
        self.index=0;self.pending=None;self.sequence=0;self.not_before=0.
        self.create_timer(.25,self.tick)
    def tick(self):
        if self.index>=len(self.names):return
        if self.get_clock().now().nanoseconds*1e-9<self.not_before:return
        if self.pending:
            if time.monotonic()-self.pending[1]<3.:return
            self.sequence+=1;self.pending[0].cancel();self.pending=None
            self.get_logger().warning('Lifecycle reply timeout; re-reading state before retry')
        name=self.names[self.index];get,change=self.lifecycle_clients[name]
        if not get.service_is_ready():return
        self.sequence+=1;seq=self.sequence;f=get.call_async(GetState.Request());self.pending=(f,time.monotonic())
        def read_done(f):
            if seq!=self.sequence:return
            self.pending=None
            try:state=f.result().current_state.id
            except Exception:return
            if state==3:
                if name=='amcl':self.not_before=self.get_clock().now().nanoseconds*1e-9+2.
                self.get_logger().info(name+' ACTIVE');self.index+=1;return
            transition=1 if state==1 else 3 if state==2 else None
            if transition is None or not change.service_is_ready():return
            req=ChangeState.Request();req.transition.id=transition
            cf=change.call_async(req);self.pending=(cf,time.monotonic())
            def changed(cf):
                if seq!=self.sequence:return
                self.pending=None
                try:
                    if not cf.result().success:self.get_logger().warning(name+' transition rejected; will re-read state')
                    elif transition==1 and name in ('planner_server','controller_server'):
                        self.not_before=self.get_clock().now().nanoseconds*1e-9+2.
                except Exception:pass
            cf.add_done_callback(changed)
        f.add_done_callback(read_done)

def main():
    rclpy.init();node=Guard()
    try:rclpy.spin(node)
    finally:node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
