"""Independent measured rotation limits; translation, direction and stop resets."""
import math
from .policy import norm_angle
class RotationGuard:
    def __init__(self,cfg):
        self.cfg=cfg;self.anchor=None;self.previous=None;self.total=0.;self.continuous=0.;self.sign=0;self.ended=None;self.reason=None
    def observe(self,x,y,yaw,angular,now):
        if self.anchor is None:self.anchor=(x,y)
        if math.hypot(x-self.anchor[0],y-self.anchor[1])>=self.cfg['stuck_distance_m']:
            self.anchor=(x,y);self.total=0.
            if self.reason=='CUMULATIVE_ROTATION_LIMIT':self.reason=None
        delta=norm_angle(yaw-self.previous) if self.previous is not None else 0.;self.previous=yaw
        self.total+=abs(delta)
        sign=1 if angular>self.cfg['rotation_stop_rps'] else -1 if angular< -self.cfg['rotation_stop_rps'] else 0
        if sign:
            if self.sign and sign!=self.sign:self.continuous=0.
            self.continuous+=abs(delta);self.sign=sign;self.ended=None
        else:
            if self.ended is None:self.ended=now
            if now-self.ended>=self.cfg['rotation_end_hold_s']:
                self.continuous=0.;self.sign=0
                if self.reason=='CONTINUOUS_ROTATION_LIMIT':self.reason=None
        if self.total>=self.cfg['cumulative_rotation_rad']:self.reason='CUMULATIVE_ROTATION_LIMIT'
        elif self.continuous>=self.cfg['continuous_rotation_rad']:self.reason='CONTINUOUS_ROTATION_LIMIT'
    def permits(self,requested,period=.05):
        if not requested:return True
        if self.reason:return False
        if self.total+abs(requested)*period>=self.cfg['cumulative_rotation_rad']:
            self.reason='CUMULATIVE_ROTATION_LIMIT';return False
        same=not self.sign or (requested>0)==(self.sign>0)
        if (self.continuous if same else 0.)+abs(requested)*period>=self.cfg['continuous_rotation_rad']:
            self.reason='CONTINUOUS_ROTATION_LIMIT';return False
        return True
    def snapshot(self):return {'cumulative_rad':self.total,'continuous_rad':self.continuous,'direction':self.sign,'reason':self.reason}
