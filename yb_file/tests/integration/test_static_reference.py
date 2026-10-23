from pathlib import Path
import sys,math,numpy as np
root=Path(__file__).resolve().parents[2];sys.path.insert(0,str(root/'ros_ws/src/cafe_autonomy'))
from cafe_autonomy.static_reference import register_static,closest

def test_static_matching_handles_pose_error_without_erasing_new_obstacle():
 shapes=[(-4.,0.,0.,'box',.06,3.),(4.,0.,0.,'box',.06,3.),(0.,3.,0.,'box',4.,.06),(0.,-3.,0.,'box',4.,.06)]
 t=np.linspace(-2.8,2.8,120);x=np.linspace(-3.8,3.8,120)
 walls=np.concatenate([np.column_stack((np.full_like(t,-3.94),t)),np.column_stack((np.full_like(t,3.94),t)),np.column_stack((x,np.full_like(x,2.94))),np.column_stack((x,np.full_like(x,-2.94)))])
 actor=np.column_stack((.2*np.cos(np.linspace(0,6.28,30)),.2*np.sin(np.linspace(0,6.28,30))))
 a=.10;r=np.array([[math.cos(a),-math.sin(a)],[math.sin(a),math.cos(a)]])
 points=np.concatenate([walls,actor])@r.T+np.array([.06,-.05]);fixed,meta=register_static(points,shapes)
 d,_=closest(fixed,shapes)
 assert meta['accepted'] and np.percentile(d[:len(walls)],95)<.15
 assert np.min(d[len(walls):])>2.
 assert np.array_equal(points,np.concatenate([walls,actor])@r.T+np.array([.06,-.05]))
