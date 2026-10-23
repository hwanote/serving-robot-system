"""Robust static-surface registration used only for foreground classification.

Does not alter TF, the SLAM map, robot localization, or tracked object positions.
"""
import numpy as np
import math

def closest(points,shapes):
 best=np.full(len(points),np.inf);targets=np.zeros_like(points)
 for cx,cy,a,kind,hx,hy in shapes:
  c,s=math.cos(a),math.sin(a);q=points-np.array([cx,cy]);u=q[:,0]*c+q[:,1]*s;v=-q[:,0]*s+q[:,1]*c
  if kind=='circle':
   d=np.maximum(np.hypot(u,v),1e-9);tu=u*hx/d;tv=v*hx/d
  else:
   tu=np.clip(u,-hx,hx);tv=np.clip(v,-hy,hy);inside=(np.abs(u)<=hx)&(np.abs(v)<=hy)
   face_x=(hx-np.abs(u))<(hy-np.abs(v));ix=inside&face_x;iy=inside&~face_x
   tu[ix]=np.where(u[ix]>=0,hx,-hx);tv[iy]=np.where(v[iy]>=0,hy,-hy)
  target=np.column_stack((cx+tu*c-tv*s,cy+tu*s+tv*c));d=np.sum((target-points)**2,axis=1);better=d<best
  best[better]=d[better];targets[better]=target[better]
 return np.sqrt(best),targets

def register_static(points,shapes,max_shift=.4,max_yaw=.2,iterations=6,match_limit=.6):
 if len(points)<30 or not shapes:return points,{'accepted':False}
 original=points.copy();current=points.copy();centre=np.mean(points,axis=0);rotation=np.eye(2);translation=np.zeros(2)
 for _ in range(iterations):
  residual,target=closest(current,shapes);valid=residual<match_limit
  if np.count_nonzero(valid)<30:break
  valid&=residual<=np.percentile(residual[valid],65)
  if np.count_nonzero(valid)<20:break
  valid&=residual>1e-6
  if np.count_nonzero(valid)<10:break
  source=current[valid];dest=target[valid];error=source-dest
  normals=error/np.maximum(np.linalg.norm(error,axis=1,keepdims=True),1e-9)
  local=source-centre
  derivative=-local[:,1]*normals[:,0]+local[:,0]*normals[:,1]
  matrix=np.column_stack((normals,derivative))
  delta,_,_,_=np.linalg.lstsq(matrix,-np.linalg.norm(error,axis=1),rcond=None)
  delta[:2]=np.clip(delta[:2],-.15,.15);angle_step=float(np.clip(delta[2],-.05,.05))
  c,s=math.cos(angle_step),math.sin(angle_step);r=np.array([[c,-s],[s,c]])
  t=centre+delta[:2]-r@centre
  current=current@r.T+t;rotation=r@rotation;translation=r@translation+t
 angle=math.atan2(rotation[1,0],rotation[0,0]);shift=np.linalg.norm(rotation@centre+translation-centre)
 residual,_=closest(current,shapes);accepted=bool(abs(angle)<=max_yaw and shift<=max_shift)
 return (current if accepted else original),{'accepted':accepted,'yaw_rad':angle,'shift_m':float(shift),'median_surface_error_m':float(np.median(residual)),'rotation':rotation.tolist(),'translation':translation.tolist()}
