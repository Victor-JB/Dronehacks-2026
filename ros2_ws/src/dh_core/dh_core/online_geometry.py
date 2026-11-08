"""Capture-time rigid poses, explicit camera calibration and coverage queries."""
import bisect
import math
import numpy as np
from .camera_geom import quat_to_rot,optical_to_body


def nominal_transform_for_tests(tilt_deg,x=.1053,z=.0304):
    """SENSOR MODEL/TEST fixture only. ROS uses measured TF, never this fallback."""
    basis=np.eye(3)
    t=np.array([x,0,z])
    R=np.asarray(optical_to_body(basis[:,0],basis[:,1],basis[:,2],math.radians(tilt_deg),x,z))-t[:,None]
    return R,t


class RigidHistory:
    def __init__(self,span=4.,max_gap=.3): self.span,self.max_gap=span,max_gap; self.samples=[]

    def add(self,stamp,position,quaternion):
        p,q=np.asarray(position,float),np.asarray(quaternion,float)
        if stamp<=0 or not math.isfinite(stamp) or p.shape!=(3,) or q.shape!=(4,) or not np.isfinite(np.r_[p,q]).all() or abs(np.linalg.norm(q)-1)>.01: return False
        ts=[s[0] for s in self.samples]; k=bisect.bisect_left(ts,stamp)
        row=(stamp,p.copy(),q/np.linalg.norm(q))
        if k<len(ts) and ts[k]==stamp: self.samples[k]=row
        else: self.samples.insert(k,row)
        while self.samples and self.samples[-1][0]-self.samples[0][0]>self.span: self.samples.pop(0)
        return True

    def at(self,stamp):
        if not self.samples or not self.samples[0][0]<=stamp<=self.samples[-1][0]: return None
        ts=[s[0] for s in self.samples]; k=bisect.bisect_left(ts,stamp)
        if ts[k]==stamp: _,p,q=self.samples[k]; return p.copy(),quat_to_rot(*q)
        t0,p0,q0=self.samples[k-1]; t1,p1,q1=self.samples[k]
        if t1-t0>self.max_gap: return None
        a=(stamp-t0)/(t1-t0)
        if q0@q1<0: q1=-q1
        dot=float(np.clip(q0@q1,-1,1)); theta=math.acos(dot)
        q=(1-a)*q0+a*q1 if theta<1e-6 else (math.sin((1-a)*theta)*q0+math.sin(a*theta)*q1)/math.sin(theta)
        return (1-a)*p0+a*p1,quat_to_rot(*(q/np.linalg.norm(q)))


def camera_tilt(R_body_optical):
    forward=np.asarray(R_body_optical)[:,2]
    return math.degrees(math.atan2(-forward[2],math.hypot(forward[0],forward[1])))


def validate_mount(R,expected_deg=0.,tolerance_deg=3.):
    R=np.asarray(R,float)
    return (R.shape==(3,3) and np.isfinite(R).all() and np.allclose(R.T@R,np.eye(3),atol=.001)
            and abs(np.linalg.det(R)-1)<.001 and abs(camera_tilt(R)-expected_deg)<=tolerance_deg)


def deproject(depth,K,pixels):
    K=np.asarray(K,float).reshape(3,3)
    uv=np.asarray(pixels,float)
    if not np.isfinite(K).all() or K[0,0]<=0 or K[1,1]<=0: raise ValueError('Invalid camera intrinsics')
    return np.column_stack([(uv[:,0]-K[0,2])/K[0,0],(uv[:,1]-K[1,2])/K[1,1],np.ones(len(uv))])*np.asarray(depth)[:,None]


def coverage(K,width,height,tilt_deg,camera_height):
    K=np.asarray(K).reshape(3,3)
    up=math.degrees(math.atan(K[1,2]/K[1,1]))-tilt_deg
    down=math.degrees(math.atan((height-1-K[1,2])/K[1,1]))+tilt_deg
    return dict(upward_degrees=up,downward_degrees=down,
                nearest_floor_forward_m=camera_height/math.tan(math.radians(down)),
                nadir_visible=down>=90.,horizontal_fov_degrees=math.degrees(math.atan(K[0,2]/K[0,0])+math.atan((width-1-K[0,2])/K[0,0])))


def register_depth_to_color(depth,K_depth,K_color,R_color_depth,t_color_depth,color_shape):
    """Calibrated z-buffer registration, no assumption that an RGB-labelled depth is aligned."""
    d=np.asarray(depth,float); ys,xs=np.nonzero(np.isfinite(d)&(d>.1)&(d<8.))
    points=deproject(d[ys,xs],K_depth,np.column_stack([xs,ys]))
    points=points@np.asarray(R_color_depth).T+np.asarray(t_color_depth)
    points=points[points[:,2]>.1]
    K=np.asarray(K_color).reshape(3,3)
    u=np.rint(K[0,0]*points[:,0]/points[:,2]+K[0,2]).astype(int)
    v=np.rint(K[1,1]*points[:,1]/points[:,2]+K[1,2]).astype(int)
    h,w=color_shape
    good=(u>=0)&(u<w)&(v>=0)&(v<h)
    flat=np.full(h*w,np.inf)
    np.minimum.at(flat,v[good]*w+u[good],points[good,2])
    flat[~np.isfinite(flat)]=np.nan
    return flat.reshape(h,w)
