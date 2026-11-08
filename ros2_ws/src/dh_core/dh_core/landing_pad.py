"""Shared blue-square/orange-floor perception and bounded static-world tracking.

Requires genuinely COLOR, geometrically registered depth and capture-time TF.
Mono bgr8 is not evidence of color. No flight/land command is defined here.
"""
from dataclasses import dataclass
import math
import cv2
import numpy as np
from .online_geometry import deproject


@dataclass
class PadObservation:
    stamp: float
    centre: np.ndarray
    corners: np.ndarray
    std: float
    confidence: float


def detect_blue_pad(bgr,depth_m,K,R_world_optical,t_world_optical,stamp,
                    registered=False,depth_std=.02,min_side=.35,max_side=2.):
    if (not registered or not math.isfinite(depth_std) or not math.isfinite(stamp)
            or depth_std>.10 or depth_std<0 or stamp<=0): return None
    image=np.asarray(bgr); depth=np.asarray(depth_m)
    if image.ndim!=3 or image.shape[2]!=3 or depth.shape!=image.shape[:2]: return None
    if image.dtype!=np.uint8 or not np.isfinite(R_world_optical).all(): return None
    hsv=cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
    blue=cv2.inRange(hsv,(90,55,30),(135,255,255))
    orange=cv2.inRange(hsv,(3,70,30),(30,255,255))
    blue=cv2.morphologyEx(blue,cv2.MORPH_CLOSE,np.ones((3,3),np.uint8))
    contours,_=cv2.findContours(blue,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    candidates=[]
    R,t=np.asarray(R_world_optical,float),np.asarray(t_world_optical,float)
    if R.shape!=(3,3) or t.shape!=(3,) or not np.isfinite(t).all(): return None
    for contour in contours:
        area=cv2.contourArea(contour)
        if area<max(60,image.shape[0]*image.shape[1]*.0008): continue
        poly=cv2.approxPolyDP(contour,.025*cv2.arcLength(contour,True),True)
        if len(poly)!=4 or not cv2.isContourConvex(poly): continue
        uv=poly[:,0,:].astype(float)
        # Partial/border clipping must not fabricate unseen pad boundaries.
        if (uv<2).any() or (uv[:,0]>image.shape[1]-3).any() or (uv[:,1]>image.shape[0]-3).any(): continue
        mask=np.zeros(depth.shape,np.uint8); cv2.fillConvexPoly(mask,uv.astype(np.int32),255)
        ring=(cv2.dilate(mask,np.ones((13,13),np.uint8))>0)&(mask==0)
        if ring.sum()<20 or (orange[ring]>0).mean()<.40: continue
        inner=cv2.erode(mask,np.ones((5,5),np.uint8))>0
        ys,xs=np.nonzero(inner); good=np.isfinite(depth[ys,xs])&(depth[ys,xs]>.1)&(depth[ys,xs]<8.)
        if len(good)<30 or good.mean()<.65: continue
        ys,xs=ys[good],xs[good]; stride=max(1,len(xs)//1200); ys,xs=ys[::stride],xs[::stride]
        optical=deproject(depth[ys,xs],K,np.column_stack([xs,ys]))
        world=optical@R.T+t
        centre=world.mean(axis=0)
        _,_,vt=np.linalg.svd(world-centre,full_matrices=False); normal=vt[-1]
        residual=np.abs((world-centre)@normal)
        if abs(normal[2])<.95 or np.percentile(residual,90)>max(.04,2*depth_std): continue
        # Intersect corner rays with the fitted floor plane, not median-z corners.
        rays=deproject(np.ones(4),K,uv)@R.T
        denom=rays@normal
        if (abs(denom)<1e-6).any(): continue
        distances=((centre-t)@normal)/denom
        if (distances<=0).any(): continue
        corners=t+rays*distances[:,None]
        sides=np.linalg.norm(corners-np.roll(corners,1,axis=0),axis=1)
        if sides.min()<min_side or sides.max()>max_side or sides.max()/sides.min()>1.35: continue
        u=corners[1]-corners[0]; v=corners[2]-corners[1]
        if abs(u@v)/(np.linalg.norm(u)*np.linalg.norm(v))>.25: continue
        confidence=float(min(1.,good.mean())*(1.-min(.5,depth_std*3)))
        candidates.append((area,PadObservation(stamp,corners.mean(axis=0),corners,float(depth_std+np.std(residual)),confidence)))
    candidates.sort(key=lambda c:c[0],reverse=True)
    if not candidates or (len(candidates)>1 and candidates[1][0]>.6*candidates[0][0]): return None
    return candidates[0][1]


class PadTrack:
    def __init__(self,max_age=2.): self.observation=None; self.count=0; self.max_age=max_age

    def update(self,obs,now):
        if (obs is None or not .7<=obs.confidence<=1. or not 0<=obs.std<=.1
                or np.asarray(obs.corners).shape!=(4,3) or np.asarray(obs.centre).shape!=(3,)
                or not -.02<=now-obs.stamp<=.5 or not np.isfinite(np.r_[obs.centre,np.asarray(obs.corners).ravel()]).all()): return False
        old=self.observation
        if old is not None and (obs.stamp<=old.stamp or np.linalg.norm(obs.centre-old.centre)>.25): return False
        if old is not None and obs.stamp-old.stamp>self.max_age: self.count=0
        self.observation=PadObservation(obs.stamp,np.array(obs.centre,copy=True),np.array(obs.corners,copy=True),obs.std,obs.confidence)
        self.count+=1; return True

    def estimate(self,now,pos_std=0.):
        o=self.observation
        if o is None or self.count<3 or not 0<=now-o.stamp<=self.max_age: return None
        sigma=o.std+max(0.,pos_std)+.03*(now-o.stamp)
        if sigma>.12: return None
        return PadObservation(o.stamp,o.centre.copy(),o.corners.copy(),sigma,o.confidence)


def landing_readiness(track,position,now,clear_descent=False,approved_mechanism=False,radius=.27,margin=.08,pos_std=0.):
    """No descent command: expose a reviewed authority boundary rather than inventing one."""
    pad=track.estimate(now,pos_std)
    if pad is None: return 'HOLD_PAD_UNCERTAIN'
    edges=np.roll(pad.corners,-1,axis=0)-pad.corners
    distances=np.abs(np.cross(edges[:,:2],np.asarray(position)[:2]-pad.corners[:,:2]))/np.maximum(np.linalg.norm(edges[:,:2],axis=1),1e-9)
    signs=np.cross(edges[:,:2],np.asarray(position)[:2]-pad.corners[:,:2])
    if not ((signs>=0).all() or (signs<=0).all()) or distances.min()<radius+margin+3*pad.std: return 'ALIGN_INSIDE_PAD'
    if now-pad.stamp>.35: return 'HOLD_VISUAL_CONTACT_LOST'
    if not clear_descent: return 'HOLD_DESCENT_CLEARANCE_UNKNOWN'
    if not approved_mechanism: return 'WAIT_ORGANIZER_LANDING_AUTHORITY'
    return 'READY_FOR_APPROVED_LANDING'
