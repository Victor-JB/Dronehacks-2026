"""Bounded wall-aperture evidence from the SAME observed voxel map.

No CAD, nominal positions or unseen exits. Quantized horizontal plane normals
(30deg) are a deliberately limited classifier, not a general cylinder fitter.
Unknown boundary pixels never count as a wall. A circular aperture needs axial
ring persistence before it is called a tunnel. All crossing paths still need
full-body observed-free certification from the shared map.
"""
from dataclasses import dataclass
import math
import cv2
import numpy as np


@dataclass
class Passage:
    kind: str
    centre: np.ndarray
    normal: np.ndarray
    width: float
    height: float
    radius: float
    length: float
    entry: np.ndarray
    exit: np.ndarray
    confidence: float


def detect_passages(view,legal_band,max_planes=12):
    if legal_band is None: return ()
    cells=np.argwhere(view.states==1)
    if len(cells)<24: return ()
    points=view.lo+(cells+.5)*view.res
    points=points[(points[:,2]>=legal_band[0])&(points[:,2]<=legal_band[1])]
    if len(points)<24: return ()
    # Bounded deterministic search. General arbitrary/sloping tubes need further work.
    proposals=[]; extent=np.linalg.norm(np.array(view.states.shape[:2])*view.res)/2
    us=np.arange(-extent,extent+view.res,view.res)
    zs=view.lo[2]+(np.arange(view.states.shape[2])+.5)*view.res
    U,Z=np.meshgrid(us,zs)
    for angle in np.arange(0,math.pi,math.pi/6):
        n=np.array([math.cos(angle),math.sin(angle),0.]); tangent=np.array([-n[1],n[0],0.])
        bins=np.rint((points@n)/view.res).astype(int)
        values,counts=np.unique(bins,return_counts=True)
        for index in np.argsort(counts)[-2:]:
            if counts[index]<24: continue
            proposals.append((int(counts[index]),n,tangent,values[index]*view.res))
    result=[]
    for _,normal,tangent,offset in sorted(proposals,key=lambda p:-p[0])[:max_planes]:
        xyz=offset*normal+U[:,:,None]*tangent+Z[:,:,None]*np.array([0,0,1.])
        def sample(delta):
            c=np.floor((xyz+delta*normal-view.lo)/view.res).astype(int)
            inside=((c>=0)&(c<view.states.shape)).all(axis=2)
            states=np.full(U.shape,-1,np.int8); states[inside]=view.states[tuple(c[inside].T)]
            states[(Z<legal_band[0])|(Z>legal_band[1])]=-1
            return states
        states=sample(0.)
        free=(states==0).astype(np.uint8)*255
        contours,_=cv2.findContours(free,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            x,y,w,h=cv2.boundingRect(contour)
            if min(w,h)<3 or x<=0 or y<=0 or x+w>=len(us) or y+h>=len(zs): continue
            mask=np.zeros_like(free); cv2.drawContours(mask,[contour],-1,255,-1)
            interior=mask>0
            if not (states[interior]==0).all(): continue
            ring=(cv2.dilate(mask,np.ones((3,3),np.uint8))>0)&~interior
            if ring.sum()<12 or (states[ring]==1).mean()<.90: continue
            width,height=w*view.res,h*view.res
            if min(width,height)<.5 or max(width,height)>2.5: continue
            fill=float(interior.sum()/(w*h))
            poly=cv2.approxPolyDP(contour,.04*cv2.arcLength(contour,True),True)
            circular=(.58<fill<.9 and max(width,height)/min(width,height)<1.35)
            if len(poly)==4 and fill>=.90: kind='window'
            elif circular: kind='circular_opening'
            else: continue
            yy,xx=np.nonzero(interior)
            centre=offset*normal+float(us[xx].mean())*tangent+np.array([0,0,float(zs[yy].mean())])
            if any(np.linalg.norm(centre-p.centre)<.65 and abs(normal@p.normal)>.85 for p in result): continue
            axial=[0.]
            if circular:
                for sign in (-1,1):
                    for distance in np.arange(view.res,1.51,view.res):
                        s=sample(sign*distance)
                        if (s[ring]==1).mean()<.85 or (s[interior]==0).mean()<.95: break
                        axial.append(sign*distance)
                if max(axial)-min(axial)>=.5: kind='tunnel'
            length=max(axial)-min(axial)+view.res
            entry=centre+normal*(min(axial)-.5)
            exit=centre+normal*(max(axial)+.5)
            result.append(Passage(kind,centre,normal,width,height,min(width,height)/2,
                                  length,entry,exit,float((states[ring]==1).mean())))
    return tuple(result)


def apply_no_overwall(view,geometric_safe,legal_band,floor_z=None):
    """No height above a detected vertical obstruction, except enclosed apertures.

Conservative: this also blocks over-furniture shortcuts. Only a positively
bounded opening can whitelist its local volume; unknown wall tops never do.
    A verified floor reference separates floor hits from vertical obstructions.
"""
    if legal_band is None or floor_z is None: return np.zeros_like(geometric_safe),()
    zs=view.lo[2]+(np.arange(view.states.shape[2])+.5)*view.res
    walls=(view.states==1)&(zs[None,None,:]>floor_z+view.res/2+.03)
    above=np.maximum.accumulate(walls,axis=2)
    safe=geometric_safe&~above
    passages=detect_passages(view,legal_band)
    candidate_cells=np.argwhere(geometric_safe&above)
    if len(candidate_cells):
        points=view.lo+(candidate_cells+.5)*view.res
        for p in passages:
            if p.kind not in ('window','tunnel'): continue
            tangent=np.array([-p.normal[1],p.normal[0],0.]); delta=points-p.centre
            # Whitelist only the measured aperture extrusion, never above its top.
            allowed=(abs(delta@tangent)<p.width/2)&(abs(delta[:,2])<p.height/2)
            allowed &= (delta@p.normal>min((p.entry-p.centre)@p.normal,(p.exit-p.centre)@p.normal))
            allowed &= (delta@p.normal<max((p.entry-p.centre)@p.normal,(p.exit-p.centre)@p.normal))
            safe[tuple(candidate_cells[allowed].T)]=True
    return safe,passages


class PassageProgress:
    """Count only an aligned observed-free crossing from one side to the other."""
    def __init__(self,required=('tunnel','window')):
        self.required=set(required); self.completed=set(); self.pending=None

    def target(self,view,position):
        if self.pending is not None:
            p,near,far=self.pending
            return far if view.segment_safe(near,far) else None
        candidates=[p for p in view.passages if p.kind in self.required-self.completed]
        if not candidates: return None
        p=min(candidates,key=lambda p:np.linalg.norm(p.centre-position))
        near,far=(p.entry,p.exit) if (position-p.centre)@p.normal<0 else (p.exit,p.entry)
        if np.linalg.norm(position-near)>.18: return near
        if not view.segment_safe(near,far): return None
        self.pending=(p,near.copy(),far.copy())
        return far

    def update(self,view,position):
        if self.pending is None: return
        p,near,far=self.pending
        if not view.segment_safe(near,far): self.pending=None; return
        if np.linalg.norm(position-far)<.18:
            self.completed.add(p.kind); self.pending=None

    @property
    def done(self): return self.required<=self.completed
