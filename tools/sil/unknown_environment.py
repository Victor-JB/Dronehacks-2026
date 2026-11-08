"""Sensor producer ONLY. Complete geometry lives here, never in navigation.
Ideal noiseless ray sensor/odometry are disclosed simulation assumptions.
Panoramic mode is an algorithm harness, NOT the physical forward OAK camera.
"""
import math
import numpy as np


class UnknownWorld:
    def __init__(self,layout='wall',seed=0):
        rng=np.random.default_rng(seed)
        self.boxes=[]
        # Closed box gives real finite distant returns, not invented infinity-clear.
        self.boxes.extend([(np.array(a,float),np.array(b,float)) for a,b in (
            ((-6,-4,-.2),(6,4,0)),((-6,-4,2.8),(6,4,3.)),
            ((-6.2,-4,0),(-6,4,2.8)),((6,-4,0),(6.2,4,2.8)),
            ((-6,-4.2,0),(6,-4,2.8)),((-6,4,0),(6,4.2,2.8)))])
        gap=float(rng.uniform(.7,1.4))
        if layout=='wall': self.add_box((-.1,-4,0),(.1,gap,2.8))
        elif layout=='opening':
            self.add_box((-.1,-4,0),(.1,4,.5)); self.add_box((-.1,-4,2.),(.1,4,2.8))
            self.add_box((-.1,-4,.5),(.1,-1.,2.)); self.add_box((-.1,1.,.5),(.1,4,2.))
        elif layout=='dead_end':
            self.add_box((-1,-.8,0),(1,-.6,2.8)); self.add_box((-1,.6,0),(1,.8,2.8)); self.add_box((.8,-.8,0),(1,.8,2.8))
        elif layout=='narrow':
            self.add_box((-.2,-4,0),(.2,-.35,2.8)); self.add_box((-.2,.35,0),(.2,4,2.8))
        elif layout!='open': raise ValueError(layout)
        self.pad=np.array([3.,0.,0.])

    def add_box(self,lo,hi): self.boxes.append((np.array(lo,float),np.array(hi,float)))

    def raycast(self,origin,dirs):
        o=np.asarray(origin); d=np.asarray(dirs)
        result=np.full(len(d),np.inf)
        for lo,hi in self.boxes:
            with np.errstate(divide='ignore',invalid='ignore'):
                a=(lo-o)/d; b=(hi-o)/d
            parallel=abs(d)<1e-12
            outside=parallel&((o<lo)|(o>hi))
            lower=np.where(parallel,-np.inf,np.minimum(a,b)).max(axis=1)
            upper=np.where(parallel,np.inf,np.maximum(a,b)).min(axis=1)
            valid=(~outside.any(axis=1))&(upper>=np.maximum(lower,0))
            hit=np.where(lower>1e-5,lower,upper)
            result=np.minimum(result,np.where(valid&(hit>1e-5),hit,np.inf))
        return result

    def observe(self,position,yaw=0.,panoramic=True):
        if panoramic:
            az,el=np.meshgrid(np.linspace(-math.pi,math.pi,120,endpoint=False),np.linspace(-1.35,1.35,25))
            dirs=np.column_stack([np.cos(el.ravel())*np.cos(az.ravel()),np.cos(el.ravel())*np.sin(az.ravel()),np.sin(el.ravel())])
        else:
            # Actual sampled 960x600 mono calibration, forward-facing test mount (runtime uses measured TF).
            u,v=np.meshgrid(np.arange(0,960,24),np.arange(0,600,24))
            optical=np.column_stack([(u.ravel()-487.3531)/595.1951,(v.ravel()-305.8477)/595.3043,np.ones(u.size)])
            from dh_core.online_geometry import nominal_transform_for_tests
            R,_=nominal_transform_for_tests(0.)
            dirs=optical@R.T
            dirs/=np.linalg.norm(dirs,axis=1)[:,None]
            c,s=math.cos(yaw),math.sin(yaw)
            dirs=dirs@np.array([[c,-s,0],[s,c,0],[0,0,1]]).T
        ranges=self.raycast(position,dirs)
        ranges[(ranges>8)|(~np.isfinite(ranges))]=np.nan
        return np.asarray(position),dirs,ranges

    def goal_sensor(self,position,stamp):
        # Ideal visual landmark observation, available only if in range and unoccluded.
        from dh_core.online_navigation import GoalObservation
        delta=self.pad-position; distance=np.linalg.norm(delta)
        if distance>2.5: return None
        hit=self.raycast(position,np.atleast_2d(delta/distance))[0]
        if hit<distance-.08: return None
        return GoalObservation(self.pad+np.array([0,0,.8]),stamp,.95,'blue_pad')

    def collision(self,position,radius=.27,half_height=.12):
        p=np.asarray(position)
        for lo,hi in self.boxes:
            if p[2]+half_height<=lo[2] or p[2]-half_height>=hi[2]: continue
            closest=np.clip(p[:2],lo[:2],hi[:2])
            if np.linalg.norm(p[:2]-closest)<radius: return True
        return False
