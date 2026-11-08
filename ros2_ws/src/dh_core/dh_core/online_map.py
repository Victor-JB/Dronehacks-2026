"""Sensor-only 3D map. Reuses the team's vectorized ray/log-odds implementation.

Unknown is blocked, including unobserved height inside a footprint. No course,
mesh, simulator truth, nominal camera rotation or nearest-free pose relocation.
"""
from dataclasses import dataclass
import math
import numpy as np
from .team_mapping import VoxelMap, UNKNOWN, FREE, OCCUPIED, camera_rays
from .passages import apply_no_overwall


@dataclass
class MapView:
    lo: np.ndarray
    res: float
    states: np.ndarray                 # [x,y,z], UNKNOWN/FREE/OCCUPIED
    safe: np.ndarray                   # full 3D body+margin certified in FREE
    revision: int
    passages: tuple=()

    def cell(self, xyz):
        return tuple(np.floor((np.asarray(xyz)-self.lo)/self.res).astype(int))

    def centre(self, cell):
        return self.lo+(np.asarray(cell)+.5)*self.res

    def inside(self, cell):
        return all(0 <= i < n for i,n in zip(cell,self.safe.shape))

    def points_safe(self, points):
        p=np.atleast_2d(points)
        cells=np.floor((p-self.lo)/self.res).astype(int)
        ok=np.all((cells>=0)&(cells<self.safe.shape),axis=1)
        result=np.zeros(len(p),bool)
        result[ok]=self.safe[tuple(cells[ok].T)]
        return result

    def segment_safe(self, a, b):
        n=max(2,int(math.ceil(np.linalg.norm(np.asarray(b)-a)/(self.res/3)))+1)
        return bool(self.points_safe(np.linspace(a,b,n)).all())

    def path_safe(self, path):
        return len(path)>=2 and all(self.segment_safe(a,b) for a,b in zip(path[:-1],path[1:]))


class OnlineMap:
    def __init__(self,res=.2,size=16.,z_min=-1.,z_max=3.,radius=.27,half_height=.12,margin=.05,free_ttl=20.,legal_band=None,floor_z=None):
        if (not np.isfinite([res,size,z_min,z_max,radius,half_height,margin,free_ttl]).all()
                or not .04<=res<=.5 or size<2 or z_max<=z_min or radius<=0 or half_height<=0 or margin<0 or free_ttl<=0
                or (math.ceil(size/res)**2*math.ceil((z_max-z_min)/res)>2_000_000)):
            raise ValueError('Invalid or over-budget voxel/body configuration')
        self.model=VoxelMap(res,size,z_min,z_max)
        self.seen=np.full(self.model.shape,-np.inf,np.float64)  # ROS epoch seconds need subsecond precision
        self.radius,self.half_height,self.margin=radius,half_height,margin
        self.free_ttl=free_ttl
        if legal_band is not None and (len(legal_band)!=2 or not np.isfinite(legal_band).all() or legal_band[0]>=legal_band[1]):
            raise ValueError('Invalid reviewed legal centre-height envelope')
        self.legal_band=None if legal_band is None else tuple(legal_band)
        if floor_z is not None and not math.isfinite(floor_z): raise ValueError('Invalid measured floor height')
        self.floor_z=floor_z
        self.revision=0; self.last_stamp=None
        self._cache=None

    def reset(self):
        self.model.grid.fill(0); self.seen.fill(-np.inf)
        self.last_stamp=None; self.revision+=1; self._cache=None

    def observe(self,origin,directions,ranges,stamp,now,max_range=4.):
        origin=np.asarray(origin,float); dirs=np.asarray(directions,float); r=np.asarray(ranges,float)
        if (origin.shape!=(3,) or dirs.shape!=(len(r),3) or not np.isfinite(origin).all()
                or not math.isfinite(stamp) or stamp<=0 or not -.02<=now-stamp<=.5
                or (self.last_stamp is not None and stamp<=self.last_stamp)):
            return False
        norms=np.linalg.norm(dirs,axis=1)
        good=np.isfinite(r)&(r>0)&np.isfinite(dirs).all(axis=1)&(norms>.99)&(norms<1.01)
        dirs,r=dirs[good],r[good]
        if not len(r): return False   # NaN/inf/zero cannot carve FREE
        step=self.model.voxel/2
        ts=np.arange(step,max_range,step)
        free=origin+dirs[:,None,:]*ts[None,:,None]
        free=free[ts[None,:]<np.minimum(r,max_range)[:,None]-self.model.voxel]
        hit=origin+dirs[r<=max_range]*r[r<=max_range,None]
        touched=np.concatenate([self.model._index(free),self.model._index(hit)])
        self.model.integrate(origin,dirs,r,max_range)
        hits=self.model._index(hit)
        # A new positive return overrides old FREE confidence immediately.
        # Subsequent measured free rays can clear it; passage is never delayed
        # until several contradictory hits have overcome old log-odds.
        self.model.grid.reshape(-1)[hits]=np.maximum(self.model.grid.reshape(-1)[hits],self.model.l_occ+.001)
        self.seen.reshape(-1)[touched]=stamp
        self.last_stamp=stamp; self.revision+=1; self._cache=None
        return True

    def snapshot(self,now,pos_std=0.):
        flight_band=self.legal_band
        key=(self.revision,int(now*5),round(pos_std,3),flight_band)
        if self._cache is not None and self._cache[0]==key: return self._cache[1]
        g=self.model.grid; res=self.model.voxel
        states=np.full(g.shape,UNKNOWN,np.int8)
        states[(g<0)&(now-self.seen<=self.free_ttl)]=FREE
        states[g>self.model.l_occ]=OCCUPIED
        free=states==FREE
        safe=free.copy()
        # A voxel is a volume, not a zero-size point; include its half-diagonal.
        rr=self.radius+self.margin+3*max(0.,pos_std)+math.sqrt(3)*res/2
        hh=self.half_height+self.margin+3*max(0.,pos_std)+math.sqrt(3)*res/2
        for dx in range(-math.ceil(rr/res),math.ceil(rr/res)+1):
            for dy in range(-math.ceil(rr/res),math.ceil(rr/res)+1):
                if math.hypot(dx,dy)*res>rr: continue
                for dz in range(-math.ceil(hh/res),math.ceil(hh/res)+1):
                    if abs(dz)*res>hh: continue
                    offset=(dx,dy,dz)
                    dst=tuple(slice(max(0,-o),min(n,n-o)) for o,n in zip(offset,g.shape))
                    src=tuple(slice(max(0,o),min(n,n+o)) for o,n in zip(offset,g.shape))
                    shifted=np.zeros(g.shape,bool); shifted[dst]=free[src]
                    safe &= shifted
        zs=self.model.lo[2]+(np.arange(g.shape[2])+.5)*res
        if flight_band is None: safe[:]=False
        else: safe[:,:,(zs<flight_band[0])|(zs>flight_band[1])]=False
        view=MapView(self.model.lo.copy(),res,states,safe,self.revision)
        view.safe,view.passages=apply_no_overwall(view,safe,flight_band,self.floor_z)
        for a in (view.lo,view.states,view.safe): a.setflags(write=False)
        self._cache=(key,view)
        return view

    @property
    def memory_bytes(self): return self.model.grid.nbytes+self.seen.nbytes
