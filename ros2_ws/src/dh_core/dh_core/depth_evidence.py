"""Conservative dense free evidence for the existing team's sparse-ray voxel map.

Every voxel must fit wholly inside a valid depth-image rectangle, in front of
its nearest measured surface. No interpolation across invalid pixels. This
closes sampling holes, not blind spots; occupied cells are never cleared here.
"""
import cv2
import numpy as np


def carve_visible_band(voxels,times,depth,K,R,t,stamp,z_low,z_high,max_range=4.,error=.03):
    fx,fy,cx,cy=K;h,w=depth.shape
    half=voxels.voxel/2
    lo=np.maximum([0,0,0],np.floor((np.asarray(t)-max_range-voxels.lo)/voxels.voxel).astype(int))
    hi=np.minimum(voxels.shape,np.ceil((np.asarray(t)+max_range-voxels.lo)/voxels.voxel).astype(int))
    lo[2]=max(lo[2],int(np.floor((z_low-voxels.lo[2])/voxels.voxel)))
    hi[2]=min(hi[2],int(np.ceil((z_high-voxels.lo[2])/voxels.voxel)))
    if np.any(hi<=lo):return 0
    ijk=np.stack(np.meshgrid(*(np.arange(a,b) for a,b in zip(lo,hi)),indexing='ij'),axis=-1).reshape(-1,3)
    p=(voxels.lo+(ijk+.5)*voxels.voxel-t)@R
    # A cube's optical-axis extent is <= sqrt(3)*half, including rotated views.
    r=np.sqrt(3)*half;z=p[:,2]
    good=(z>r+.05)&(np.linalg.norm(p,axis=1)+r<max_range)
    ijk,p,z=ijk[good],p[good],z[good]
    u=fx*p[:,0]/z+cx;v=fy*p[:,1]/z+cy
    # Conservative perspective bounds for the entire voxel's enclosing sphere.
    extent=np.ceil(np.maximum(fx,fy)*r*(1+np.maximum(abs(p[:,0]),abs(p[:,1]))/z)/(z-r)+2).astype(int)
    valid=np.isfinite(depth)&(depth>.1)&(depth<8.)
    source=np.where(valid,depth,0.).astype(np.float32)
    count=0
    for radius in (2,4,8,16,32):
        select=(extent<=radius)&(extent>(0 if radius==2 else radius//2))
        select&=(u>=radius)&(u<w-radius-1)&(v>=radius)&(v<h-radius-1)
        ids=np.flatnonzero(select)
        if not len(ids):continue
        minimum=cv2.erode(source,np.ones((2*radius+1,2*radius+1),np.uint8),borderType=cv2.BORDER_CONSTANT,borderValue=0)
        observed=minimum[np.rint(v[ids]).astype(int),np.rint(u[ids]).astype(int)]>z[ids]+r+error
        q=ijk[ids[observed]];idx=tuple(q.T)
        # Existing ray-map positive hits veto dense free evidence.
        keep=voxels.grid[idx]<=0;q=q[keep];idx=tuple(q.T)
        voxels.grid[idx]=np.maximum(voxels.l_min,voxels.grid[idx]-voxels.l_free)
        times[idx]=stamp;count+=len(q)
    return count
