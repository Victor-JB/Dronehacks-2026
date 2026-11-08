"""Synthetic COLOR camera/depth renderer only; not an OAK mono feed."""
import math
import numpy as np
from dh_core.online_geometry import nominal_transform_for_tests

K=np.array([[595.1951,0,487.3531],[0,595.3043,305.8477],[0,0,1.]])


def render_pad(light=1.,yaw=0.,height=.8,centre=(2.8,0.),side=1.2,occluded=False,noise=0.,seed=1,position_xy=(0.,0.)):
    h,w=600,960
    R,t=nominal_transform_for_tests(0.)
    c,s=math.cos(yaw),math.sin(yaw); yaw_R=np.array([[c,-s,0],[s,c,0],[0,0,1]])
    R=yaw_R@R; t=yaw_R@t+np.array([0,0,height])
    t[:2]+=position_xy
    u,v=np.meshgrid(np.arange(w),np.arange(h))
    rays=np.stack([(u-K[0,2])/K[0,0],(v-K[1,2])/K[1,1],np.ones((h,w))],axis=-1)@R.T
    with np.errstate(divide='ignore',invalid='ignore'): depth=-t[2]/rays[:,:,2]
    valid=(depth>.1)&(depth<8.)
    points=t+rays*depth[:,:,None]
    blue=valid&(abs(points[:,:,0]-centre[0])<side/2)&(abs(points[:,:,1]-centre[1])<side/2)
    image=np.full((h,w,3),(0,120,240),np.uint8)
    image[~valid]=(90,90,90); image[blue]=(230,35,10)
    if occluded:
        image[(u>450)&(u<510)&blue]=(0,120,240)
    image=np.clip(image.astype(float)*light,0,255).astype(np.uint8)
    depth[~valid]=np.nan
    if noise: depth+=np.random.default_rng(seed).normal(0,noise,depth.shape)
    return image,depth,K,R,t
