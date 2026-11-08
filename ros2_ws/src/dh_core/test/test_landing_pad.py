import sys
from pathlib import Path
import numpy as np
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[4]/'tools/sil'))
from pad_scene import render_pad
from dh_core.landing_pad import detect_blue_pad,PadTrack,landing_readiness
from dh_core.online_geometry import register_depth_to_color


def detect(scene,stamp=1,**kw):
    return detect_blue_pad(*scene,stamp,registered=True,**kw)


@pytest.mark.parametrize('light,yaw',[(1,0),(.3,0),(.65,.2),(1.05,-.3)])
def test_lighting_and_approach_geometry(light,yaw):
    o=detect(render_pad(light=light,yaw=yaw))
    assert o is not None
    assert np.linalg.norm(o.centre-[2.8,0,0])<.05
    assert np.max(abs(o.corners[:,2]))<.03


def test_unregistered_mono_and_nonorange_false_positive():
    s=render_pad()
    assert detect_blue_pad(*s,1,registered=False) is None
    mono=np.repeat(s[0].mean(axis=2).astype(np.uint8)[:,:,None],3,axis=2)
    assert detect((mono,*s[1:])) is None
    gray=s[0].copy(); gray[(gray[:,:,2]>gray[:,:,0])]=(100,100,100)
    assert detect((gray,*s[1:])) is None


def test_bad_depth_and_uncertainty_rejected():
    s=render_pad(); assert detect(s,depth_std=.11) is None
    assert detect((s[0],np.full_like(s[1],np.nan),*s[2:])) is None
    vertical=np.full_like(s[1],3.)
    assert detect((s[0],vertical,*s[2:])) is None


def test_partial_occlusion_retains_bounded_track_not_invented_corners():
    track=PadTrack()
    for stamp in (1,1.1,1.2):
        assert track.update(detect(render_pad(),stamp),stamp)
    assert detect(render_pad(occluded=True),1.3) is None
    assert track.estimate(1.5) is not None
    assert track.estimate(3.3) is None


def test_visual_loss_and_landing_authority_gate():
    track=PadTrack()
    for stamp in (1,1.1,1.2): track.update(detect(render_pad(),stamp),stamp)
    assert landing_readiness(track,[2.8,0,.7],1.25,True,False)=='WAIT_ORGANIZER_LANDING_AUTHORITY'
    assert landing_readiness(track,[2.8,0,.7],1.25,True,True)=='READY_FOR_APPROVED_LANDING'
    assert landing_readiness(track,[2.8,0,.7],1.7,True,True)=='HOLD_VISUAL_CONTACT_LOST'
    assert landing_readiness(track,[3.35,0,.7],1.25,True,True)=='ALIGN_INSIDE_PAD'


def test_identity_registration_preserves_metric_depth():
    s=render_pad()
    registered=register_depth_to_color(s[1],s[2],s[2],np.eye(3),np.zeros(3),s[1].shape)
    assert np.nanmax(abs(registered-s[1]))<1e-9


def test_registration_uses_measured_translation_not_same_pixel_assumption():
    K=np.array([[100.,0,2],[0,100.,2],[0,0,1]])
    depth=np.full((5,5),np.nan); depth[2,2]=1.
    registered=register_depth_to_color(depth,K,K,np.eye(3),[.01,0,0],(5,5))
    assert registered[2,3]==1. and np.isnan(registered[2,2])


def test_long_loss_requires_reconfirmation_and_nan_uncertainty_is_invalid():
    s=render_pad(); track=PadTrack()
    assert detect(s,depth_std=float('nan')) is None
    for t in (1.,1.1,1.2): track.update(detect(s,t),t)
    assert track.estimate(1.3) is not None
    assert track.update(detect(s,3.5),3.5) and track.estimate(3.5) is None


def test_nadir_pad_lost_is_expected_with_forward_camera():
    for height in (1.2,.8,.5,.3):
        assert detect(render_pad(centre=(0,0),height=height)) is None


def test_depth_noise_and_ambiguous_squares():
    assert detect(render_pad(noise=.04),depth_std=.04) is not None
    assert detect(render_pad(noise=.2),depth_std=.03) is None
    left=render_pad(centre=(2.8,-.7),side=.8)
    right=render_pad(centre=(2.8,.7),side=.8)
    image=left[0].copy(); mask=right[0][:,:,0]>right[0][:,:,2]; image[mask]=right[0][mask]
    assert detect((image,*left[1:])) is None
