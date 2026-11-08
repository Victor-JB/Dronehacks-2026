"""Observed-voxel geometry unit fixtures, NOT end-to-end stereo/ROS evidence."""
import numpy as np
import pytest
from dh_core.online_map import OnlineMap
from dh_core.online_planning import plan_observed
from dh_core.online_navigation import OnlineNavigator,GoalObservation
from dh_core.passages import PassageProgress


def fixture(kind='window',small=False,unknown_exit=False):
    m=OnlineMap(res=.1,size=6.,z_min=-.2,z_max=2.8,legal_band=(.35,2.3),floor_z=0.)
    cells=np.indices(m.model.shape).transpose(1,2,3,0)
    xyz=m.model.lo+(cells+.5)*m.model.voxel; x,y,z=xyz[...,0],xyz[...,1],xyz[...,2]
    m.model.grid.fill(-2.); m.seen.fill(1.); m.last_stamp=1.
    if kind=='window':
        wall=(abs(x)<.15)&(z<2.25)
        hole=(abs(y)<(.3 if small else .85))&(z>.5)&(z<1.95)
        occupied=wall&~hole
    elif kind=='tunnel':
        radius=np.sqrt(y*y+(z-1.2)**2)
        inner=.35 if small else .85
        occupied=(abs(x)<.65)&(radius>=inner)&(radius<1.1)
    else:
        occupied=(abs(x)<.15)&(z<1.0)  # physically open sky above this wall
    occupied|=(z<0)|(z>2.65)
    m.model.grid[occupied]=2.
    if unknown_exit:
        m.model.grid[x>.3]=0.; m.seen[x>.3]=-np.inf
    return m


@pytest.mark.parametrize('planner',['clothoid','sfc','mpl'])
@pytest.mark.parametrize('kind',['window','tunnel'])
def test_observed_passage_classification_and_certified_centreline(planner,kind):
    view=fixture(kind).snapshot(1.)
    candidates=[p for p in view.passages if p.kind==kind]
    assert candidates,[(p.kind,p.width,p.height,p.length) for p in view.passages]
    p=min(candidates,key=lambda p:np.linalg.norm(p.centre-[0,0,1.2]))
    assert abs(p.normal[0])>.95 and abs(p.centre[1])<.15
    assert p.radius>.6 and view.segment_safe(p.entry,p.exit)
    t,stats=plan_observed(view,p.entry,p.exit,planner,.2)
    assert t is not None,stats
    assert view.path_safe(t.p) and t.v.max()<=.201
    progress=PassageProgress((kind,))
    assert np.allclose(progress.target(view,p.entry),p.exit)
    assert not progress.done
    progress.update(view,p.exit)
    assert progress.done


@pytest.mark.parametrize('kind',['window','tunnel'])
def test_small_or_unseen_exit_cannot_be_committed(kind):
    for m in (fixture(kind,small=True),fixture(kind,unknown_exit=True)):
        v=m.snapshot(1.)
        assert not v.segment_safe([-1,0,1.2],[1,0,1.2])
        progress=PassageProgress((kind,))
        for p in v.passages:
            progress.target(v,p.entry); progress.update(v,p.exit)
        assert not progress.done


@pytest.mark.parametrize('planner',['clothoid','sfc','mpl'])
def test_open_sky_above_wall_is_not_an_allowed_shortcut(planner):
    v=fixture('low_wall').snapshot(1.)
    assert not v.passages
    assert not v.segment_safe([-1,0,1.6],[1,0,1.6])
    t,stats=plan_observed(v,[-1,0,1.6],[1,0,1.6],planner,.4)
    assert t is None,stats


def test_unknown_legal_altitude_blocks_motion_and_pad_does_not_skip_objectives():
    n=OnlineNavigator(map_options=dict(size=4.)); n.active=True
    n.odometry([0,0,1],0,1,1); n.tracking(1,1,1)
    assert n.request(1) is None and n.state=='HOLD_LEGAL_ALTITUDE_UNCONFIRMED'
    n.map=fixture('window')
    n.set_goal(GoalObservation(np.array([1,0,.8]),1,.9,'blue_pad'),1)
    request=n.request(1)
    assert request is not None and request['goal'] is None
    assert n.passages.required=={'tunnel','window'} and not n.passages.done
