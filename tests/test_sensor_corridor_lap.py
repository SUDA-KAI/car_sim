import sys
from pathlib import Path
import numpy as np
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from sensor_corridor_lap import choose_arc


def visible(points):
    # Approximate angular scan with nearest return per small bin.
    angle=np.arctan2(points[:,1],points[:,0]);rad=np.linalg.norm(points,axis=1)
    bins=np.round(angle/.008).astype(int);order=np.argsort(rad);_,idx=np.unique(bins[order],return_index=True)
    return points[order[idx]]


def test_straight_centered():
    x=np.linspace(-8,12,4000)
    walls=[np.stack([x,np.full_like(x,y)],axis=1) for y in [-.85,.85]]
    walls.append(np.stack([np.full(1000,12.),np.linspace(-.85,.85,1000)],axis=1))
    pts=visible(np.concatenate(walls))
    arc=choose_arc(pts)
    assert abs(arc['curvature'])<.015
    assert arc['clearance']>.8


def test_curve_is_observed_not_world_route():
    theta=np.linspace(-1.5,1.5,10000)
    pts=visible(np.concatenate([np.stack([r*np.sin(theta),5-r*np.cos(theta)],axis=1) for r in [4.15,5.85]]))
    arc=choose_arc(pts)
    assert .15<arc['curvature']<.25
    assert arc['clearance']>.7
    with pytest.raises(ValueError):choose_arc(pts,lookahead=4.)


def test_blocked_corridor_rejected():
    a=np.linspace(-np.pi,np.pi,360)
    with pytest.raises(ValueError):choose_arc(np.stack([.5*np.cos(a),.5*np.sin(a)],axis=1))


def test_right_curve_symmetry():
    theta=np.linspace(-1.5,1.5,10000)
    points=visible(np.concatenate([np.stack([r*np.sin(theta),5-r*np.cos(theta)],axis=1) for r in [4.15,5.85]]))
    left=choose_arc(points,lookahead=1.8)
    right=choose_arc(points*np.array([1.,-1.]),lookahead=1.8)
    assert left['curvature']>.1
    assert abs(left['curvature']+right['curvature'])<.01


def test_recenter_from_lateral_offset():
    x=np.linspace(-8,12,4000)
    walls=[np.stack([x,np.full_like(x,y)],axis=1) for y in [-1.,.7]]
    walls.append(np.stack([np.full(1000,12.),np.linspace(-1.,.7,1000)],axis=1))
    arc=choose_arc(visible(np.concatenate(walls)),lookahead=1.8)
    assert arc['y']<-.05
    assert arc['clearance']>.65


def test_transition_goal_follows_end_tangent():
    straight=np.linspace(-8,.8,4000)
    theta=np.linspace(0,1.5,5000)
    walls=[]
    for radius in [4.15,5.85]:
        walls.extend([np.stack([straight,np.full_like(straight,5-radius)],axis=1),
                      np.stack([.8+radius*np.sin(theta),5-radius*np.cos(theta)],axis=1)])
    goal=choose_arc(visible(np.concatenate(walls)),lookahead=1.8)
    assert goal['y']>.06
    assert goal['yaw']>.12
    assert goal['clearance']>.7
