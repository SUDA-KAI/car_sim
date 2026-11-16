from pathlib import Path
import sys
import numpy as np
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from planar_lidar_odometry import register_translation,gyro_integral


def walls():
    x=np.linspace(-4,5,240);y=np.linspace(-.85,.85,120)
    return np.concatenate([np.stack([x,np.full_like(x,v)],axis=1) for v in [-.85,.85]]+[np.stack([np.full_like(y,v),y],axis=1) for v in [-4,5]])


@pytest.mark.parametrize('yaw,t',[(0.,[.1,0]),(.03,[.08,.01]),(-.04,[.1,-.01])])
def test_scan_translation_with_measured_rotation(yaw,t):
    reference=walls();t=np.array(t);c,s=np.cos(yaw),np.sin(yaw);r=np.array([[c,-s],[s,c]])
    current=(reference-t)@r
    result,quality=register_translation(reference,current,r,[.07,0.])
    assert np.linalg.norm(result-t)<.005
    assert quality['inliers']>200
    assert quality['rmse_m']<.005


def test_gyro_interval_and_missing_history():
    assert abs(gyro_integral([(0.,.2),(.5,.2),(1.,.2)],.1,.9)-.16)<1e-6
    with pytest.raises(ValueError):gyro_integral([(.5,.2),(1.,.2)],.1,.9)


def test_insufficient_scan_overlap():
    with pytest.raises(ValueError):register_translation(walls(),walls()+[30.,0.],np.eye(2),[0.,0.])
