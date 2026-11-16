from pathlib import Path
import sys
import unittest
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'scripts'))
from lio_odometry_adapter import fresh_stamp

class SensorInputValidityTest(unittest.TestCase):
    def test_fresh_lio_only_and_no_retimestamping_stale_poses(self):
        self.assertTrue(fresh_stamp(10.,10.2))
        self.assertFalse(fresh_stamp(10.,130.))
        self.assertFalse(fresh_stamp(11.,10.))
        self.assertFalse(fresh_stamp(float('nan'),10.))
        self.assertFalse(fresh_stamp(10.,float('inf')))
if __name__=='__main__':unittest.main()

class LioConsistencyTest(unittest.TestCase):
    def test_reject_wrong_turn_and_large_translation(self):
        import numpy as np
        from scipy.spatial.transform import Rotation
        from lio_odometry_adapter import visual_consistent
        reference=np.eye(4);reference[0,3]=8.
        good=reference.copy();good[0,3]+=.1
        self.assertTrue(visual_consistent(good,reference))
        bad=reference.copy();bad[0,3]-=2.
        self.assertFalse(visual_consistent(bad,reference))
        bad=reference.copy();bad[:3,:3]=Rotation.from_euler('z',.25).as_matrix()
        self.assertFalse(visual_consistent(bad,reference))
    def test_yaw_wrap_and_nonfinite_rejection(self):
        import numpy as np
        from scipy.spatial.transform import Rotation
        from lio_odometry_adapter import visual_consistent
        a=np.eye(4);b=np.eye(4)
        a[:3,:3]=Rotation.from_euler('z',3.13).as_matrix();b[:3,:3]=Rotation.from_euler('z',-3.13).as_matrix()
        self.assertTrue(visual_consistent(a,b))
        a[0,3]=float('nan');self.assertFalse(visual_consistent(a,b))
