import math
from pathlib import Path
import sys
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from analyze_turns_run import analyze
from build_narrow_suite import capsule

SCENE=dict(clear_width_m=1.7,reference_geometry=dict(radius=5.,left_x=-2.,right_x=6.))

def rows(xy,yaw):
    return [dict(x=str(p[0]),y=str(p[1]),yaw=str(a),sim_time=str(i*.1),true_vx='.3',
        estimated_side_clearance_m='.2',wheel_sweep_clearance_bound_m='.1',plan_distance_m='0')
        for i,(p,a) in enumerate(zip(xy,yaw))]

class TurnsAnalysisTest(unittest.TestCase):
    def test_action_success_while_stationary_is_not_a_pass(self):
        result=analyze(rows([(-2,-5)]*10,[0.]*10),SCENE,dict(result='success'))
        self.assertFalse(result['passed'])
        self.assertFalse(result['checks']['sufficient_travel'])
        self.assertFalse(result['checks']['complete_heading_rotation'])
    def test_full_loop_requires_coverage_and_positive_clearance(self):
        xy=capsule(5.);xy=np.vstack([xy,xy[0]])
        delta=np.diff(xy,axis=0);yaw=np.arctan2(delta[:,1],delta[:,0]);yaw=np.r_[yaw,yaw[-1]]
        trace=rows(xy,yaw)
        result=analyze(trace,SCENE,dict(result='success'))
        self.assertTrue(result['passed'],result)
        trace[50]['wheel_sweep_clearance_bound_m']='-.01'
        result=analyze(trace,SCENE,dict(result='success'))
        self.assertFalse(result['passed'])
        self.assertFalse(result['checks']['no_conservative_wheel_envelope_overlap'])
if __name__=='__main__':unittest.main()
