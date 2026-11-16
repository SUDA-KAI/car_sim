from pathlib import Path
import sys
import unittest
import yaml
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from patrol_route import route_groups

class PatrolRouteTest(unittest.TestCase):
    def test_closed_route_uses_distinct_endpoints_and_no_handoff_in_bends(self):
        route=yaml.safe_load((ROOT/'test_scenes/routes/narrow_turns.yaml').read_text())
        groups=route_groups(route,'through_poses')
        self.assertEqual([len(group) for group in groups],[21,19])
        self.assertEqual([p for group in groups for p in group],route['waypoints'])
        start={'x':-2.,'y':-5.}
        for group in groups:
            end=group[-1]
            self.assertGreater((end['x']-start['x'])**2+(end['y']-start['y'])**2,1.)
            self.assertGreater(end['x'],-2.01);self.assertLess(end['x'],6.01)
            self.assertAlmostEqual(abs(end['y']),5.)
            start=end
        self.assertEqual(len(route_groups(route,'waypoints')),40)
        # Each bend is constrained at intervals below 0.9 m, unlike the old
        # 45-degree connections that could cut 0.2 m inside the centreline.
        import math
        for group in groups:
            for before,after in zip(group,group[1:]):
                if not -2<=after['x']<=6:
                    self.assertLess(math.hypot(after['x']-before['x'],after['y']-before['y']),.9)
    def test_bad_segments_and_existing_unmarked_routes(self):
        points=[{'x':0},{'x':1},{'x':2}]
        self.assertEqual(route_groups({'waypoints':points},'through_poses'),[points])
        for ends in ([0,3],[2,1,3],[2],[4,3],[1.,3],[]):
            with self.assertRaises(ValueError):route_groups({'waypoints':points,'segment_end_indices':ends},'through_poses')
if __name__=='__main__':unittest.main()
