import math
from pathlib import Path
import sys
import unittest
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from split_drive import targets, MAX_CENTRE_STEER, MAX_WHEEL_STEER
from split_drive_world import install_split_drive


class SplitDriveLimits(unittest.TestCase):
    def test_forward_reverse_and_each_front_wheel(self):
        for velocity in (-3.,-.2,.2,3.):
            for yaw in (-5.,0.,5.):
                drive,left,right=targets(velocity,yaw,0.)
                self.assertGreaterEqual(drive,-.2)
                self.assertLessEqual(drive,.5)
                self.assertLessEqual(abs(left),math.radians(40.)+1e-9)
                self.assertLessEqual(abs(right),math.radians(40.)+1e-9)
        self.assertAlmostEqual(targets(3.,0.,0.)[0],.5)
        self.assertAlmostEqual(targets(-3.,0.,0.)[0],-.2)
        self.assertLess(MAX_CENTRE_STEER,MAX_WHEEL_STEER)

    def test_sdf_joint_stops_match_software(self):
        import tempfile
        with tempfile.NamedTemporaryFile(suffix='.sdf') as file:
            install_split_drive(ROOT/'test_scenes/worlds/narrow_turns.sdf',file.name)
            robot=ET.parse(file.name).getroot().find(".//model[@name='patrol_car']")
            self.assertIsNone(robot.find("plugin[@name='ignition::gazebo::systems::AckermannSteering']"))
            for name in ('front_left_steer','front_right_steer'):
                limit=robot.find(f"joint[@name='{name}']/axis/limit")
                self.assertAlmostEqual(float(limit.findtext('lower')),-math.radians(40.))
                self.assertAlmostEqual(float(limit.findtext('upper')),math.radians(40.))


if __name__=='__main__':unittest.main()
