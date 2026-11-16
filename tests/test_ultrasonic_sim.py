import copy
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
import yaml

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from ultrasonic_config import load_mounting,attach_sensors,ultrasonic_layer_params,fan_distance,range_arc,finite_navigation_range


class UltrasonicTest(unittest.TestCase):
    def test_user_coordinates_and_channel_order(self):
        mounting=load_mounting()
        expected=[(-.46,-.475,-math.pi/2),(.49,-.475,-math.pi/2),
                  (.49,.475,math.pi/2),(-.46,.475,math.pi/2)]
        for i,(sensor,(x,y,yaw)) in enumerate(zip(mounting,expected),1):
            self.assertEqual(sensor['frame'],f'ultrasonic_{i}')
            self.assertAlmostEqual(sensor['x'],x);self.assertAlmostEqual(sensor['y'],y)
            self.assertAlmostEqual(sensor['yaw'],yaw);self.assertEqual(sensor['z'],.49)
            self.assertAlmostEqual(sensor['field_of_view'],math.pi/3)
            self.assertEqual(sensor['max_range'],.55)

    def test_add_sensors_preserves_world_physics_and_mass(self):
        source=ROOT/'test_scenes/worlds/narrow_17.sdf'
        original=ET.parse(source)
        with tempfile.TemporaryDirectory() as tmp:
            output=Path(tmp)/'world.sdf';attach_sensors(source,output,load_mounting())
            tree=ET.parse(output)
            self.assertEqual([x.text for x in tree.findall('.//inertial/mass')],
                             [x.text for x in original.findall('.//inertial/mass')])
            base=tree.find("world/model[@name='patrol_car']/link[@name='base_link']")
            for sensor in load_mounting():
                element=base.find(f"sensor[@name='{sensor['frame']}']")
                self.assertEqual(element.find('ignition_frame_id').text,sensor['frame'])
                self.assertEqual(float(element.find('lidar/range/max').text),.55)
                self.assertEqual(float(element.find('pose').text.split()[2]),.49)

    def test_ultrasonic_layer_is_local_only_and_preserves_original(self):
        original=yaml.safe_load((ROOT/'test_scenes/nav2_narrow_turns.yaml').read_text())
        frozen=copy.deepcopy(original);cfg=ultrasonic_layer_params(original)
        self.assertEqual(original,frozen)
        local=cfg['local_costmap']['local_costmap']['ros__parameters']
        self.assertLess(local['plugins'].index('ultrasonic_layer'),local['plugins'].index('inflation_layer'))
        legacy=ultrasonic_layer_params(original,mode='arcs')['local_costmap']['local_costmap']['ros__parameters']
        self.assertEqual(len(legacy['ultrasonic_layer']['observation_sources'].split()),8)
        self.assertEqual(local['ultrasonic_layer']['plugin'],'nav2_costmap_2d::RangeSensorLayer')
        self.assertEqual(len(local['ultrasonic_layer']['topics']),4)
        self.assertTrue(local['ultrasonic_layer']['clear_on_max_reading'])
        for source in legacy['ultrasonic_layer']['observation_sources'].split():
            self.assertGreater(legacy['ultrasonic_layer'][source]['max_obstacle_height'],.49)
        self.assertEqual(cfg['global_costmap'],original['global_costmap'])

    def test_navigation_adapter_preserves_no_echo_and_too_close_semantics(self):
        self.assertEqual(finite_navigation_range(math.inf,.03,.55),.55)
        self.assertEqual(finite_navigation_range(-math.inf,.03,.55),.03)
        self.assertEqual(finite_navigation_range(.2,.03,.55),.2)
        self.assertIsNone(finite_navigation_range(math.nan,.03,.55))
        self.assertIsNone(finite_navigation_range(.6,.03,.55))
        self.assertIsNone(finite_navigation_range(.2,.55,.03))

    def test_no_echo_invalid_and_near_limit_are_distinct(self):
        self.assertAlmostEqual(fan_distance([math.inf,.4,.38],.03,.55),.38)
        self.assertEqual(fan_distance([math.inf,math.inf],.03,.55),math.inf)
        self.assertEqual(fan_distance([.02,.4],.03,.55),-math.inf)
        self.assertIsNone(fan_distance([math.nan,math.inf],.03,.55))
        self.assertIsNone(fan_distance([],.03,.55))
        points,marking=range_arc(math.inf,.03,.55,math.pi/3)
        self.assertFalse(marking);self.assertAlmostEqual(math.hypot(*points[0][:2]),.55)
        points,marking=range_arc(.195,.03,.55,math.pi/3)
        self.assertTrue(marking);self.assertAlmostEqual(points[15][0],.195)
        self.assertEqual(range_arc(math.nan,.03,.55,math.pi/3),([],False))
        self.assertEqual(range_arc(.2,.55,.03,math.pi/3),([],False))


if __name__=='__main__':unittest.main()
