import math
import copy
from pathlib import Path
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from sensor_visuals import attach_visuals,pose_text,resolve_resource_paths
from build_sensor_description import matrix


class VisualsTest(unittest.TestCase):
    def test_temporary_world_preserves_relative_mesh_paths(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'temporary.sdf'
            path.write_text('<sdf><world><uri>../models/car.stl</uri>'
                            '<uri>model://ground_plane</uri><uri>file:///tmp/lidar.stl</uri></world></sdf>')
            resolve_resource_paths(path,ROOT/'worlds')
            values=[item.text for item in ET.parse(path).findall('.//uri')]
            self.assertEqual(values,[(ROOT/'models/car.stl').as_uri(),
                                     'model://ground_plane','file:///tmp/lidar.stl'])

    def test_measured_lidar_is_up_and_car_physics_unchanged(self):
        source=ROOT/'test_scenes/worlds/narrow_turns.sdf'
        original=ET.parse(source)
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'world.sdf'
            attach_visuals(source,path,ROOT/'sensor_mounting_tf.yaml',axes=True)
            tree=ET.parse(path)
            def normalized(element):
                element=copy.deepcopy(element)
                for item in element.iter():
                    item.text=item.text.strip() if item.text else None
                    item.tail=None
                return ET.tostring(element)
            for kind in ('inertial','collision','joint','plugin','sensor'):
                self.assertEqual([normalized(e) for e in original.findall('.//'+kind)],
                                 [normalized(e) for e in tree.findall('.//'+kind)])
            base=tree.find("world/model[@name='patrol_car']/link[@name='base_link']")
            lidar=base.find("visual[@name='measured_l2_mesh']")
            pose=list(map(float,lidar.find('pose').text.split()))
            np.testing.assert_allclose(pose,[.09,0,.929,0,0,0],atol=1e-9)
            z_axis=base.find("visual[@name='measured_unilidar_lidar_axis_2']")
            self.assertGreater(float(z_axis.find('pose').text.split()[2]),pose[2])
            self.assertEqual(lidar.find('geometry/mesh/scale').text,'1 1 1')

    def test_pose_rotation_roundtrip(self):
        for angles in [(0,0,0),(.2,-.5,1.0),(0,math.pi/2,.4)]:
            pose=dict(zip(('roll','pitch','yaw'),angles));pose.update(x=.2,y=-.1,z=.9)
            transform=matrix(pose);values=list(map(float,pose_text(transform).split()))
            recovered=matrix(dict(zip(('x','y','z','roll','pitch','yaw'),values)))
            np.testing.assert_allclose(transform,recovered,atol=1e-8)


if __name__=='__main__':unittest.main()
