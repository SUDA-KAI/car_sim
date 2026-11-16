import copy
import math
from pathlib import Path
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
import numpy as np
from sensor_msgs.msg import PointCloud2,PointField
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'scripts'))
from l2_sensor_config import load_config,attach_inertial_lidar,BUILTIN_IMU_OFFSET
from l2_sim import read_xyz,convert_points,project_scan,L2Sim


class L2Test(unittest.TestCase):
    def test_read_organized_padded_big_endian_cloud(self):
        msg=PointCloud2(height=2,width=2,point_step=16,row_step=40,is_bigendian=True)
        msg.fields=[PointField(name=k,offset=j*4,datatype=PointField.FLOAT32,count=1) for j,k in enumerate(('x','y','z'))]
        buffer=bytearray(80);expected=np.arange(12).reshape(4,3)
        for j in range(3):
            view=np.ndarray((2,2),dtype='>f4',buffer=buffer,offset=j*4,strides=(40,16));view[:]=expected[:,j].reshape(2,2)
        msg.data=bytes(buffer);np.testing.assert_equal(read_xyz(msg),expected)
        msg.data=bytes(buffer[:-1])
        with self.assertRaises(ValueError):read_xyz(msg)

    def test_ranges_heights_seam_and_snapshot_timing(self):
        config=load_config()
        xyz=np.array([[1.,0,0],[2.,0,0],[-1.,0,0],[-2.,-1e-12,0],[0,1.,1.],[np.inf,0,0],[.01,0,0],[31.,0,0]])
        points=convert_points(xyz,config)
        self.assertEqual(len(points),5);self.assertTrue(np.all(points['time']==0))
        scan=project_scan(points,config)
        self.assertAlmostEqual(scan[180],1.);self.assertAlmostEqual(scan[0],1.)
        self.assertTrue(np.isinf(scan[270])) # upper return is not a horizontal obstacle
        self.assertEqual(np.count_nonzero(np.isfinite(scan)),2)
        self.assertGreater(points['ring'][4],0)

    def test_mounts_and_no_change_to_vehicle_dynamics(self):
        source=ROOT/'test_scenes/worlds/narrow_17.sdf';original=ET.parse(source)
        def normalize(e):
            e=copy.deepcopy(e)
            for child in e.iter():
                child.text=child.text.strip() if child.text else None;child.tail=None
            return ET.tostring(e)
        with tempfile.TemporaryDirectory() as folder:
            out=Path(folder)/'sensor.sdf';frames=attach_inertial_lidar(source,out,ROOT/'sensor_mounting_tf.yaml');tree=ET.parse(out)
            for kind in ('inertial','collision','joint'):
                self.assertEqual([normalize(e) for e in original.findall('.//'+kind)],[normalize(e) for e in tree.findall('.//'+kind)])
            sensors={s.get('name'):s for s in tree.findall('.//sensor')}
            self.assertNotIn('lidar',sensors)
            np.testing.assert_allclose(list(map(float,sensors['l2_lidar_3d'].findtext('pose').split())),[.09,0,.929,0,0,0])
            np.testing.assert_allclose(list(map(float,sensors['external_imu'].findtext('pose').split())),[.09,0,.645,0,0,0])
            np.testing.assert_allclose([frames['unilidar_imu'][k] for k in ('x','y','z')],BUILTIN_IMU_OFFSET)
            self.assertEqual(sensors['l2_lidar_3d'].findtext('lidar/visibility_mask'),'4')
            self.assertEqual(len(tree.findall("world/plugin[@name='ignition::gazebo::systems::Imu']")),1)

    def test_mapping_projection_recovers_observed_distant_wall(self):
        config=load_config();points=convert_points(np.array([[10.,0.,.9],[0.,.85,0.]]),config)
        self.assertTrue(np.isinf(project_scan(points,config)[180]))
        mapping=copy.deepcopy(config);mapping['scan']['max_height_m']=1.2
        self.assertAlmostEqual(project_scan(points,mapping)[180],10.)
        self.assertAlmostEqual(project_scan(points,mapping)[270],.85,places=6)
        self.assertEqual(config['scan']['max_height_m'],.12)

    def test_sensor_mode_does_not_publish_perfect_world_attitude(self):
        from types import SimpleNamespace
        from sensor_msgs.msg import Imu
        message=Imu();message.orientation.z=.7;message.orientation.w=.7
        message.angular_velocity.z=.1;message.linear_acceleration.z=9.8
        received=[];publisher=SimpleNamespace(publish=received.append)
        instance=SimpleNamespace(config=load_config(),no_world_orientation=True)
        L2Sim.imu(instance,message,publisher,'imu_link')
        self.assertEqual(message.orientation_covariance[0],-1.)
        self.assertEqual(message.orientation.z,0.);self.assertEqual(message.orientation.w,1.)
        self.assertEqual(message.angular_velocity.z,.1);self.assertEqual(message.linear_acceleration.z,9.8)


if __name__=='__main__':unittest.main()
