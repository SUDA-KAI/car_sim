from pathlib import Path
import copy
import math
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
import cv2
import numpy as np
import yaml
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from stereo_config import load_calibration,calibrated_mounts,attach_stereo
from stereo_geometry import estimate_delta
from build_sensor_description import world_transforms
from perception_texture import paint_scene
from sensor_localization_config import sensor_only_bridges


class StereoLocalizationTest(unittest.TestCase):
    def setUp(self):
        self.c=load_calibration();self.original=yaml.safe_load((ROOT/'sensor_mounting_tf.yaml').read_text())
        self.frames=calibrated_mounts(self.original,self.c);self.transforms=world_transforms(self.frames)

    def test_real_calibration_scaling_transform_direction_and_housings(self):
        self.assertAlmostEqual(self.c['baseline_m'],.2471029205650454)
        self.assertAlmostEqual(self.c['K'][0][0,0],1526.3412892728695/3)
        relative=np.linalg.inv(self.transforms['camera_right_link_optical'])@self.transforms['camera_left_link_optical']
        np.testing.assert_allclose(relative,self.c['T_right_left'],atol=1e-12)
        self.assertEqual(self.frames['camera_right_link'],self.original['camera_right_link'])
        self.assertNotIn('camera_left_rect_optical',self.original)

    def test_distorted_stereo_rectification_produces_metric_depth(self):
        rng=np.random.default_rng(31);points=rng.uniform([-.8,-.5,3.],[.8,.5,8.],(100,3)).astype(float)
        t=self.c['T_right_left'];pixels=[]
        for i in (0,1):
            rv=np.zeros(3) if i==0 else cv2.Rodrigues(t[:3,:3])[0]
            tv=np.zeros(3) if i==0 else t[:3,3]
            raw=cv2.projectPoints(points,rv,tv,self.c['K'][i],self.c['D'][i])[0]
            pixels.append(cv2.undistortPoints(raw,self.c['K'][i],self.c['D'][i],R=self.c['R'][i],P=self.c['P'][i]).reshape(-1,2))
        np.testing.assert_allclose(pixels[0][:,1],pixels[1][:,1],atol=1e-4)
        depth=self.c['P'][0][0,0]*self.c['baseline_m']/(pixels[0][:,0]-pixels[1][:,0])
        np.testing.assert_allclose(depth,(self.c['R'][0]@points.T).T[:,2],atol=1e-4)

    def test_pnp_motion_sign_and_body_lever_arm_without_truth(self):
        rng=np.random.default_rng(21);points=rng.uniform([-1,-.7,3.],[1.,.7,8.],(100,3)).astype(np.float32)
        bc=self.transforms['camera_left_rect_optical'];k=self.c['P'][0][:,:3]
        desired=np.eye(4);a=.025;desired[:3,:3]=[[math.cos(a),-math.sin(a),0],[math.sin(a),math.cos(a),0],[0,0,1]];desired[0,3]=.1
        camera_motion=np.linalg.inv(np.linalg.inv(bc)@desired@bc)
        pixels=cv2.projectPoints(points,cv2.Rodrigues(camera_motion[:3,:3])[0],camera_motion[:3,3],k,None)[0].reshape(-1,2)
        actual,inliers,error=estimate_delta(points,pixels,k,bc,.5)
        np.testing.assert_allclose(actual,desired,atol=1e-5);self.assertEqual(inliers,100)
        with self.assertRaises(ValueError):estimate_delta(points[:5],pixels[:5],k,bc,.5)
        with self.assertRaises(ValueError):estimate_delta(points,pixels,k,bc,.001)

    def test_cameras_and_texture_preserve_original_collision_geometry(self):
        source=ROOT/'test_scenes/worlds/narrow_turns.sdf'
        original=ET.parse(source)
        original_collisions=[ET.tostring(x) for x in original.findall('.//collision')]
        with tempfile.TemporaryDirectory() as directory:
            target=Path(directory)/'stereo.sdf';attach_stereo(source,target,self.frames,self.c)
            paint_scene(target,target);tree=ET.parse(target)
            self.assertEqual([ET.tostring(x) for x in tree.findall('.//collision')],original_collisions)
            cameras=tree.findall(".//sensor[@type='camera']");self.assertEqual(len(cameras),2)
            self.assertEqual(cameras[0].findtext('camera/image/width'),'640')
            self.assertAlmostEqual(float(cameras[0].findtext('camera/lens/intrinsics/fx')),self.c['K'][0][0,0])
            self.assertIsNone(cameras[0].find('camera/distortion'))

    def test_no_simulator_pose_bridges_or_absolute_imu_attitude(self):
        original=yaml.safe_load((ROOT/'config/bridge.yaml').read_text())
        original.append(dict(ros_topic_name='/another_truth_name',ros_type_name='nav_msgs/msg/Odometry'))
        safe=sensor_only_bridges(original)
        self.assertFalse(any(x['ros_type_name'] in ('nav_msgs/msg/Odometry','tf2_msgs/msg/TFMessage') for x in safe))
        self.assertTrue(any(x['ros_topic_name']=='/clock' for x in safe))
        self.assertFalse(any(x['ros_topic_name']=='/cmd_vel' for x in safe))
        self.assertTrue(any(x['ros_topic_name']=='/localization/safe_cmd_vel' for x in safe))
        ekf=yaml.safe_load((ROOT/'config/sensor_ekf.yaml').read_text())['ekf_filter_node']['ros__parameters']
        self.assertEqual(ekf['imu0_config'][3:6],[False]*3)
        self.assertEqual(ekf['odom1_config'][:6],[False]*6)
        self.assertFalse(any('wheel' in value or 'ground_truth' in value for value in ekf.values() if isinstance(value,str)))

if __name__=='__main__':unittest.main()
