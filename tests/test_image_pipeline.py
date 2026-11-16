from array import array
from pathlib import Path
import sys
import unittest
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Header
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'scripts'))
from stereo_sim import image_array,image_message

class ImagePipelineTest(unittest.TestCase):
    def test_fast_buffer_path_preserves_pixels_and_header(self):
        pixels=np.arange(24,dtype=np.uint8).reshape(2,4,3);header=Header(frame_id='left_optical')
        message=image_message(pixels,header)
        self.assertIsInstance(message.data,array)
        np.testing.assert_equal(image_array(message),pixels)
        message.header.frame_id='rectified'
        self.assertEqual(header.frame_id,'left_optical')
    def test_rgb_input_with_row_padding(self):
        message=Image(height=2,width=2,step=8,encoding='rgb8')
        message.data=array('B',[1,2,3,4,5,6,99,99,7,8,9,10,11,12,99,99])
        expected=np.array([[[3,2,1],[6,5,4]],[[9,8,7],[12,11,10]]],np.uint8)
        np.testing.assert_equal(image_array(message),expected)
if __name__=='__main__':unittest.main()
