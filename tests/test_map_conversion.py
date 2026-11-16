import sys
import tempfile
import unittest
from pathlib import Path
import xml.etree.ElementTree as E
import numpy as np
import yaml
from PIL import Image
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from build_world import build, rectangles

class MapConversionTest(unittest.TestCase):
    def test_merging_preserves_occupied_cells(self):
        rng=np.random.default_rng(42)
        for _ in range(30):
            mask=rng.random((20,30))>0.6
            restored=np.zeros_like(mask,dtype=int)
            for x0,x1,r0,r1 in rectangles(mask): restored[r0:r1,x0:x1]+=1
            np.testing.assert_array_equal(restored,mask.astype(int))

    def test_large_map_mesh_preserves_box_geometry(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp); pixels=np.full((100,100),254,dtype=np.uint8)
            pixels[0:30:2,0:30:2]=0
            Image.fromarray(pixels).save(root/'map.pgm')
            cfg=dict(image='map.pgm',resolution=0.1,origin=[0.,0.,0.],negate=0,occupied_thresh=0.65,free_thresh=0.196)
            (root/'map.yaml').write_text(yaml.safe_dump(cfg))
            build(root/'map.yaml',root/'map.sdf',1.5,[7.,3.,0.])
            lines=(root/'map.obj').read_text().splitlines()
            vertices=np.array([list(map(float,line.split()[1:])) for line in lines if line.startswith('v ')])
            self.assertEqual(vertices.shape,(225*8,3))
            self.assertEqual(sum(line.startswith('f ') for line in lines),225*12)
            self.assertEqual(sum(line.startswith('vn ') for line in lines),6)
            self.assertTrue(all('//' in line for line in lines if line.startswith('f ')))
            centers=vertices.reshape(-1,8,3).mean(axis=1)
            for x,y,z in centers:
                col=int(x/0.1); row=99-int(y/0.1)
                self.assertEqual(pixels[row,col],0)
                self.assertAlmostEqual(z,0.75)
            world=E.parse(root/'map.sdf').getroot().find('world')
            link=world.find("model[@name='map_obstacles']/link")
            self.assertEqual(len(link.findall('collision')),1)
            self.assertTrue(link.find('collision/geometry/mesh/uri').text.endswith('map.obj'))

    def test_origin_rotation_pixel_flip_and_spawn(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp); pixels=np.full((40,40),254,dtype=np.uint8)
            pixels[0,0]=0; pixels[0,1]=205
            Image.fromarray(pixels).save(root/'map.pgm')
            cfg=dict(image='map.pgm',resolution=0.1,origin=[10.,20.,1.5707963267948966],negate=0,occupied_thresh=0.65,free_thresh=0.196)
            (root/'map.yaml').write_text(yaml.safe_dump(cfg))
            build(root/'map.yaml',root/'map.sdf',1.5,[8.,22.,0.])
            world=E.parse(root/'map.sdf').getroot().find('world')
            obstacles=world.find("model[@name='map_obstacles']")
            self.assertEqual(obstacles.find('pose').text,'10.0 20.0 0 0 0 1.5707963267948966')
            self.assertEqual(len(obstacles.findall('link')),1)
            xyz=list(map(float,obstacles.find('link/collision/pose').text.split()[:3]))
            np.testing.assert_allclose(xyz,[0.05,3.95,0.75])
            with self.assertRaises(ValueError):build(root/'map.yaml',root/'bad.sdf',1.5,[10.,20.,0.])

if __name__=='__main__':unittest.main()
