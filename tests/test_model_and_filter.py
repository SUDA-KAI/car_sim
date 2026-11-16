import sys,unittest
from pathlib import Path
import xml.etree.ElementTree as E
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from build_car import mass_properties,shift
from filter_map import filter_occupancy
ROOT=Path(__file__).resolve().parents[1]

class SolidPropertiesTest(unittest.TestCase):
    def test_uniform_box_analytic_inertia_and_offset(self):
        v=np.array([[-1,-2,-3],[1,-2,-3],[1,2,-3],[-1,2,-3],[-1,-2,3],[1,-2,3],[1,2,3],[-1,2,3]],float)+[10,20,30]
        faces=np.array([(0,2,1),(0,3,2),(4,5,6),(4,6,7),(0,1,5),(0,5,4),(1,2,6),(1,6,5),(2,3,7),(2,7,6),(3,0,4),(3,4,7)])
        volume,com,inertia=mass_properties(v[faces],12)
        self.assertAlmostEqual(volume,48)
        np.testing.assert_allclose(com,[10,20,30],atol=1e-10)
        np.testing.assert_allclose(inertia,np.diag([52,40,20]),atol=1e-8)
        with self.assertRaises(ValueError):mass_properties(v[faces[:-1]],12)

    def test_drivable_urdf_matches_uniform_body_mass_properties(self):
        import json
        expected=json.loads((ROOT/'models/car_properties.json').read_text())
        robot=E.parse(ROOT/'models/patrol_car.urdf').getroot()
        positions={'base_link':np.zeros(3)}
        joints=robot.findall('joint')
        while joints:
            pending=[]
            for j in joints:
                parent=j.find('parent').get('link');child=j.find('child').get('link')
                if parent in positions:positions[child]=positions[parent]+np.fromstring(j.find('origin').get('xyz'),sep=' ')
                else:pending.append(j)
            self.assertLess(len(pending),len(joints));joints=pending
        total=0.;first=np.zeros(3);origin_inertia=np.zeros((3,3))
        for link in robot.findall('link'):
            part=link.find('inertial')
            if part is None:continue
            m=float(part.find('mass').get('value'));pos=positions[link.get('name')]+np.fromstring(part.find('origin').get('xyz'),sep=' ')
            p=part.find('inertia').attrib
            inertia=np.array([[float(p[k]) for k in row] for row in [('ixx','ixy','ixz'),('ixy','iyy','iyz'),('ixz','iyz','izz')]])
            self.assertGreater(np.linalg.eigvalsh(inertia).min(),0)
            total+=m;first+=m*pos;origin_inertia+=inertia+m*shift(pos)
        self.assertAlmostEqual(total,100.)
        com=first/total;np.testing.assert_allclose(com,expected['center_of_mass_m'],atol=1e-9)
        np.testing.assert_allclose(origin_inertia-total*shift(com),expected['inertia_kg_m2'],atol=1e-8)

class OccupancyFilterTest(unittest.TestCase):
    def test_speckles_thin_walls_gaps_and_unknown(self):
        occ=np.zeros((100,100),bool);occ[50,10:90]=True;occ[70:75,10:90]=True;occ[70:75,48:50]=False
        occ[10,10]=True;occ[80:100,99]=True;unknown=np.zeros_like(occ);unknown[:5]=True
        result,stats=filter_occupancy(occ,unknown,.02)
        self.assertFalse(result[10,10]);self.assertTrue(result[50,20]);self.assertTrue(result[99,99])
        self.assertTrue(result[72,49]);self.assertFalse(result[unknown].any())
        self.assertEqual(stats['components_removed'],1)

if __name__=='__main__':unittest.main()
