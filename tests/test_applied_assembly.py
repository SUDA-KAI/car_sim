import copy
import hashlib
from pathlib import Path
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
import numpy as np

ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'scripts'))
from assembly_config import load_applied,apply_visuals,ultrasonic_poses
from sensor_visuals import add_visuals
from build_sensor_description import matrix,world_transforms
from ultrasonic_config import load_mounting


class AppliedAssemblyTest(unittest.TestCase):
    def test_frozen_meshes_and_exact_saved_geometry_keep_vehicle_physics(self):
        state,assets,frames=load_applied();transforms=world_transforms(frames)
        original=ET.parse(ROOT/'test_scenes/worlds/narrow_17.sdf');tree=copy.deepcopy(original)
        with tempfile.TemporaryDirectory() as directory:
            import yaml
            path=Path(directory)/'frames.yaml';path.write_text(yaml.safe_dump(frames))
            add_visuals(tree,path,l2_mesh=assets['l2']['path']);apply_visuals(tree,state,assets,frames)
        def normalized(e):
            e=copy.deepcopy(e)
            for node in e.iter():node.text=node.text.strip() if node.text else None;node.tail=None
            return ET.tostring(e)
        for kind in ('inertial','collision','joint','plugin','sensor'):
            self.assertEqual([normalized(e) for e in original.findall('.//'+kind)],[normalized(e) for e in tree.findall('.//'+kind)])
        base=tree.find("world/model[@name='patrol_car']/link[@name='base_link']")
        expected={'main_bracket','l2_bracket','camera_bracket','camera_right_bracket',
                  *{f'ultrasonic_{i}_{suffix}' for i in range(1,5) for suffix in ('model','body')}}
        self.assertEqual({v.get('name').removeprefix('assembled_') for v in base.findall('visual') if v.get('name','').startswith('assembled_')},expected)
        for p in state['parts']:
            if p['id'] not in expected:continue
            visual=base.find(f"visual[@name='assembled_{p['id']}']");a=assets[p['asset']]
            values=list(map(float,visual.findtext('pose').split()));sdf=matrix(dict(zip(('x','y','z','roll','pitch','yaw'),values)))
            corner=np.array(a['bounds_max_m']);offset=np.array(a['offset_m'])
            np.testing.assert_allclose((sdf@np.r_[corner,1])[:3],(transforms[p['id']]@np.r_[corner+offset,1])[:3],atol=1e-9)
            self.assertEqual(visual.findtext('geometry/mesh/scale'),' '.join([str(a['scale'])]*3))
            self.assertEqual(hashlib.sha256(Path(a['path']).read_bytes()).hexdigest(),a['sha256'])
            self.assertNotIn('%',visual.findtext('geometry/mesh/uri')) # ASCII cached filenames.
        self.assertIsNone(base.find("visual[@name='measured_ultrasonic_1_body']"))

    def test_sonar_sensor_geometry_matches_canonical_assembly_not_editor_pivots(self):
        state,assets,frames=load_applied();expected=world_transforms(frames)
        actual=ultrasonic_poses(load_mounting(),frames)
        for s in actual:
            transform=matrix({key:s[key] for key in ('x','y','z','roll','pitch','yaw')})
            np.testing.assert_allclose(transform,expected[s['frame']],atol=1e-10)
        changed=copy.deepcopy(state)
        for p in changed['parts']:
            if p['asset']!='frame':p['pivot_percent']=[0,100,100]
        from assembly.model import frames_for
        self.assertEqual(frames_for(state),frames_for(changed))


if __name__=='__main__':unittest.main()
