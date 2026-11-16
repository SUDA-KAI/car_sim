import copy
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET
import numpy as np
from model import catalog,defaults,validate,Store,ConflictError,ROOT,frames_for,world_transforms,EXTRA_PARTS,atomic_json


class AssemblyTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):cls.assets=catalog()
    def test_native_mesh_units_and_sensor_origins(self):
        self.assertAlmostEqual(self.assets['l2_bracket']['size_mm'][2],89.)
        self.assertEqual(self.assets['camera_bracket']['scale'],.001)
        frames=world_transforms(frames_for(defaults(self.assets)))
        np.testing.assert_allclose(frames['unilidar_lidar'][:3,3],[.09,0,.929])
        self.assertTrue(all(not p['confirmed'] for p in defaults(self.assets)['parts'] if p['category']=='bracket'))
    def test_cycle_fixed_reference_and_invalid_marker_rejected(self):
        for change in ('cycle','fixed','normal'):
            state=defaults(self.assets);parts={p['id']:p for p in state['parts']}
            if change=='cycle':parts['main_bracket']['parent']='l2_bracket';parts['l2_bracket']['parent']='main_bracket'
            elif change=='fixed':parts['third_crossbar']['xyz_m'][2]=.6
            else:state['markers']=[dict(id=1,part='base_link',point_m=[0,0,0],normal=[0,0,0],label='')]
            with self.assertRaises(ValueError):validate(state,self.assets)
    def test_atomic_save_backup_and_stale_revision(self):
        with tempfile.TemporaryDirectory() as folder:
            store=Store(Path(folder)/'draft.json',self.assets);state=store.load();old=copy.deepcopy(state)
            state['notes']='主控支架固定在横杠';state['parts'][-1]['xyz_m']=[.1,.2,.3]
            saved=store.save(state);self.assertEqual(store.load(),saved)
            self.assertTrue((Path(folder)/'backups/assembly_r00000.json').exists())
            with self.assertRaises(ConflictError):store.save(old)
            self.assertEqual(store.load()['revision'],1)
    def test_export_retains_car_dynamics_and_uses_same_pivots(self):
        with tempfile.TemporaryDirectory() as folder:
            store=Store(Path(folder)/'draft.json',self.assets);store.load();result=store.export()
            original=ET.parse(ROOT/'models/patrol_car.urdf');exported=ET.parse(result['files'][0])
            def normalize(e):
                e=copy.deepcopy(e)
                for child in e.iter():child.text=child.text.strip() if child.text else None;child.tail=None
                return ET.tostring(e)
            for kind in ('inertial','collision'):
                self.assertEqual([normalize(e) for e in original.findall('.//'+kind)],[normalize(e) for e in exported.findall('.//'+kind)])
            visual=exported.find("link[@name='camera_bracket']/visual")
            np.testing.assert_allclose(list(map(float,visual.find('origin').get('xyz').split())),self.assets['camera_bracket']['offset_m'])
            self.assertEqual(visual.find('geometry/mesh').get('scale'),'0.001 0.001 0.001')
            self.assertIsNotNone(exported.find("link[@name='camera_right_bracket']/visual/geometry/mesh"))
            for i in range(1,5):
                frame=f'ultrasonic_{i}'
                joint=exported.find(f"joint[@name='assembly_{frame}_model']")
                self.assertEqual(joint.find('parent').get('link'),frame)
                mesh=exported.find(f"link[@name='{frame}_model']/visual/geometry/mesh")
                self.assertEqual(mesh.get('filename'),Path(self.assets['ultrasonic_right' if i<3 else 'ultrasonic_left']['path']).as_uri())
                body=exported.find(f"link[@name='{frame}_body']/visual/geometry/mesh")
                self.assertEqual(body.get('filename'),Path(self.assets['ultrasonic_body']['path']).as_uri())
                self.assertEqual(exported.find(f"joint[@name='assembly_{frame}_body']/parent").get('link'),frame+'_model')

    def test_additional_mesh_migration_retains_existing_work(self):
        state=defaults(self.assets);state['parts']=[p for p in state['parts'] if p['id'] not in EXTRA_PARTS]
        state['revision']=7;main=next(p for p in state['parts'] if p['id']=='main_bracket')
        main['xyz_m']=[.12,.23,.45];main['rpy_deg']=[90,0,12];main['notes']='已填写的安装说明'
        state['markers']=[dict(id=1,part='main_bracket',point_m=[.01,.02,.03],normal=[0,0,1],label='原安装孔')]
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'draft.json';atomic_json(path,state);store=Store(path,self.assets);loaded=store.load()
            self.assertEqual(loaded['revision'],7);self.assertEqual(loaded['markers'],state['markers'])
            self.assertEqual(next(p for p in loaded['parts'] if p['id']=='main_bracket'),main)
            self.assertTrue((Path(folder)/'backups/before_extra_meshes_r00007.json').exists())
            self.assertEqual(set(p['id'] for p in loaded['parts'])-set(p['id'] for p in state['parts']),EXTRA_PARTS)
            self.assertEqual(store.save(state)['revision'],8) # Old open pages may save without deleting new instances.

    def test_two_left_two_right_models_do_not_move_acoustic_frames(self):
        state=defaults(self.assets);before=world_transforms(frames_for(state));parts={p['id']:p for p in state['parts']}
        for i,asset in [(1,'ultrasonic_right'),(2,'ultrasonic_right'),(3,'ultrasonic_left'),(4,'ultrasonic_left')]:
            frame=f'ultrasonic_{i}';model=parts[frame+'_model']
            self.assertEqual(model['asset'],asset);self.assertEqual(model['parent'],frame)
            model['xyz_m']=[.02,-.03,.04];model['rpy_deg']=[90,0,180]
            body=parts[frame+'_body'];self.assertEqual(body['parent'],frame+'_model')
            self.assertEqual(body['asset'],'ultrasonic_body');self.assertFalse(body['confirmed'])
        after=world_transforms(frames_for(state))
        for i in range(1,5):np.testing.assert_allclose(before[f'ultrasonic_{i}'],after[f'ultrasonic_{i}'])
        for i in range(1,5):
            np.testing.assert_allclose(np.linalg.inv(after[f'ultrasonic_{i}_model'])@after[f'ultrasonic_{i}_body'],
                                       np.linalg.inv(before[f'ultrasonic_{i}_model'])@before[f'ultrasonic_{i}_body'],atol=1e-12)

    def test_edit_pivot_persists_without_redefining_physical_frames(self):
        with tempfile.TemporaryDirectory() as folder:
            store=Store(Path(folder)/'draft.json',self.assets);state=store.load();before=world_transforms(frames_for(state))
            for p in state['parts']:
                if p['asset']!='frame':p['pivot_percent']=[25,75,100]
            saved=store.save(state);loaded=store.load();self.assertEqual(saved,loaded)
            after=world_transforms(frames_for(loaded))
            for key in before:np.testing.assert_allclose(before[key],after[key])
            main=next(p for p in loaded['parts'] if p['id']=='main_bracket')
            for values in ([-1,50,0],[50,101,0],[float('nan'),50,0],[True,50,0]):
                main['pivot_percent']=values
                with self.assertRaises(ValueError):validate(loaded,self.assets)
            main['pivot_percent']=[50,50,0]
            next(p for p in loaded['parts'] if p['id']=='third_crossbar')['pivot_percent']=[50,50,50]
            with self.assertRaises(ValueError):validate(loaded,self.assets)


if __name__=='__main__':unittest.main()
