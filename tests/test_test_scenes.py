import json,math,sys,unittest
from pathlib import Path
import xml.etree.ElementTree as E
import numpy as np
from PIL import Image
import yaml
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from record_test_metrics import nearest_path_distance,corridor_clearance
from narrow_geometry import envelope_clearance,center_offset

class TerrainScenesTest(unittest.TestCase):
    def test_every_scene_has_separate_wheel_and_ground_truth(self):
        manifest=json.loads((ROOT/'test_scenes/manifest.json').read_text())
        self.assertEqual(len(manifest),14)
        for meta in manifest:
            world=E.parse(meta['world']).getroot().find('world')
            robot=world.find("model[@name='patrol_car']")
            truth=robot.find("plugin[@name='ignition::gazebo::systems::OdometryPublisher']")
            self.assertEqual(truth.find('dimensions').text,'3')
            self.assertEqual(truth.find('odom_topic').text,'/sim/ground_truth')
            steering=robot.find("plugin[@name='ignition::gazebo::systems::AckermannSteering']")
            self.assertEqual(steering.find('odom_topic').text,'/sim/wheel_odom')
            cfg=yaml.safe_load(Path(meta['map']).read_text());im=np.asarray(Image.open(Path(meta['map']).parent/cfg['image']))
            route=yaml.safe_load(Path(meta['route']).read_text())['waypoints']
            for point in [{'x':meta['spawn'][0],'y':meta['spawn'][1]}]+route:
                col=int((point['x']-cfg['origin'][0])/cfg['resolution']);row=im.shape[0]-1-int((point['y']-cfg['origin'][1])/cfg['resolution'])
                self.assertEqual(im[row,col],254)
    def test_slope_mesh_has_correct_height_and_normals(self):
        for deg in (6,12):
            for part in ('up','down'):
                lines=(ROOT/f'test_scenes/worlds/ramp_{deg}deg_{part}.obj').read_text().splitlines()
                v=np.array([list(map(float,l.split()[1:])) for l in lines if l.startswith('v ')])
                self.assertAlmostEqual(v[:,2].max(),4*math.tan(math.radians(deg)))
                self.assertEqual(sum(l.startswith('vn ') for l in lines),12)
                self.assertTrue(all('//' in l for l in lines if l.startswith('f ')))
                slope=(v[5,2]-v[4,2])/(v[5,0]-v[4,0])
                self.assertAlmostEqual(abs(math.degrees(math.atan(slope))),deg)
    def test_friction_patch_has_no_hidden_high_friction_floor(self):
        for scene,mu in [('slip_normal',1.),('slip_low',.05),('slip_ice',.005),('narrow_slip_normal',1.),('narrow_slip_low',.05),('narrow_slip_ice',.005)]:
            world=E.parse(ROOT/f'test_scenes/worlds/{scene}.sdf').getroot().find('world')
            patch=world.find("model[@name='friction_patch']")
            self.assertAlmostEqual(float(patch.find('link/collision/surface/friction/ode/mu').text),mu)
            surfaces=[]
            for name in ['floor_before','friction_patch','floor_after']:
                m=world.find(f"model[@name='{name}']");x=float(m.find('pose').text.split()[0]);sx=float(m.find('link/collision/geometry/box/size').text.split()[0]);surfaces.append((x-sx/2,x+sx/2))
            self.assertEqual(surfaces,[(-6.,0.),(0.,7.),(7.,16.)])
    def test_narrow_gap_is_170cm_in_physics_and_grid(self):
        for name in ['narrow_17','narrow_17_baseline']:
            world=E.parse(ROOT/f'test_scenes/worlds/{name}.sdf').getroot().find('world')
            left=world.find("model[@name='left_wall']");right=world.find("model[@name='right_wall']")
            inner_left=float(left.find('pose').text.split()[1])-float(left.find('link/collision/geometry/box/size').text.split()[1])/2
            inner_right=float(right.find('pose').text.split()[1])+float(right.find('link/collision/geometry/box/size').text.split()[1])/2
            self.assertAlmostEqual(inner_left-inner_right,1.7)
            cfg=yaml.safe_load((ROOT/f'test_scenes/maps/{name}.yaml').read_text())
            grid=np.asarray(Image.open(ROOT/f'test_scenes/maps/{name}.pgm'))
            column=round((0-cfg['origin'][0])/cfg['resolution'])
            self.assertAlmostEqual(sum(grid[:,column]==254)*cfg['resolution'],1.7)
        self.assertAlmostEqual(corridor_clearance(0,0),.4)
        self.assertAlmostEqual(corridor_clearance(.1,0),.3)
        self.assertAlmostEqual(corridor_clearance(0,math.pi/2),.25)
        self.assertLess(corridor_clearance(.5,0),0)

    def test_all_new_terrain_corridors_are_17m(self):
        for name in ['narrow_ramp_6deg','narrow_ramp_12deg','narrow_slip_normal','narrow_slip_low','narrow_slip_ice']:
            world=E.parse(ROOT/f'test_scenes/worlds/{name}.sdf').getroot().find('world')
            faces=[]
            for wall,sign in [('left_wall',-1),('right_wall',1)]:
                model=world.find(f"model[@name='{wall}']")
                y=float(model.find('pose').text.split()[1]);thickness=float(model.find('link/collision/geometry/box/size').text.split()[1]);faces.append(y+sign*thickness/2)
            self.assertAlmostEqual(faces[0]-faces[1],1.7)
            metadata_path=ROOT/f'test_scenes/worlds/{name}.json'
            meta=json.loads(metadata_path.read_text())
            map_path=Path(meta['map'])
            if not map_path.is_absolute():map_path=(metadata_path.parent/map_path).resolve()
            cfg=yaml.safe_load(map_path.read_text());grid=np.asarray(Image.open(map_path.with_suffix('.pgm')))
            for x in [-3,2,5.5,9,13]:
                col=int((x-cfg['origin'][0])/cfg['resolution'])
                self.assertAlmostEqual(sum(grid[:,col]==254)*cfg['resolution'],1.7)
        for deg in [6,12]:
            vertices=[]
            for line in (ROOT/f'test_scenes/worlds/narrow_ramp_{deg}deg_up.obj').read_text().splitlines():
                if line.startswith('v '):vertices.append(list(map(float,line.split()[1:])))
            self.assertAlmostEqual(np.ptp(np.array(vertices)[:,1]),1.7)

    def test_curved_corridor_width_and_3d_envelope(self):
        meta=json.loads((ROOT/'test_scenes/worlds/narrow_turns.json').read_text())
        a=meta['reference_geometry']['radius']
        self.assertAlmostEqual(a,5.)
        scene=dict(clear_width_m=1.7,robot_length_m=1.2,robot_width_m=.9,robot_height_m=.495,reference_geometry=dict(type='straight',center_y=0.))
        self.assertAlmostEqual(envelope_clearance(scene,0,0,(0,0,0,1)),.4)
        self.assertLess(envelope_clearance(scene,0,0,(math.sin(.05),0,0,math.cos(.05))),.4)
        q=(0,0,math.sin(math.pi/4),math.cos(math.pi/4))
        self.assertAlmostEqual(envelope_clearance(meta,6+a,0,q),a+.85-math.hypot(a+.45,.6))
        self.assertAlmostEqual(center_offset(meta,6+a+.1,0),.1)
        def vertices(name):
            lines=(ROOT/f'test_scenes/worlds/{name}').read_text().splitlines()
            return np.array([list(map(float,l.split()[1:])) for l in lines if l.startswith('v ')])
        inner=vertices('narrow_turns_inner.obj');outer=vertices('narrow_turns_outer.obj')
        n=len(inner)//2
        # ring order: inner-bottom, outer-bottom, inner-top, outer-top.
        np.testing.assert_allclose(np.linalg.norm(outer[:n,:2]-inner[:n,:2],axis=1),1.7,atol=1e-10)
        for filename in ['narrow_turns_inner.obj','narrow_turns_outer.obj']:
            self.assertTrue(all('//' in line for line in (ROOT/f'test_scenes/worlds/{filename}').read_text().splitlines() if line.startswith('f ')))

    def test_plan_distance_measures_segments(self):
        self.assertAlmostEqual(nearest_path_distance(1,2,[(0,0),(4,0)]),2)
        self.assertAlmostEqual(nearest_path_distance(6,0,[(0,0),(4,0)]),2)
        self.assertIsNone(nearest_path_distance(0,0,[]))
if __name__=='__main__':unittest.main()
