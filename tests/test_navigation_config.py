"""Controller switching must preserve scene constraints and the TEB baseline."""
import json
from pathlib import Path
import sys
import unittest
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
from navigation_config import configure_controller,configure_speed,configure_yaw_rate,configure_planning_timeout


class NavigationConfigTest(unittest.TestCase):
    def test_planning_timeout_preserves_scene_constraints_and_default(self):
        baseline=yaml.safe_load((ROOT/'config/nav2_sim.yaml').read_text())
        frozen=yaml.safe_dump(baseline)
        self.assertEqual(configure_planning_timeout(baseline,0),baseline)
        result=configure_planning_timeout(baseline,20)
        self.assertEqual(result['planner_server']['ros__parameters']['GridBased']['max_planning_time'],20)
        result['planner_server']['ros__parameters']['GridBased']['max_planning_time']=baseline['planner_server']['ros__parameters']['GridBased']['max_planning_time']
        self.assertEqual(result,baseline)
        self.assertEqual(yaml.safe_dump(baseline),frozen)
        for value in (-1,.5,61,float('nan'),float('inf')):
            with self.assertRaises(ValueError):configure_planning_timeout(baseline,value)

    def test_mppi_respects_each_scene_and_does_not_mutate_baseline(self):
        for meta in json.loads((ROOT / 'test_scenes/manifest.json').read_text()):
            with self.subTest(scene=meta['name']):
                baseline = yaml.safe_load(Path(meta['nav_params']).read_text())
                frozen = yaml.safe_dump(baseline)
                cfg = configure_controller(baseline, 'mppi', meta)
                control = cfg['controller_server']['ros__parameters']
                mppi = control['FollowPath']
                self.assertEqual(mppi['motion_model'], 'Ackermann')
                self.assertEqual(mppi['model_dt'], 1 / control['controller_frequency'])
                smoother = cfg['velocity_smoother']['ros__parameters']
                self.assertEqual(mppi['vx_max'], smoother['max_velocity'][0])
                self.assertEqual(mppi['vx_min'], smoother['min_velocity'][0])
                self.assertEqual(mppi['AckermannConstraints']['min_turning_r'],
                                 cfg['planner_server']['ros__parameters']['GridBased']['minimum_turning_radius'])
                if meta.get('wheel_sweep_width_bound_m'):
                    for name in ('global_costmap', 'local_costmap'):
                        footprint = json.loads(cfg[name][name]['ros__parameters']['footprint'])
                        self.assertAlmostEqual(2 * footprint[0][1], meta['wheel_sweep_width_bound_m'])
                self.assertEqual(yaml.safe_dump(baseline), frozen)
                self.assertEqual(configure_controller(baseline, 'teb', meta), baseline)

    def test_bad_controller_is_rejected(self):
        with self.assertRaises(ValueError):
            configure_controller({}, 'typo', {})

    def test_mppi_prediction_and_path_fit_map_at_both_speed_caps(self):
        baseline=yaml.safe_load((ROOT/'test_scenes/nav2_narrow_turns.yaml').read_text())
        meta=json.loads((ROOT/'test_scenes/worlds/narrow_turns.json').read_text())
        for speed in (.3,.5):
            cfg=configure_speed(configure_controller(baseline,'mppi',meta),speed)
            follow=cfg['controller_server']['ros__parameters']['FollowPath']
            local=cfg['local_costmap']['local_costmap']['ros__parameters']
            margin=max(__import__('math').hypot(*p) for p in json.loads(local['footprint']))+local['footprint_padding']+.2
            available=min(local['width'],local['height'])/2-margin
            self.assertLessEqual(speed*follow['time_steps']*follow['model_dt'],available)
            self.assertLessEqual(follow['prune_distance'],available)
            self.assertGreaterEqual(follow['prune_distance'],speed*follow['time_steps']*follow['model_dt'])

    def test_yaw_cap_reaches_controller_and_smoother_preserving_kinematics(self):
        baseline=yaml.safe_load((ROOT/'test_scenes/nav2_narrow_turns.yaml').read_text())
        for controller in ('scene','teb','mppi'):
            initial=configure_controller(baseline,controller,{})
            frozen=yaml.safe_dump(initial)
            cfg=configure_yaw_rate(initial,.4)
            smooth=cfg['velocity_smoother']['ros__parameters']
            follow=cfg['controller_server']['ros__parameters']['FollowPath']
            self.assertEqual(smooth['max_velocity'][2],.4)
            self.assertEqual(smooth['min_velocity'][2],-.4)
            self.assertEqual(follow.get('wz_max',follow.get('max_vel_theta')),.4)
            self.assertEqual(smooth['max_accel'],initial['velocity_smoother']['ros__parameters']['max_accel'])
            self.assertEqual(cfg['planner_server'],initial['planner_server'])
            if controller=='mppi':
                self.assertEqual(follow['AckermannConstraints'],initial['controller_server']['ros__parameters']['FollowPath']['AckermannConstraints'])
            self.assertEqual(yaml.safe_dump(initial),frozen)
        for rate in (0,-.1,1.1,float('nan'),float('inf')):
            with self.assertRaises(ValueError):configure_yaw_rate({},rate)

    def test_requested_forward_cap_is_shared_without_changing_turn_limits(self):
        for name in ('narrow_17','narrow_turns'):
            meta=json.loads((ROOT/'test_scenes/worlds'/f'{name}.json').read_text())
            path=Path(meta['nav_params']);path=path if path.is_absolute() else ROOT/'test_scenes/worlds'/path
            baseline=yaml.safe_load(path.read_text())
            for controller in ('scene','teb','mppi'):
                initial=configure_controller(baseline,controller,meta);frozen=yaml.safe_dump(initial)
                cfg=configure_speed(initial,.3);smooth=cfg['velocity_smoother']['ros__parameters']
                follow=cfg['controller_server']['ros__parameters']['FollowPath']
                self.assertEqual(smooth['max_velocity'][0],.3)
                self.assertEqual(follow.get('vx_max',follow.get('max_vel_x')),.3)
                self.assertEqual(smooth['max_velocity'][2],initial['velocity_smoother']['ros__parameters']['max_velocity'][2])
                self.assertEqual(smooth['min_velocity'][0],-.2)
                if 'vx_min' in follow:self.assertEqual(follow['vx_min'],-.2)
                if 'max_vel_x_backwards' in follow:self.assertEqual(follow['max_vel_x_backwards'],.2)
                self.assertEqual(yaml.safe_dump(initial),frozen)
        for speed in (0,-1,.6,float('nan')):
            with self.assertRaises(ValueError):configure_speed({},speed)


if __name__ == '__main__':
    unittest.main()
