#!/usr/bin/python3
"""Make a simulation-only copy of the car's Ackermann Nav2 configuration."""
import shutil
from pathlib import Path
import yaml
ROOT=Path(__file__).resolve().parents[1]
source=ROOT.parent/'src/robot_modules/navigation/robot_navigation/config'
params=yaml.safe_load((source/'nav2_params.yaml').read_text())
for name, node in params.items():
    if 'ros__parameters' in node:
        node['ros__parameters']['use_sim_time']=True
for name in ('local_costmap','global_costmap'):
    p=params[name][name]['ros__parameters']; p['use_sim_time']=True
    if name == 'local_costmap':
        p['plugins']=['obstacle_layer','inflation_layer']
        p.pop('voxel_layer',None); p.pop('range_layer',None)
        p['obstacle_layer']=dict(plugin='nav2_costmap_2d::ObstacleLayer', observation_sources='scan',
          scan=dict(topic='/scan',data_type='LaserScan',marking=True,clearing=True,
                    obstacle_max_range=20.0,raytrace_max_range=25.0,max_obstacle_height=2.0))
for key in ('default_nav_to_pose_bt_xml','default_nav_through_poses_bt_xml'):
    filename=Path(params['bt_navigator']['ros__parameters'][key]).name
    shutil.copyfile(source/filename,ROOT/'config'/filename)
    params['bt_navigator']['ros__parameters'][key]=str(ROOT/'config'/filename)
# AMCL replaces Point-LIO/GICP only inside this simulation.
params['amcl']={'ros__parameters':dict(use_sim_time=True,base_frame_id='base_link',odom_frame_id='odom',
    global_frame_id='map',scan_topic='scan',robot_model_type='nav2_amcl::DifferentialMotionModel',
    min_particles=500,max_particles=2000,laser_max_range=25.0,laser_min_range=0.15,
    transform_tolerance=1.0,set_initial_pose=True,
    initial_pose=dict(x=-5.0,y=-5.0,z=0.0,yaw=0.0))}
# DifferentialMotionModel describes odometry noise; steering is still Ackermann.
(ROOT/'config/nav2_sim.yaml').write_text(yaml.safe_dump(params,sort_keys=False))
