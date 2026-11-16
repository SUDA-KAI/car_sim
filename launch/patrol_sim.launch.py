from pathlib import Path
import json
import math
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction, Shutdown, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
ROOT=Path(__file__).resolve().parents[1]
import sys
sys.path.insert(0,str(ROOT/'scripts'))
from navigation_config import configure_controller,configure_speed,configure_yaw_rate,configure_planning_timeout
from ultrasonic_config import load_mounting,attach_sensors,ultrasonic_layer_params,DEFAULT_SETTINGS
from sensor_visuals import attach_visuals,resolve_resource_paths
from l2_sensor_config import attach_inertial_lidar,bridges as l2_bridges,DEFAULT_CONFIG as L2_CONFIG
from assembly_config import APPLIED,load_applied,attach_visuals as attach_assembly_visuals,ultrasonic_poses
from stereo_config import load_calibration,calibrated_mounts,attach_stereo,bridges as camera_bridges,DEFAULT_CALIBRATION
from build_sensor_description import world_transforms
from sensor_localization_config import sensor_only_bridges
from split_drive_world import install_split_drive, bridges as split_drive_bridges
from d6_sensor_config import load_config as load_d6_config, add_mounts as add_d6_mounts, attach_sensors as attach_d6_sensors, bridges as d6_bridges

def start(context):
    world=Path(LaunchConfiguration('world').perform(context)).resolve()
    metadata=json.loads(world.with_suffix('.json').read_text())
    # New bundles use paths relative to the metadata file.  Continue accepting
    # historical absolute metadata for backwards compatibility.
    for key in ('map','nav_params','route'):
        if metadata.get(key) and not Path(metadata[key]).is_absolute():
            metadata[key]=str((world.parent/metadata[key]).resolve())
    x,y,yaw=metadata['spawn']
    teleop=LaunchConfiguration('teleop').perform(context).lower()=='true'
    nav_enabled=LaunchConfiguration('nav').perform(context).lower()=='true' and not teleop
    localization=LaunchConfiguration('localization').perform(context)
    if localization=='auto':localization=metadata.get('localization','amcl')
    if localization not in ('amcl','ground_truth','sensors'):raise ValueError('localization must be auto, amcl, ground_truth or sensors')
    sensors_mode=localization=='sensors'
    fixed_gate=nav_enabled and localization=='amcl' and LaunchConfiguration('fixed_map_safety_gate').perform(context).lower()=='true'
    use_cameras=sensors_mode or LaunchConfiguration('use_cameras').perform(context).lower()=='true'
    use_d6=LaunchConfiguration('use_d6').perform(context).lower()=='true'
    sim_drive_adapter=LaunchConfiguration('sim_drive_adapter').perform(context).lower()=='true' and nav_enabled
    d6_config=load_d6_config() if use_d6 else None
    calibration=load_calibration(LaunchConfiguration('camera_calibration').perform(context),
        int(LaunchConfiguration('camera_width').perform(context))) if use_cameras else None
    if sensors_mode and LaunchConfiguration('record_metrics').perform(context).lower()=='true':
        raise ValueError('Sensor mode does not bridge simulator truth; use independent Gazebo evaluation, not record_metrics:=true')
    if sensors_mode and LaunchConfiguration('patrol').perform(context).lower()=='true':
        route_path=LaunchConfiguration('route').perform(context)
        import yaml
        if not route_path or yaml.safe_load(Path(route_path).read_text()).get('coordinate_reference')!='sensor_map':
            raise ValueError('Sensor mode requires an explicitly recorded sensor_map route. World-frame test waypoints would leak spawn alignment.')
    if localization=='ground_truth' and not metadata.get('has_ground_truth'):
        raise ValueError('This world has no ground-truth publisher')
    # Override initial pose in a generated file, so it always matches world metadata.
    import yaml
    import tempfile
    use_assembly=LaunchConfiguration('use_assembly').perform(context).lower()=='true'
    assembly_state=assembly_assets=None
    mounting_path=LaunchConfiguration('sensor_mounting').perform(context)
    if use_assembly:
        assembly_state,assembly_assets,assembly_frames=load_applied(LaunchConfiguration('assembly_config').perform(context))
        if mounting_path:assembly_frames.update(yaml.safe_load(Path(mounting_path).read_text()))
        with tempfile.NamedTemporaryFile(mode='w',suffix='.yaml',prefix='patrol_assembly_tf_',delete=False) as file:
            yaml.safe_dump(assembly_frames,file);mounting_path=file.name
    elif not mounting_path:mounting_path=str(ROOT/'sensor_mounting_tf.yaml')
    if use_cameras:
        camera_frames=calibrated_mounts(yaml.safe_load(Path(mounting_path).read_text()),calibration)
        with tempfile.NamedTemporaryFile(mode='w',suffix='.yaml',prefix='patrol_calibrated_tf_',delete=False) as file:
            yaml.safe_dump(camera_frames,file);mounting_path=file.name
    if use_d6:
        d6_mounts=add_d6_mounts(yaml.safe_load(Path(mounting_path).read_text()),d6_config)
        with tempfile.NamedTemporaryFile(mode='w',suffix='.yaml',prefix='patrol_d6_tf_',delete=False) as file:
            yaml.safe_dump(d6_mounts,file);mounting_path=file.name
    params=yaml.safe_load(Path(metadata.get('nav_params',str(ROOT/'config/nav2_sim.yaml'))).read_text())
    if sim_drive_adapter:
        # Scene baselines include forward-only Dubins settings.  The deployed
        # Nav2 path permits direction changes; respect the physical 40 deg
        # steering cap when deriving the simulated minimum radius.
        planner=params['planner_server']['ros__parameters']['GridBased']
        planner['motion_model_for_search']='REEDS_SHEPP'
        planner['minimum_turning_radius']=max(1.0,0.90/math.tan(math.radians(40.0)))
    # Scene YAMLs may have been generated on another machine. Resolve their
    # bundled behavior-tree XML files after unpacking, before writing Nav2's
    # temporary runtime parameters.
    navigator=params['bt_navigator']['ros__parameters']
    for key in ('default_nav_to_pose_bt_xml','default_nav_through_poses_bt_xml'):
        path=Path(navigator.get(key,''))
        bundled=ROOT/'config'/path.name
        if path.name and bundled.is_file():navigator[key]=str(bundled)
    controller=LaunchConfiguration('controller').perform(context)
    params=configure_controller(params,controller,metadata)
    max_speed=float(LaunchConfiguration('max_speed').perform(context))
    params=configure_speed(params,max_speed)
    max_yaw_rate=float(LaunchConfiguration('max_yaw_rate').perform(context))
    params=configure_yaw_rate(params,max_yaw_rate)
    params=configure_planning_timeout(params,LaunchConfiguration('planning_timeout').perform(context))
    ultrasonic=LaunchConfiguration('use_ultrasonic').perform(context).lower()=='true'
    if sim_drive_adapter and (localization!='amcl' or controller!='mppi' or not ultrasonic):
        raise ValueError('sim_drive_adapter requires AMCL, MPPI and four A22 sensors')
    use_l2=LaunchConfiguration('use_l2').perform(context).lower()=='true'
    multi_amcl=localization=='amcl' and use_l2 and use_d6
    pointlio=LaunchConfiguration('pointlio').perform(context).lower()=='true'
    if pointlio and not use_l2:raise ValueError('pointlio:=true requires use_l2:=true')
    l2_config_path=LaunchConfiguration('l2_config').perform(context)
    if sensors_mode:
        # The sparse upper-hemisphere rays miss long-range walls in a 12 cm
        # near-horizontal band. Mapping projects observed vertical wall points
        # into XY; this is not a low-obstacle detector or an overhang model.
        l2_params=yaml.safe_load(Path(l2_config_path).read_text())
        l2_params['lidar']['rate_hz']=3.0
        l2_params['scan']['max_height_m']=float(LaunchConfiguration('sensor_scan_max_height').perform(context))
        with tempfile.NamedTemporaryFile(mode='w',suffix='.yaml',prefix='patrol_mapping_l2_',delete=False) as file:
            yaml.safe_dump(l2_params,file);l2_config_path=file.name
    mounting=[]
    if ultrasonic:
        mounting=load_mounting(LaunchConfiguration('ultrasonic_settings').perform(context),
            float(LaunchConfiguration('ultrasonic_height').perform(context)))
        if use_assembly:mounting=ultrasonic_poses(mounting,assembly_frames)
        params=ultrasonic_layer_params(params,mounting,LaunchConfiguration('ultrasonic_fusion').perform(context))
        if (sensors_mode or fixed_gate) and params['local_costmap']['local_costmap']['ros__parameters']['ultrasonic_layer']['plugin']=='nav2_costmap_2d::RangeSensorLayer':
            # Humble's range layer can latch current_=false after the first
            # startup timeout. The command gate independently checks all four
            # fresh Range messages, without clearing real obstacle observations.
            params['local_costmap']['local_costmap']['ros__parameters']['ultrasonic_layer']['no_readings_timeout']=0.0
    if nav_enabled and controller=='mppi':
        get_package_share_directory('nav2_mppi_controller')
    if sensors_mode:
        # Online maps retain one-beam unknown gaps. The observed-corridor goal
        # generator limits targets to lidar visibility; retain lethal obstacle
        # checking while allowing Smac's footprint to overlap unknown cells.
        params['planner_server']['ros__parameters']['GridBased']['allow_unknown']=True
        params['amcl']['ros__parameters'].pop('initial_pose',None)
        params['amcl']['ros__parameters']['set_initial_pose']=False
        # SLAM supplies the map; live lidar and sonar handle newly observed obstacles.
        for name in ('global_costmap','local_costmap'):
            costmap=params[name][name]['ros__parameters']
            costmap['track_unknown_space']=True
            if name=='local_costmap' and 'static_layer' in costmap['plugins']:
                # An expanding online map made the rolling local window grow
                # with the entire SLAM map and delayed MPPI during turns.
                costmap['plugins'].remove('static_layer')
            if name=='global_costmap':
                # Online SLAM has unknown wedges between sparse L2 rays. Plan
                # within the observed scan around the robot; SLAM still serves
                # the map display and map->odom estimate.
                costmap['plugins'].remove('static_layer')
                costmap['rolling_window']=True
                costmap['width']=8
                costmap['height']=8
            if 'laser_layer' not in costmap['plugins']:
                costmap['plugins'].insert(costmap['plugins'].index('inflation_layer'),'laser_layer')
            costmap['laser_layer']=dict(plugin='nav2_costmap_2d::ObstacleLayer',enabled=True,
                observation_sources='scan',scan=dict(topic='/scan',data_type='LaserScan',marking=True,clearing=True,
                obstacle_max_range=15.,raytrace_max_range=20.,max_obstacle_height=2.5))
    else:params['amcl']['ros__parameters']['initial_pose']=dict(x=float(x),y=float(y),z=0.0,yaw=float(yaw))
    if multi_amcl:
        # Mirror the commissioned fixed-map localization chain: wheel odom,
        # acquisition-time-aligned L2 + front/rear D6 endpoints for AMCL.
        actual=yaml.safe_load((ROOT/'config/amcl_current.yaml').read_text())['amcl']['ros__parameters']
        amcl=params['amcl']['ros__parameters']
        amcl.update(actual)
        amcl['use_sim_time']=True
        amcl['scan_topic']='/localization/multi_lidar_scan'
        amcl['set_initial_pose']=True
        amcl['initial_pose']=dict(x=float(x),y=float(y),z=0.0,yaw=float(yaw))
        # The simulated L2 does not model glass returns or the real confidence
        # filter. Keep localization endpoints out of obstacle marking.
        for name in ('local_costmap','global_costmap'):
            costmap=params[name][name]['ros__parameters']
            costmap['plugins']=[p for p in costmap['plugins'] if p not in ('obstacle_layer','inflation_layer')]
    if use_d6:
        local=params['local_costmap']['local_costmap']['ros__parameters']
        if 'd6_layer' not in local['plugins']:
            position=local['plugins'].index('inflation_layer') if 'inflation_layer' in local['plugins'] else len(local['plugins'])
            local['plugins'].insert(position,'d6_layer')
        local['d6_layer']=dict(plugin='nav2_costmap_2d::ObstacleLayer',enabled=True,
            observation_sources='front rear',combination_method=1,
            min_obstacle_height=0.05,max_obstacle_height=0.80)
        for name,key in (('front','b'),('rear','a')):
            local['d6_layer'][name]=dict(topic=f'/sim/d6_{name}_obstacle_cloud',data_type='PointCloud2',
                marking=True,clearing=False,obstacle_min_range=0.05,obstacle_max_range=4.0,
                min_obstacle_height=0.05,max_obstacle_height=0.80,
                observation_persistence=0.2)
    import tempfile
    with tempfile.NamedTemporaryFile(mode='w',suffix='.yaml',prefix='patrol_nav_',delete=False) as f:
        yaml.safe_dump(params,f); runtime=f.name
    nav=IncludeLaunchDescription(PythonLaunchDescriptionSource(str(Path(get_package_share_directory('nav2_bringup'))/'launch/bringup_launch.py')), condition=IfCondition(LaunchConfiguration('nav')), launch_arguments={'map':metadata['map'],'params_file':runtime,'use_sim_time':'true','use_composition':'False','autostart':'true'}.items())
    bridge=yaml.safe_load((ROOT/'config/bridge.yaml').read_text())
    if use_l2:
        bridge=[item for item in bridge if item['ros_topic_name']!='/scan']
        bridge+=l2_bridges()
    if use_d6:bridge+=d6_bridges()
    if use_cameras:bridge+=camera_bridges()
    for sensor in mounting:
        bridge.append(dict(ros_topic_name=f"/sim/{sensor['frame']}/scan",gz_topic_name=f"/sim/{sensor['frame']}/scan",
            ros_type_name='sensor_msgs/msg/LaserScan',gz_type_name='ignition.msgs.LaserScan',direction='GZ_TO_ROS'))
    if sensors_mode:
        # Neither perfect Gazebo poses nor wheel odometry/TF enter this graph.
        bridge=sensor_only_bridges(bridge)
    elif metadata.get('has_ground_truth'):
        for item in bridge:
            if item['ros_topic_name']=='/chassis/odom':
                item['ros_topic_name']='/wheel/odom';item['gz_topic_name']='/sim/wheel_odom'
            if item['ros_topic_name']=='/tf':item['gz_topic_name']='/sim/wheel_tf'
        bridge.append(dict(ros_topic_name='/ground_truth/odom',gz_topic_name='/sim/ground_truth',
            ros_type_name='nav_msgs/msg/Odometry',gz_type_name='ignition.msgs.Odometry',direction='GZ_TO_ROS'))
        if localization=='ground_truth':
            for item in bridge:
                if item['ros_topic_name']=='/tf':item['gz_topic_name']='/sim/truth_tf'
            bridge.append(dict(ros_topic_name='/chassis/odom',gz_topic_name='/sim/ground_truth',
                ros_type_name='nav_msgs/msg/Odometry',gz_type_name='ignition.msgs.Odometry',direction='GZ_TO_ROS'))
        else:
            bridge.append(dict(ros_topic_name='/chassis/odom',gz_topic_name='/sim/wheel_odom',
                ros_type_name='nav_msgs/msg/Odometry',gz_type_name='ignition.msgs.Odometry',direction='GZ_TO_ROS'))
    # The stock Ackermann plugin combines steering and propulsion. Split
    # actuator topics are bridged after the sensor-only localization filter.
    bridge=[item for item in bridge if item['gz_topic_name'] not in
            ('/sim/cmd_vel','/sim/odom','/sim/wheel_odom','/sim/tf','/sim/wheel_tf')]
    bridge+=split_drive_bridges()
    with tempfile.NamedTemporaryFile(mode='w',suffix='.yaml',prefix='patrol_bridge_',delete=False) as f:
        yaml.safe_dump(bridge,f);bridge_file=f.name
    simulation_world=world
    measured=yaml.safe_load(Path(mounting_path).read_text()) or {}
    if ultrasonic:
        with tempfile.NamedTemporaryFile(suffix='.sdf',prefix='patrol_ultrasonic_',delete=False) as f:
            simulation_world=Path(f.name)
        attach_sensors(world,simulation_world,mounting)
    if LaunchConfiguration('sensor_visuals').perform(context).lower()=='true':
        source=simulation_world
        with tempfile.NamedTemporaryFile(suffix='.sdf',prefix='patrol_measured_sensors_',delete=False) as f:
            simulation_world=Path(f.name)
        measured,_=attach_visuals(source,simulation_world,mounting_path,
            axes=LaunchConfiguration('show_sensor_axes').perform(context).lower()=='true',
            l2_mesh=assembly_assets['l2']['path'] if use_assembly else None)
        if use_assembly:attach_assembly_visuals(simulation_world,simulation_world,assembly_state,assembly_assets,mounting_path)
    if use_l2:
        source=simulation_world
        with tempfile.NamedTemporaryFile(suffix='.sdf',prefix='patrol_l2_',delete=False) as f:
            simulation_world=Path(f.name)
        measured=attach_inertial_lidar(source,simulation_world,mounting_path,
            l2_config_path)
    if use_d6:
        source=simulation_world
        with tempfile.NamedTemporaryFile(suffix='.sdf',prefix='patrol_d6_',delete=False) as f:
            simulation_world=Path(f.name)
        attach_d6_sensors(source,simulation_world,d6_config)
    if use_assembly and not measured:measured=assembly_frames
    if use_cameras:
        source=simulation_world
        with tempfile.NamedTemporaryFile(suffix='.sdf',prefix='patrol_stereo_',delete=False) as f:simulation_world=Path(f.name)
        attach_stereo(source,simulation_world,yaml.safe_load(Path(mounting_path).read_text()),calibration,
            float(LaunchConfiguration('camera_rate').perform(context)))
    if sensors_mode and LaunchConfiguration('perception_texture').perform(context).lower()=='true':
        from perception_texture import paint_scene
        source=simulation_world
        with tempfile.NamedTemporaryFile(suffix='.sdf',prefix='patrol_textured_',delete=False) as f:simulation_world=Path(f.name)
        paint_scene(source,simulation_world)
    if sensors_mode:
        import xml.etree.ElementTree as ET
        factor=float(LaunchConfiguration('sensor_realtime_factor').perform(context))
        if not 0.05<=factor<=1.:raise ValueError('sensor_realtime_factor must be within [0.05, 1]')
        source=simulation_world
        with tempfile.NamedTemporaryFile(suffix='.sdf',prefix='patrol_sensor_paced_',delete=False) as f:simulation_world=Path(f.name)
        tree=ET.parse(source);tree.find('world/physics/real_time_factor').text=str(factor)
        ET.indent(tree,space='  ');tree.write(simulation_world,encoding='utf-8',xml_declaration=True)
    source=simulation_world
    with tempfile.NamedTemporaryFile(suffix='.sdf',prefix='patrol_split_drive_',delete=False) as f:simulation_world=Path(f.name)
    install_split_drive(source,simulation_world)
    if simulation_world != world:
        resolve_resource_paths(simulation_world,world.parent)
    args=['ign','gazebo','-r','-v','3',str(simulation_world)]
    if LaunchConfiguration('gui').perform(context).lower() != 'true':
        args += ['-s','--headless-rendering']
    actions=[LogInfo(msg=f"Navigation controller: {params['controller_server']['ros__parameters']['FollowPath']['plugin']}"),
        ExecuteProcess(cmd=args,output='screen',on_exit=Shutdown()),
        Node(package='ros_gz_bridge',executable='parameter_bridge',parameters=[{'config_file':bridge_file,'use_sim_time':True}],output='screen')]
    actions.append(ExecuteProcess(cmd=['/usr/bin/python3',str(ROOT/'scripts/split_drive.py'),
        '--ros-args','-p','input_topic:='+('/sim/adapter_unused_cmd_vel' if sim_drive_adapter else '/localization/safe_cmd_vel' if sensors_mode or fixed_gate else '/cmd_vel'),
        '-p','ackermann_input_topic:='+('/sim/ackermann_cmd' if sim_drive_adapter else '' if sensors_mode or fixed_gate else '/sim/ackermann_cmd'),
        '-p','publish_nav_odom:='+str(localization=='amcl').lower(),
        '-p','publish_wheel_odom:='+str(not sensors_mode).lower(),
        '-p','publish_wheel_tf:='+str(localization=='amcl').lower(),
        '-p','use_sim_time:=true'],output='screen'))
    if sim_drive_adapter:
        actions.append(ExecuteProcess(cmd=['/usr/bin/python3',str(ROOT/'scripts/sim_nav_ackermann_adapter.py'),
            '--ros-args','-p','use_sim_time:=true',
            '-p','input_topic:='+('/localization/safe_cmd_vel' if fixed_gate else '/cmd_vel')],output='screen'))
    if fixed_gate:
        actions.append(ExecuteProcess(cmd=['/usr/bin/python3',str(ROOT/'scripts/fixed_map_command_gate.py'),
            '--ros-args','-p','use_sim_time:=true'],output='screen'))
    if LaunchConfiguration('sensor_monitor').perform(context).lower()=='true':
        actions.append(ExecuteProcess(cmd=['/usr/bin/python3',str(ROOT/'scripts/sensor_monitor.py'),
            '--threshold',LaunchConfiguration('proximity_threshold').perform(context),
            '--port',LaunchConfiguration('sensor_monitor_port').perform(context),
            '--ros-args','-p','use_sim_time:=true'],output='screen'))
    if use_l2:
        command=['/usr/bin/python3',str(ROOT/'scripts/l2_sim.py'),'--config',l2_config_path]
        if sensors_mode:command+=['--no-world-orientation']
        actions.append(ExecuteProcess(cmd=command+['--ros-args','-p','use_sim_time:=true'],output='screen'))
    if use_d6:
        actions.append(ExecuteProcess(cmd=['/usr/bin/python3',str(ROOT/'scripts/d6_sim.py'),
            '--ros-args','-p','use_sim_time:=true'],output='screen'))
    if multi_amcl:
        actions.append(LogInfo(msg='AMCL: L2 + two D6 time-aligned endpoints; wheel odom. L2 has no local obstacle marking.'))
        actions.append(ExecuteProcess(cmd=['/usr/bin/python3',str(ROOT/'scripts/amcl_multi_lidar_sim.py'),
            '--ros-args','-p','use_sim_time:=true'],output='screen'))
    else:
        actions.append(Node(package='tf2_ros',executable='static_transform_publisher',arguments=['--x','0','--y','0','--z','0.85','--frame-id','base_link','--child-frame-id','laser_frame'],parameters=[{'use_sim_time':True}]))
    pointlio_config=None
    if pointlio:
        pointlio_config=str(ROOT/'config/pointlio_sim.yaml')
        if sensors_mode:
            pointlio_params=yaml.safe_load(Path(pointlio_config).read_text())
            settings=pointlio_params['/**']['ros__parameters']
            transform=world_transforms(measured)['unilidar_imu']
            settings.update(odom_header_frame_id='lio_odom',odom_child_frame_id='lio_base_link')
            settings.update(point_filter_num=3,filter_size_surf=.2,filter_size_map=.2,use_imu_as_input=True)
            settings['publish']['base_to_imu_translation']=transform[:3,3].tolist()
            settings['publish']['base_to_imu_rotation']=transform[:3,:3].reshape(-1).tolist()
            with tempfile.NamedTemporaryFile(mode='w',suffix='.yaml',prefix='patrol_sensor_lio_',delete=False) as file:
                yaml.safe_dump(pointlio_params,file);pointlio_config=file.name
        actions.append(LogInfo(msg='Point-LIO feeds sensor fusion; no simulator pose input.' if sensors_mode else 'Point-LIO is independent of Nav2 localization.'))
        actions.append(Node(package='point_lio',executable='pointlio_mapping',name='sim_pointlio_mapping',
            parameters=[pointlio_config,{'use_sim_time':True}],
            remappings=[('/cloud_registered','/pointlio/cloud_registered'),('/cloud_registered_body','/pointlio/cloud_body'),
                ('/cloud_effected','/pointlio/effective_points'),('/Laser_map','/pointlio/local_map'),
                ('/path','/pointlio/path'),('/aft_mapped_to_init','/pointlio/odometry')],output='screen'))
    if ultrasonic:
        actions.append(LogInfo(msg='Ultrasonic: height 0.49 m by default; dashboard rear-left origin mapped to base_link at vehicle centre. Settings length does not resize STL.'))
        for sensor in ([] if sensors_mode else mounting):
            actions.append(Node(package='tf2_ros',executable='static_transform_publisher',name=f"{sensor['frame']}_mount_tf",
                arguments=['--x',str(sensor['x']),'--y',str(sensor['y']),'--z',str(sensor['z']),
                           '--roll',str(sensor.get('roll',0)),'--pitch',str(sensor.get('pitch',0)),
                           '--yaw',str(sensor['yaw']),'--frame-id','base_link','--child-frame-id',sensor['frame']],
                parameters=[{'use_sim_time':True}]))
        actions.append(ExecuteProcess(cmd=['/usr/bin/python3',str(ROOT/'scripts/ultrasonic_sim.py'),
            '--settings',LaunchConfiguration('ultrasonic_settings').perform(context),'--height',str(mounting[0]['z']),
            '--ros-args','-p','use_sim_time:=true'],output='screen'))
        actions.append(ExecuteProcess(cmd=['/usr/bin/python3',str(ROOT/'scripts/ultrasonic_projection.py'),
            '--ros-args','-p','use_sim_time:=true'],output='screen'))
    for frame,pose in ([] if sensors_mode else measured.items()):
        if ultrasonic and frame in {s['frame'] for s in mounting}:continue
        arguments=[]
        for key in ('x','y','z','roll','pitch','yaw'):arguments += ['--'+key,str(pose.get(key,0.))]
        arguments += ['--frame-id',pose['parent'],'--child-frame-id',frame]
        actions.append(Node(package='tf2_ros',executable='static_transform_publisher',name='measured_'+frame+'_tf',
            arguments=arguments,parameters=[{'use_sim_time':True}]))
    if use_cameras:
        actions += [ExecuteProcess(cmd=['/usr/bin/python3',str(ROOT/'scripts/stereo_sim.py'),'--calibration',calibration['source'],
            '--width',str(calibration['width']),'--ros-args','-p','use_sim_time:=true'],output='screen')]
    if sensors_mode:
        with tempfile.NamedTemporaryFile(mode='w',suffix='.yaml',prefix='patrol_all_sensor_tf_',delete=False) as file:
            yaml.safe_dump(measured,file);all_mounts=file.name
        actions += [LogInfo(msg='SENSOR LOCALIZATION: LIO + stereo visual velocities + external gyro -> EKF; online SLAM defines map. No spawn pose, wheel odom or truth TF supplied.'),
            ExecuteProcess(cmd=['/usr/bin/python3',str(ROOT/'scripts/static_sensor_tf.py'),'--mounting',all_mounts,'--ros-args','-p','use_sim_time:=true'],output='screen'),
            ExecuteProcess(cmd=['/usr/bin/python3',str(ROOT/'scripts/stereo_odometry.py'),'--calibration',calibration['source'],
                '--width',str(calibration['width']),'--mounting',all_mounts,'--ros-args','-p','use_sim_time:=true'],output='screen'),
            ExecuteProcess(cmd=['/usr/bin/python3',str(ROOT/'scripts/planar_lidar_odometry.py'),'--ros-args','-p','use_sim_time:=true'],output='screen'),
            ExecuteProcess(cmd=['/usr/bin/python3',str(ROOT/'scripts/sensor_command_gate.py'),'--ros-args','-p','use_sim_time:=true'],output='screen'),
            Node(package='robot_localization',executable='ekf_node',name='ekf_filter_node',
                parameters=[str(ROOT/'config/sensor_ekf.yaml')],remappings=[('odometry/filtered','/chassis/odom')],output='screen'),
            Node(package='slam_toolbox',executable='async_slam_toolbox_node',name='slam_toolbox',parameters=[LaunchConfiguration('sensor_slam_config').perform(context)],output='screen')]
        audit=dict(mode='sensors',bridge=bridge,mounting=all_mounts,pointlio_config=pointlio_config,
            stereo_calibration=calibration['source'],calibration_sha256=calibration['sha256'],baseline_m=calibration['baseline_m'],
            camera_width=calibration['width'],camera_height=calibration['height'],
            camera_lidar_imu_extrinsics='measured mounting TF, not joint calibration',runtime_nav_params=runtime,
            slam_config=LaunchConfiguration('sensor_slam_config').perform(context),map_source='online slam_toolbox, no preloaded world map',simulator_world=str(simulation_world),l2_config=l2_config_path)
        (ROOT/'artifacts/sensor_localization_current.json').write_text(json.dumps(audit,indent=2)+'\n')
    if LaunchConfiguration('rviz').perform(context).lower()=='true':
        actions.append(Node(package='rviz2',executable='rviz2',arguments=['-d',str(ROOT/'config/sensor_navigation.rviz')],
            parameters=[{'use_sim_time':True}],output='screen'))
    if teleop:
        config=LaunchConfiguration('joy_config').perform(context)
        actions += [LogInfo(msg='Joystick mode: Nav2 disabled; /joy -> Ackermann /cmd_vel.'),
            Node(package='joy',executable='joy_node',name='sim_joy',parameters=[config],output='screen'),
            ExecuteProcess(cmd=['/usr/bin/python3',str(ROOT/'scripts/joystick_control.py'),'--ros-args','--params-file',config,
                '-p','max_forward_speed:='+str(max_speed)],output='screen',on_exit=Shutdown())]
    elif nav_enabled:
        if sensors_mode:
            actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(str(Path(get_package_share_directory('nav2_bringup'))/'launch/navigation_launch.py')),
                launch_arguments={'params_file':runtime,'use_sim_time':'true','use_composition':'False','autostart':'true'}.items()))
        elif localization=='ground_truth':
            actions += [LogInfo(msg='TEST BASELINE: ideal 3D ground-truth localization; static terrain map.'),
                Node(package='tf2_ros',executable='static_transform_publisher',arguments=['--frame-id','map','--child-frame-id','odom'],parameters=[{'use_sim_time':True}]),
                Node(package='nav2_map_server',executable='map_server',name='map_server',parameters=[{'use_sim_time':True,'yaml_filename':metadata['map']}],output='screen'),
                Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',name='lifecycle_manager_test_map',parameters=[{'use_sim_time':True,'autostart':True,'node_names':['map_server']}],output='screen'),
                IncludeLaunchDescription(PythonLaunchDescriptionSource(str(Path(get_package_share_directory('nav2_bringup'))/'launch/navigation_launch.py')),
                    launch_arguments={'params_file':runtime,'use_sim_time':'true','use_composition':'False','autostart':'true'}.items())]
        else:actions.append(nav)
    output=None
    if LaunchConfiguration('record_metrics').perform(context).lower()=='true':
        if not metadata.get('has_ground_truth'):raise ValueError('Metrics require a test scene with ground truth')
        import datetime,zoneinfo
        stamp=datetime.datetime.now(zoneinfo.ZoneInfo('Asia/Shanghai')).strftime('%Y%m%d_%H%M%S')
        tag='' if controller=='scene' else f'_{controller}'
        output=ROOT/'test_scenes/results'/f"{metadata.get('name','scene')}{tag}_{stamp}"
        output.with_suffix('.nav.yaml').write_text(yaml.safe_dump(params))
        output.with_suffix('.run.json').write_text(json.dumps(dict(scene=metadata.get('name'),controller=controller,
            controller_plugin=params['controller_server']['ros__parameters']['FollowPath']['plugin'],
            localization=localization,world=str(world),ros_domain_id=os.environ.get('ROS_DOMAIN_ID'),
            use_ultrasonic=ultrasonic,ultrasonic_mounting=mounting,use_l2=use_l2,pointlio=pointlio,
            ultrasonic_fusion=LaunchConfiguration('ultrasonic_fusion').perform(context),
            sensor_mounting=mounting_path,use_assembly=use_assembly,
            assembly_revision=assembly_state['revision'] if use_assembly else None,
            max_forward_speed_mps=max_speed,
            max_yaw_rate_radps=max_yaw_rate,
            scene_metadata=metadata,
            l2_config=l2_config_path),indent=2))
    if LaunchConfiguration('patrol').perform(context).lower()=='true':
        actions.append(LogInfo(msg='正在初始化 Nav2，完成后自动发送巡逻航点；首次加载请等待约30–60秒。'))
        if not nav_enabled:raise ValueError('Automatic patrol needs nav:=true and teleop:=false')
        route=LaunchConfiguration('route').perform(context) or metadata.get('route','')
        if not route:raise ValueError('Specify route:=/path/to/route.yaml')
        command=['/usr/bin/python3',str(ROOT/'scripts/patrol.py'),'--route',route]
        command += ['--loops',LaunchConfiguration('patrol_loops').perform(context)]
        report=LaunchConfiguration('patrol_report').perform(context)
        if report:command += ['--report',report]
        elif output is not None:command += ['--report',str(output.with_suffix('.patrol.json'))]
        actions.append(ExecuteProcess(cmd=command,output='screen'))
    if output is not None:
        command=['/usr/bin/python3',str(ROOT/'scripts/record_test_metrics.py'),
            '--scene',str(world.with_suffix('.json')),'--output',str(output),'--localization',localization]
        if LaunchConfiguration('patrol').perform(context).lower()=='true':
            command += ['--patrol-report',str(output.with_suffix('.patrol.json'))]
        actions.append(ExecuteProcess(cmd=command,output='screen'))
    return actions

def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('world',default_value=str(ROOT/'worlds/map_20260817_154310_filtered.sdf')),
        DeclareLaunchArgument('teleop',default_value='false'),
        DeclareLaunchArgument('localization',default_value='auto'),
        DeclareLaunchArgument('fixed_map_safety_gate',default_value='false',description='Require fresh lidar, wheel odometry and four sonar readings before driving in AMCL mode'),
        DeclareLaunchArgument('sim_drive_adapter',default_value='false',description='Use steering feedback, A22/AMCL freshness gate and pre-steer command adapter'),
        DeclareLaunchArgument('controller',default_value='scene',description='scene / teb / mppi (sampling-based MPC)'),
        DeclareLaunchArgument('max_speed',default_value='0.5',description='Simulation forward speed cap in m/s; chassis hard cap is 0.5 m/s and reverse is 0.2 m/s'),
        DeclareLaunchArgument('planning_timeout',default_value='0',description='Global search seconds: 0 keeps scene default; otherwise 1–60'),
        DeclareLaunchArgument('max_yaw_rate',default_value='0.4',description='Navigation and velocity smoother yaw-rate cap in rad/s'),
        DeclareLaunchArgument('use_ultrasonic',default_value='true'),
        DeclareLaunchArgument('ultrasonic_fusion',default_value='range',description='range: probabilistic RangeSensorLayer; arcs: legacy hard obstacle arcs'),
        DeclareLaunchArgument('sensor_monitor',default_value='true',description='Local sensor viewer and advisory ultrasonic alarm'),
        DeclareLaunchArgument('sensor_monitor_port',default_value='8768'),
        DeclareLaunchArgument('proximity_threshold',default_value='0.1',description='Ultrasonic warning distance in metres; does not stop the car'),
        DeclareLaunchArgument('use_l2',default_value='true'),
        DeclareLaunchArgument('use_d6',default_value='true',description='Two 10 Hz low-mounted D6 scans and Nav2 local obstacles'),
        DeclareLaunchArgument('l2_config',default_value=str(L2_CONFIG)),
        DeclareLaunchArgument('pointlio',default_value='false',description='Optional sensor-centric LIO; does not replace Nav2 localization'),
        DeclareLaunchArgument('use_cameras',default_value='false'),
        DeclareLaunchArgument('camera_calibration',default_value=str(DEFAULT_CALIBRATION)),
        DeclareLaunchArgument('camera_width',default_value='640'),
        DeclareLaunchArgument('camera_rate',default_value='6'),
        DeclareLaunchArgument('sensor_scan_max_height',default_value='1.2',description='Mapping-only upper-hemisphere wall projection height in lidar frame'),
        DeclareLaunchArgument('sensor_realtime_factor',default_value='0.3',description='Sensor-mode target simulation/wall time ratio; physics step unchanged'),
        DeclareLaunchArgument('perception_texture',default_value='true',description='Paint existing scene visuals for stereo features; collision geometry unchanged'),
        DeclareLaunchArgument('sensor_slam_config',default_value=str(ROOT/'config/sensor_slam.yaml')),
        DeclareLaunchArgument('rviz',default_value='false'),
        DeclareLaunchArgument('ultrasonic_settings',default_value=str(DEFAULT_SETTINGS)),
        DeclareLaunchArgument('ultrasonic_height',default_value='0.49'),
        DeclareLaunchArgument('sensor_visuals',default_value='true'),
        DeclareLaunchArgument('sensor_mounting',default_value='',description='Optional TF override; defaults to applied assembly or measured YAML'),
        DeclareLaunchArgument('use_assembly',default_value='true' if APPLIED.exists() else 'false'),
        DeclareLaunchArgument('assembly_config',default_value=str(APPLIED)),
        DeclareLaunchArgument('show_sensor_axes',default_value='false'),
        DeclareLaunchArgument('patrol',default_value='false'),
        DeclareLaunchArgument('patrol_loops',default_value='1'),
        DeclareLaunchArgument('patrol_report',default_value=''),
        DeclareLaunchArgument('route',default_value=''),
        DeclareLaunchArgument('record_metrics',default_value='false'),
        DeclareLaunchArgument('joy_config',default_value=str(ROOT/'config/joystick.yaml')),
        DeclareLaunchArgument('gui',default_value='true'),DeclareLaunchArgument('nav',default_value='true'),OpaqueFunction(function=start)])
