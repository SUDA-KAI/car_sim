"""Apply optional simulation controller overrides without changing real-car config."""
from copy import deepcopy
import json
import math
from pathlib import Path
import yaml

ROOT = Path(__file__).resolve().parents[1]


def configure_planning_timeout(params, timeout):
    """Override search time only; keep footprint, map and steering constraints."""
    timeout = float(timeout)
    if not math.isfinite(timeout) or timeout != 0 and not 1 <= timeout <= 60:
        raise ValueError('planning_timeout must be 0 (scene default) or 1–60 seconds')
    result = deepcopy(params)
    if timeout:
        result['planner_server']['ros__parameters']['GridBased']['max_planning_time'] = timeout
    return result


def configure_yaw_rate(params, max_yaw_rate):
    """Share the simulation yaw-rate cap between navigation and smoothing."""
    max_yaw_rate=float(max_yaw_rate)
    if not math.isfinite(max_yaw_rate) or not 0<max_yaw_rate<=1.0:
        raise ValueError('max_yaw_rate must be finite and in (0, 1.0] rad/s')
    params=deepcopy(params)
    smoother=params['velocity_smoother']['ros__parameters']
    smoother['max_velocity'][2]=max_yaw_rate
    smoother['min_velocity'][2]=-max_yaw_rate
    follow=params['controller_server']['ros__parameters']['FollowPath']
    for key in ('wz_max','max_vel_theta'):
        if key in follow:follow[key]=max_yaw_rate
    return params


def configure_speed(params,max_speed):
    """One simulation forward cap for the selected controller and smoother."""
    max_speed=float(max_speed)
    if not math.isfinite(max_speed) or not 0<max_speed<=.5:
        raise ValueError('max_speed must be finite and in (0, 0.5] m/s')
    params=deepcopy(params)
    smoother=params['velocity_smoother']['ros__parameters']
    smoother['max_velocity'][0]=max_speed
    reverse_cap=.10 if params['controller_server']['ros__parameters']['FollowPath']['plugin']=='nav2_mppi_controller::MPPIController' else .20
    smoother['min_velocity'][0]=-reverse_cap
    follow=params['controller_server']['ros__parameters']['FollowPath']
    for key in ('vx_max','max_vel_x'):
        if key in follow:follow[key]=max_speed
    if 'vx_min' in follow:follow['vx_min']=-reverse_cap
    if 'max_vel_x_backwards' in follow:follow['max_vel_x_backwards']=reverse_cap
    if follow['plugin']=='nav2_mppi_controller::MPPIController':
        # Keep the prediction horizon and its path inside the rolling map even
        # when a launch override raises forward speed. Include the full body.
        local=params['local_costmap']['local_costmap']['ros__parameters']
        footprint=json.loads(local['footprint'])
        margin=max(math.hypot(*point) for point in footprint)+float(local.get('footprint_padding',0))+.2
        travel=max_speed*float(follow['model_dt'])*int(follow['time_steps'])
        size=math.ceil(2*(travel+margin))
        local['width']=max(local['width'],size)
        local['height']=max(local['height'],size)
        available=min(local['width'],local['height'])/2-margin
        follow['prune_distance']=min(available,max(2.,travel+.2))
    return params


def configure_controller(params, controller, metadata):
    params = deepcopy(params)
    if controller not in ('scene', 'teb', 'mppi'):
        raise ValueError('controller must be scene, teb or mppi')
    if controller == 'scene':
        return params
    server = params['controller_server']['ros__parameters']
    original = server['FollowPath']
    if controller == 'teb':
        if original['plugin'] != 'teb_local_planner::TebLocalPlannerROS':
            raise ValueError('Selected scene does not provide a TEB baseline')
        return params
    mppi = yaml.safe_load((ROOT / 'config/controller_mppi.yaml').read_text())
    # Keep the selected scene's conservative speed bounds and planner curvature.
    smoother = params['velocity_smoother']['ros__parameters']
    mppi['vx_max'] = float(smoother['max_velocity'][0])
    mppi['vx_min'] = float(smoother['min_velocity'][0])
    mppi['wz_max'] = float(smoother['max_velocity'][2])
    planner = params['planner_server']['ros__parameters']['GridBased']
    mppi['AckermannConstraints']['min_turning_r'] = float(planner['minimum_turning_radius'])
    server['FollowPath'] = mppi
    server['controller_frequency'] = 10.0
    # Low-speed Ackermann turns need small yaw rates; don't erase them downstream.
    smoother['deadband_velocity'] = [0.0, 0.0, 0.0]
    server['min_theta_velocity_threshold'] = 0.001
    # Account for the tires' full steering sweep, keeping the actual model intact.
    if metadata.get('wheel_sweep_width_bound_m'):
        half_width = float(metadata['wheel_sweep_width_bound_m']) / 2
        half_length = float(metadata['robot_length_m']) / 2
        footprint = json.dumps([[half_length, half_width], [half_length, -half_width],
                                [-half_length, -half_width], [-half_length, half_width]])
        for name in ('local_costmap', 'global_costmap'):
            params[name][name]['ros__parameters']['footprint'] = footprint
    return params
