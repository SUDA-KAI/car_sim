#!/usr/bin/python3
"""One observed-corridor lap: local laser goals -> Nav2, no world/spawn inputs."""
import argparse
import json
import math
import os
from pathlib import Path
import time
import numpy as np
from scipy.spatial import cKDTree


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def choose_arc(points, lookahead=1.8):
    """Follow the bisector of observed walls; return its endpoint AND tangent.

    Constant-curvature averaging understeers at straight-to-curve transitions.
    Local quadratic fits preserve the tangent required by the car's goal pose.
    No scene shape, absolute pose, or nominal turning radius is supplied.
    """
    points=np.asarray(points,dtype=float)
    points=points[np.all(np.isfinite(points),axis=1)]
    if len(points)<60:raise ValueError('Insufficient laser returns')
    tree=cKDTree(points);p=np.zeros(2);heading=0.;path=[]
    steps=int(math.ceil(lookahead/.1));ds=lookahead/steps
    for _ in range(steps+1):
        tangent=np.array([math.cos(heading),math.sin(heading)])
        normal=np.array([-tangent[1],tangent[0]])
        relative=points-p;longitudinal=relative@tangent;lateral=relative@normal
        near=(np.abs(longitudinal)<.65)&(np.abs(lateral)<2.5)
        fits=[]
        for side in (1,-1):
            mask=near&(lateral*side>.15)
            x=longitudinal[mask];y=lateral[mask]
            if len(x)<5 or np.ptp(x)<.3:raise ValueError('Both local wall tangents must be observed')
            weight=np.exp(-x*x/.18)
            coefficients=np.polynomial.polynomial.polyfit(x,y,2,w=weight)
            residual=np.abs(y-np.polynomial.polynomial.polyval(x,coefficients))
            keep=residual<max(.06,3*np.median(residual))
            if keep.sum()>=5:coefficients=np.polynomial.polynomial.polyfit(x[keep],y[keep],2,w=weight[keep])
            fits.append(coefficients)
        width=fits[0][0]-fits[1][0]
        if not 1.25<width<2.2:raise ValueError('Observed wall separation does not support this corridor task')
        offset=(fits[0][0]+fits[1][0])*.5
        p=p+normal*np.clip(offset,-.12,.12)
        heading+=float(np.clip(math.atan((fits[0][1]+fits[1][1])*.5),-.10,.10))
        path.append(p.copy())
        if len(path)<=steps:p=p+ds*np.array([math.cos(heading),math.sin(heading)])
    path=np.array(path);clearance=float(tree.query(path)[0].min())
    if clearance<.63:raise ValueError('No observed corridor centreline with sufficient clearance')
    angle=np.arctan2(points[:,1],points[:,0]);radius=np.linalg.norm(points,axis=1)
    order=np.argsort(angle);angle=angle[order];radius=radius[order]
    query_angle=np.arctan2(path[1:,1],path[1:,0]);indices=np.searchsorted(angle,query_angle).clip(1,len(angle)-1)
    visible=np.minimum(radius[indices-1],radius[indices])
    if np.any(np.linalg.norm(path[1:],axis=1)+.3>=visible) or np.any(angle[indices]-angle[indices-1]>.09):
        raise ValueError('Centreline leaves measured laser visibility')
    return dict(x=float(p[0]),y=float(p[1]),yaw=heading,curvature=heading/lookahead,clearance=clearance)


def main():
    import rclpy
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions
    from nav2_simple_commander.robot_navigator import BasicNavigator,TaskResult
    from lifecycle_msgs.srv import GetState
    from nav_msgs.msg import Odometry,Path as RosPath
    from sensor_msgs.msg import LaserScan
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import String
    from tf2_ros import Buffer,TransformListener
    from scipy.spatial.transform import Rotation
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--report',type=Path,default=Path(__file__).resolve().parents[1]/'artifacts/sensor_corridor_lap.json')
    parser.add_argument('--wall-timeout',type=float,default=1800)
    args=parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID')!='43':raise RuntimeError('Only isolated simulation domain 43 is allowed')
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    nav=BasicNavigator(node_name='sensor_corridor_lap');nav.set_parameters([Parameter('use_sim_time',value=True)])
    buffer=Buffer();listener=TransformListener(buffer,nav);latest={}
    subs=[nav.create_subscription(LaserScan,'/scan',lambda m:latest.update(scan=m),qos_profile_sensor_data),
          nav.create_subscription(String,'/localization/gate_status',lambda m:latest.update(gate=json.loads(m.data)),10),
          nav.create_subscription(Odometry,'/chassis/odom',lambda m:latest.update(odom=m),qos_profile_sensor_data)]
    state_client=nav.create_client(GetState,'/bt_navigator/get_state');state_future=None;nav_ready=False
    status_pub=nav.create_publisher(String,'/localization/patrol_status',10)
    path_pub=nav.create_publisher(RosPath,'/localization/patrol_path',10)
    report=dict(state='initializing',source='live /scan and estimated map TF; no world route',distance_m=0.,turn_radians=0.,goals=[],trajectory=[],completed_laps=0)
    started=time.monotonic();previous=None;origin=None;last_goal_pose=None;active=False;last_log=0;last_save=0;last_update=0;blocked_since=None;final=False;failed_goals=0
    def save():
        args.report.parent.mkdir(parents=True,exist_ok=True)
        args.report.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
        status_pub.publish(String(data=json.dumps({k:v for k,v in report.items() if k not in ('trajectory','goals')})))
    save()
    try:
        while rclpy.ok():
            rclpy.spin_once(nav,timeout_sec=.08)
            wall=time.monotonic();sim=nav.get_clock().now().nanoseconds*1e-9
            if wall-started>args.wall_timeout:raise RuntimeError('Lap wall-clock timeout')
            if not nav_ready:
                if state_future is not None and state_future.done():
                    result=state_future.result();nav_ready=result is not None and result.current_state.id==3;state_future=None
                if not nav_ready:
                    if state_future is None and state_client.service_is_ready():state_future=state_client.call_async(GetState.Request())
                    if wall-started>120:raise RuntimeError('Nav2 did not activate within 120 seconds')
                    continue
            if 'scan' not in latest or not buffer.can_transform('map','base_link',rclpy.time.Time()):continue
            transform=buffer.lookup_transform('map','base_link',rclpy.time.Time()).transform
            q=transform.rotation;yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            pose=(transform.translation.x,transform.translation.y,yaw)
            if origin is None:
                if latest.get('gate',{}).get('state')!='enabled':continue
                audit=json.loads((Path(__file__).resolve().parents[1]/'artifacts/sensor_localization_current.json').read_text())
                if audit.get('mode')!='sensors' or any(item['ros_type_name'] in ('nav_msgs/msg/Odometry','tf2_msgs/msg/TFMessage') for item in audit['bridge']):
                    raise RuntimeError('Sensor-only bridge audit failed')
                publishers=[p.node_name for p in nav.get_publishers_info_by_topic('/chassis/odom')]
                if publishers!=['ekf_filter_node']:raise RuntimeError('EKF must be the only fused-odometry publisher')
                report['truth_not_bridged']=True;report['fused_publishers']=publishers
                origin=pose;report['origin']=dict(x=pose[0],y=pose[1],yaw=yaw)
            if previous:
                d=math.hypot(pose[0]-previous[0],pose[1]-previous[1])
                if d>.4:raise RuntimeError('Estimated pose jumped; stopping lap')
                if d>=.01 or abs(wrap(yaw-previous[2]))>=.01:
                    report['distance_m']+=d;report['turn_radians']+=wrap(yaw-previous[2]);previous=pose
            else:previous=pose
            if not report['trajectory'] or math.hypot(pose[0]-report['trajectory'][-1]['x'],pose[1]-report['trajectory'][-1]['y'])>.12:
                report['trajectory'].append(dict(x=pose[0],y=pose[1],yaw=yaw,sim_time=sim))
            report['distance_to_start_m']=math.hypot(pose[0]-origin[0],pose[1]-origin[1])
            report['wall_duration_s']=wall-started
            report['pose']=dict(x=pose[0],y=pose[1],yaw=yaw)
            if wall-last_save>1:save();last_save=wall
            if latest.get('gate',{}).get('state')!='enabled':
                blocked_since=blocked_since or wall
                if wall-blocked_since>25:raise RuntimeError('Sensor validity gate blocked for 25 seconds')
                continue
            blocked_since=None
            if active and nav.isTaskComplete():
                result=nav.getResult();active=False
                if result!=TaskResult.SUCCEEDED:
                    failed_goals+=1
                    report['navigation_retries']=failed_goals
                    if final or failed_goals>3:raise RuntimeError(f'Nav2 task ended: {result}')
                    # An online map can grow during a turn. Recompute a nearer
                    # target from the latest scan instead of retrying stale XY.
                    last_goal_pose=None
                    print(f'Nav2 goal failed; selecting fresh short target ({failed_goals}/3)',flush=True)
                else:failed_goals=0
                if final:
                    report['completed_laps']=1;report['state']='succeeded';save();break
            if not final and report['distance_m']>20 and abs(report['turn_radians'])>5.5 and report['distance_to_start_m']<1.7:
                goal=origin;final=True
            elif final:continue
            else:
                if active and wall-last_update<1.:continue
                # Only preempt after real progress. Reissuing essentially the
                # same goal every second resets Nav2 recovery while stationary.
                if active and report.get('last_sent_pose') and math.hypot(pose[0]-report['last_sent_pose'][0],pose[1]-report['last_sent_pose'][1])<.65:continue
                scan=latest['scan'];age=sim-(scan.header.stamp.sec+scan.header.stamp.nanosec*1e-9)
                if not -.05<=age<=1.:continue
                angles=scan.angle_min+np.arange(len(scan.ranges))*scan.angle_increment
                ranges=np.asarray(scan.ranges);valid=np.isfinite(ranges)&(ranges>scan.range_min)&(ranges<scan.range_max)
                points=np.stack((ranges[valid]*np.cos(angles[valid]),ranges[valid]*np.sin(angles[valid]),np.zeros(valid.sum())),axis=1)
                t=buffer.lookup_transform('base_link',scan.header.frame_id,rclpy.time.Time()).transform;r=t.rotation
                points=Rotation.from_quat([r.x,r.y,r.z,r.w]).apply(points)+[t.translation.x,t.translation.y,t.translation.z]
                lookahead=1.15 if failed_goals or abs(report.get('arc',{}).get('yaw',0.))>.12 else 1.8
                try:arc=choose_arc(points[:,:2],lookahead=lookahead)
                except ValueError as e:
                    if wall-last_log>3:print(str(e),flush=True);last_log=wall
                    if not active and wall-started>90:raise
                    continue
                c,s=math.cos(yaw),math.sin(yaw)
                goal=(pose[0]+c*arc['x']-s*arc['y'],pose[1]+s*arc['x']+c*arc['y'],wrap(yaw+arc['yaw']))
                report['arc']=arc
            target=PoseStamped();target.header.frame_id='map';target.header.stamp=nav.get_clock().now().to_msg()
            target.pose.position.x=goal[0];target.pose.position.y=goal[1];target.pose.orientation.z=math.sin(goal[2]/2);target.pose.orientation.w=math.cos(goal[2]/2)
            if not nav.goToPose(target):raise RuntimeError('Nav2 rejected navigation goal')
            active=True;last_update=wall;last_goal_pose=goal;report['last_sent_pose']=pose
            report['state']='returning' if final else 'running'
            report['goals'].append(dict(x=goal[0],y=goal[1],yaw=goal[2],sim_time=sim,distance=report['distance_m'],selection='estimated_start' if final else 'observed_laser_arc'))
            report['target']=dict(x=goal[0],y=goal[1],yaw=goal[2])
            path=RosPath();path.header=target.header
            for point in report['trajectory']:
                p=PoseStamped();p.header=target.header;p.pose.position.x=point['x'];p.pose.position.y=point['y'];p.pose.orientation.w=1.;path.poses.append(p)
            path.poses.append(target);path_pub.publish(path)
            print(f"{report['state']} distance={report['distance_m']:.2f} turn={report['turn_radians']:.2f} goal={goal}",flush=True)
            save()
    except KeyboardInterrupt:report['state']='canceled'
    except Exception as error:report['state']='failed';report['reason']=str(error);print('Lap failed:',error,flush=True)
    finally:
        if active:nav.cancelTask()
        save();nav.destroy_node()
        if rclpy.ok():rclpy.shutdown()
    if report['state']!='succeeded':raise SystemExit(1)

if __name__=='__main__':main()
