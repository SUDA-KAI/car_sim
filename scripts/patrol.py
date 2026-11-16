#!/usr/bin/python3
"""Sequential stops or continuous NavigateThroughPoses patrol, selected by route."""
import argparse
import math
import json
import time
from pathlib import Path
import rclpy
from rclpy.signals import SignalHandlerOptions
import yaml
from geometry_msgs.msg import PoseStamped
from lifecycle_msgs.srv import GetState
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from tf2_ros import Buffer, TransformListener
from patrol_route import route_groups

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--route',type=Path,default=Path(__file__).resolve().parents[1]/'config/patrol_map_20260817_154310_filtered.yaml')
    parser.add_argument('--report',type=Path,help='Optional per-waypoint JSON result')
    parser.add_argument('--loops',type=int,default=1,help='0 means repeat until Ctrl+C')
    parser.add_argument('--timeout',type=float,default=180.,help='simulation seconds per goal')
    parser.add_argument('--mode',choices=('auto','waypoints','through_poses'),default='auto')
    args=parser.parse_args()
    route=yaml.safe_load(args.route.read_text())
    mode=route.get('navigation_mode','waypoints') if args.mode=='auto' else args.mode
    if mode not in ('waypoints','through_poses'):parser.error('Unknown route navigation_mode')
    groups=route_groups(route,mode)
    route_timeout=float(route.get('route_timeout_s',args.timeout*len(route['waypoints'])))
    if not math.isfinite(route_timeout) or route_timeout<=0:parser.error('Invalid route_timeout_s')
    if args.loops < 0 or args.timeout <= 0 or not route['waypoints']:
        parser.error('loops >= 0, timeout > 0 and nonempty waypoints are required')
    report={'route':str(args.route.resolve()),'requested_loops':args.loops,'navigation_mode':mode,'result':'running','waypoints':[],'segments':[],'laps':[]}
    wall_start=time.monotonic()
    def save_report():
        if args.report:
            args.report.parent.mkdir(parents=True,exist_ok=True)
            report['wall_duration_s']=time.monotonic()-wall_start
            args.report.write_text(json.dumps(report,indent=2))
    save_report()
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO); nav=BasicNavigator(); nav.set_parameters([rclpy.parameter.Parameter('use_sim_time',value=True)])
    try:
        # Launch owns AMCL's initial pose. BasicNavigator's default AMCL wait
        # can publish (0,0,0), so wait for lifecycle and the actual map TF instead.
        nav.get_logger().info('正在等待 Nav2 就绪，完成后会自动开始巡逻。')
        state_client=nav.create_client(GetState, '/bt_navigator/get_state')
        deadline=time.monotonic()+120
        while time.monotonic()<deadline:
            if not state_client.wait_for_service(timeout_sec=1.0):
                continue
            future=state_client.call_async(GetState.Request())
            rclpy.spin_until_future_complete(nav,future,timeout_sec=2.0)
            if future.done() and future.result() is not None and future.result().current_state.id == 3:
                break
            future.cancel()
            rclpy.spin_once(nav,timeout_sec=0.2)
        else:
            raise RuntimeError('Nav2 did not activate within 120 wall-clock seconds')
        nav.destroy_client(state_client)
        buffer=Buffer(); listener=TransformListener(buffer,nav)
        deadline=time.monotonic()+30
        while not buffer.can_transform('map','base_link',rclpy.time.Time()):
            if time.monotonic()>deadline:
                raise RuntimeError('No map -> base_link transform; check localization')
            rclpy.spin_once(nav,timeout_sec=0.2)
        nav.get_logger().info('导航已就绪，开始自动巡逻。')
        lap=0
        while rclpy.ok() and (args.loops == 0 or lap < args.loops):
            lap_start=nav.get_clock().now()
            for index,points in enumerate(groups):
                poses=[]
                for point in points:
                    pose=PoseStamped(); pose.header.frame_id=route.get('frame_id','map')
                    pose.header.stamp=nav.get_clock().now().to_msg()
                    pose.pose.position.x=float(point['x']); pose.pose.position.y=float(point['y'])
                    pose.pose.orientation.z=math.sin(float(point['yaw'])/2)
                    pose.pose.orientation.w=math.cos(float(point['yaw'])/2)
                    poses.append(pose)
                if mode=='through_poses':nav.goThroughPoses(poses)
                else:nav.goToPose(poses[0])
                start=nav.get_clock().now()
                timeout=route_timeout if mode=='through_poses' else args.timeout
                while not nav.isTaskComplete():
                    if (nav.get_clock().now()-start).nanoseconds/1e9 > timeout:
                        nav.cancelTask(); raise RuntimeError(f'{mode} task {index+1} timed out; patrol stopped')
                result=nav.getResult()
                entry={'lap':lap+1,'result':getattr(result,'name',str(result)),'sim_duration_s':(nav.get_clock().now()-start).nanoseconds/1e9}
                if mode=='through_poses':
                    entry.update(via_points=len(poses),segment=index+1);report['segments'].append(entry)
                else:
                    entry.update(index=index+1,goal=points[0]);report['waypoints'].append(entry)
                save_report()
                if result != TaskResult.SUCCEEDED:
                    raise RuntimeError(f'Waypoint {index+1} failed; patrol stopped')
                nav.get_logger().info(f'Lap {lap+1}: completed {mode} task {index+1}')
            lap+=1
            report['laps'].append(dict(lap=lap,result='SUCCEEDED',sim_duration_s=(nav.get_clock().now()-lap_start).nanoseconds/1e9))
            save_report()
        report['result']='success' if args.loops>0 and lap>=args.loops else 'stopped'
    except KeyboardInterrupt:
        report['result']='canceled'
        nav.cancelTask()
    except Exception as error:
        report['result']='failed';report['reason']=str(error)
        raise
    finally:
        save_report()
        nav.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__': main()
