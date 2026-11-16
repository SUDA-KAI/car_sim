#!/usr/bin/python3
"""Bounded Nav2 goal using estimated TF only; truth is read privately for evaluation."""
import copy
import json
import math
import os
from pathlib import Path
import re
import subprocess
import time
import rclpy
from rclpy.parameter import Parameter
from nav2_simple_commander.robot_navigator import BasicNavigator,TaskResult
from nav_msgs.msg import Odometry,OccupancyGrid
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import Buffer,TransformListener

ROOT=Path(__file__).resolve().parents[1]


def native_truth():
    # This value is never published to ROS or used to construct the goal.
    env=dict(os.environ,IGN_PARTITION='patrol_sensors')
    result=subprocess.run(['ign','topic','-e','-t','/sim/ground_truth','-n','1'],env=env,capture_output=True,text=True,timeout=10,check=True)
    position=re.search(r'pose\s*\{\s*position\s*\{([^}]*)\}',result.stdout,re.S)
    if not position:raise RuntimeError('Cannot read independent Gazebo evaluation pose')
    return {key:float(re.search(r'\b'+key+r':\s*([^\s]+)',position.group(1)).group(1)) for key in ('x','y','z')}


def main():
    if os.environ.get('ROS_DOMAIN_ID')!='43':raise RuntimeError('This test only runs in isolated sensor domain 43')
    audit=json.loads((ROOT/'artifacts/sensor_localization_current.json').read_text())
    assert audit['mode']=='sensors'
    assert all(item['ros_type_name'] not in ('nav_msgs/msg/Odometry','tf2_msgs/msg/TFMessage') for item in audit['bridge'])
    rclpy.init();nav=BasicNavigator(node_name='sensor_navigation_validation');nav.set_parameters([Parameter('use_sim_time',value=True)])
    latest={};counts={};measurements=[];statuses=[];subs=[]
    def on_odom(source,msg):
        latest[source]=msg;counts[source]=counts.get(source,0)+1
        if len(measurements)<20000:
            p=msg.pose.pose.position
            measurements.append(dict(source=source,stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,x=p.x,y=p.y,vx=msg.twist.twist.linear.x))
    for source,topic in [('fused','/chassis/odom'),('lio','/localization/lidar_odom'),('visual','/localization/visual_odom')]:
        subs.append(nav.create_subscription(Odometry,topic,lambda msg,s=source:on_odom(s,msg),qos_profile_sensor_data))
    def on_status(msg):
        value=json.loads(msg.data);latest['visual_status']=value
        if len(statuses)<2000:statuses.append(value)
    subs.append(nav.create_subscription(String,'/localization/visual_status',on_status,10))
    subs.append(nav.create_subscription(String,'/localization/gate_status',lambda msg:latest.update(gate_status=json.loads(msg.data)),10))
    subs.append(nav.create_subscription(OccupancyGrid,'/map',lambda m:latest.update(map=m),qos_profile_sensor_data))
    buffer=Buffer();listener=TransformListener(buffer,nav)
    report=dict(domain=43,partition='patrol_sensors',passed=False,goal_source='estimated map -> base_link TF plus 1.2 m forward; no truth seed',audit=audit)
    sent=False
    try:
        deadline=time.monotonic()+90
        while time.monotonic()<deadline:
            rclpy.spin_once(nav,timeout_sec=.1)
            if all(k in latest for k in ('fused','lio','visual','map')) and latest.get('visual_status',{}).get('state')=='tracking' and latest.get('gate_status',{}).get('state')=='enabled' and buffer.can_transform('map','base_link',rclpy.time.Time()):break
        else:raise RuntimeError('Sensor localization did not become ready')
        forbidden={topic:len(nav.get_publishers_info_by_topic(topic)) for topic in ('/ground_truth/odom','/wheel/odom')}
        assert not any(forbidden.values()),forbidden
        publishers=[info.node_name for info in nav.get_publishers_info_by_topic('/chassis/odom')]
        assert publishers==['ekf_filter_node'],publishers
        visual_consumers=[info.node_name for info in nav.get_subscriptions_info_by_topic('/localization/visual_odom')]
        assert 'ekf_filter_node' in visual_consumers
        safe_publishers=[info.node_name for info in nav.get_publishers_info_by_topic('/localization/safe_cmd_vel')]
        assert safe_publishers==['sim_sensor_command_gate'],safe_publishers
        assert not any(item['ros_topic_name']=='/cmd_vel' for item in audit['bridge'])
        report['command_gate_publishers']=safe_publishers
        initial={key:copy.deepcopy(latest[key]) for key in ('fused','lio','visual')}
        # Independent evaluation is sampled after the navigation source has
        # already initialized; it cannot seed LIO, EKF, SLAM or the goal.
        report['evaluation_true_start']=native_truth()
        transform=buffer.lookup_transform('map','base_link',rclpy.time.Time()).transform
        q=transform.rotation;yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        target=PoseStamped();target.header.frame_id='map';target.header.stamp=nav.get_clock().now().to_msg()
        target.pose.position.x=transform.translation.x+1.2*math.cos(yaw)
        target.pose.position.y=transform.translation.y+1.2*math.sin(yaw)
        target.pose.orientation.z=math.sin(yaw/2);target.pose.orientation.w=math.cos(yaw/2)
        report['goal']=dict(x=target.pose.position.x,y=target.pose.position.y,yaw=yaw)
        print('Estimated-frame goal',report['goal'],flush=True)
        start=time.monotonic();nav.goToPose(target);sent=True;deadline=start+150
        while not nav.isTaskComplete():
            if time.monotonic()>deadline:nav.cancelTask();raise RuntimeError('Bounded sensor navigation timed out')
        result=nav.getResult();report['nav_result']=getattr(result,'name',str(result));report['wall_duration_s']=time.monotonic()-start
        for _ in range(10):rclpy.spin_once(nav,timeout_sec=.1)
        report['evaluation_true_end']=native_truth()
        report['odom_displacements']={key:dict(dx=latest[key].pose.pose.position.x-initial[key].pose.pose.position.x,
            dy=latest[key].pose.pose.position.y-initial[key].pose.pose.position.y) for key in initial}
        delta={key:report['evaluation_true_end'][key]-report['evaluation_true_start'][key] for key in ('x','y')}
        report['evaluation_true_delta']=delta;report['true_distance_m']=math.hypot(delta['x'],delta['y'])
        report['fused_displacement_error_m']=math.hypot(report['odom_displacements']['fused']['dx']-delta['x'],report['odom_displacements']['fused']['dy']-delta['y'])
        report['message_counts']=counts;report['visual_status_final']=latest['visual_status'];report['truth_topic_publishers']=forbidden
        report['fused_publishers']=publishers;report['visual_consumers']=visual_consumers
        moved_visual=[m for m in measurements if m['source']=='visual' and abs(m['vx'])>.03]
        report['moving_visual_measurements']=len(moved_visual)
        report['checks']=dict(nav_succeeded=result==TaskResult.SUCCEEDED,true_motion=report['true_distance_m']>.7,
            fusion_position_error_under_20cm=report['fused_displacement_error_m']<.2,
            visual_motion_used=len(moved_visual)>3,truth_not_bridged=not any(forbidden.values()),ekf_owns_odometry=publishers==['ekf_filter_node'])
        report['checks']['forward_motion']=delta['x']>.7
        report['checks']['lio_motion_used']=report['odom_displacements']['lio']['dx']>.5
        report['checks']['motion_bridge_is_gated']=safe_publishers==['sim_sensor_command_gate']
        report['passed']=all(report['checks'].values())
        print(json.dumps({key:value for key,value in report.items() if key!='audit'},ensure_ascii=False,indent=2),flush=True)
    except Exception as error:
        report['error']=str(error);print('Validation failed:',error,flush=True)
    finally:
        if sent and not report.get('nav_result'):nav.cancelTask()
        report['measurements']=measurements;report['visual_status_history']=statuses
        (ROOT/'artifacts/sensor_navigation_live_validation.json').write_text(json.dumps(report,indent=2)+'\n')
        nav.destroy_node();rclpy.shutdown()
    if not report['passed']:raise SystemExit(1)

if __name__=='__main__':main()
