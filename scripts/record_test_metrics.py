#!/usr/bin/python3
"""Record physical ground truth independently from wheel odometry, CSV + JSON."""
import argparse,csv,json,math,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry,Path as NavPath
from geometry_msgs.msg import Twist
from narrow_geometry import center_offset,envelope_clearance

def angles(q):
    return (math.atan2(2*(q.w*q.x+q.y*q.z),1-2*(q.x*q.x+q.y*q.y)),
            math.asin(max(-1.,min(1.,2*(q.w*q.y-q.z*q.x)))),
            math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))

def corridor_clearance(y,yaw,width=1.7,length=1.2,body_width=.9,center_y=0.):
    lateral_half_extent=.5*(length*abs(math.sin(yaw))+body_width*abs(math.cos(yaw)))
    return width/2-abs(y-center_y)-lateral_half_extent

class Recorder(Node):
    def __init__(self,scene,output,localization="ground_truth",patrol_report=None):
        super().__init__('test_scene_metrics');self.scene=json.loads(scene.read_text());self.output=output;self.localization=localization
        self.patrol_report=patrol_report;self.finished=False;self.patrol_result=None
        output.parent.mkdir(parents=True,exist_ok=True);self.file=output.with_suffix('.csv').open('w')
        self.columns=['sim_time','x','y','z','roll_deg','pitch_deg','yaw','true_vx','true_vy','true_vz','true_wz','wheel_vx','command_vx','command_wz','wheel_minus_true_vx','wheel_pose_error_m','plan_distance_m','lateral_offset_m','estimated_side_clearance_m','wheel_sweep_clearance_bound_m']
        self.writer=csv.DictWriter(self.file,fieldnames=self.columns);self.writer.writeheader()
        self.wheel=None;self.command=Twist();self.plan=[];self.rows=0;self.last=None
        self.stats=dict(max_abs_pitch_deg=0.,max_abs_roll_deg=0.,max_height_m=0.,
                        wheel_speed_gap_rms=0.,max_abs_wheel_speed_gap=0.,max_wheel_pose_error_m=0.,
                        max_plan_distance_m=None,true_distance_m=0.,wheel_distance_m=0.)
        self.corridor_stats={}
        if 'clear_width_m' in self.scene:
            self.corridor_stats=dict(max_abs_lateral_offset_m=0.,min_estimated_side_clearance_m=None,potential_side_wall_overlap_samples=0,steering_reversal_count=0,min_wheel_sweep_clearance_bound_m=None,max_abs_lateral_offset_forward_settled_m=None)
        self.previous_steer_sign=0
        self.gap_sq=0.;self.start=None;self.end=None;self.last_flush=time.monotonic()
        self.subs=[self.create_subscription(Odometry,'/ground_truth/odom',self.truth,qos_profile_sensor_data),
                   self.create_subscription(Odometry,'/wheel/odom',lambda m:setattr(self,'wheel',m),qos_profile_sensor_data),
                   self.create_subscription(Twist,'/cmd_vel',lambda m:setattr(self,'command',m),10),
                   self.create_subscription(NavPath,'/plan',lambda m:setattr(self,'plan',[(p.pose.position.x,p.pose.position.y) for p in m.poses]),10)]
        self.timer=self.create_timer(5.,self.save)
        self.get_logger().info(f'Metrics CSV/JSON: {output}; truth localization is an ideal baseline, not slip compensation.')
    def truth(self,m):
        if self.wheel is None:return
        t=m.header.stamp.sec+m.header.stamp.nanosec/1e9
        if self.end is not None and t-self.end<.09:return
        p=m.pose.pose.position;roll,pitch,yaw=angles(m.pose.pose.orientation)
        w=self.wheel.pose.pose.position;sx,sy,sa=self.scene['spawn']
        wx=sx+math.cos(sa)*w.x-math.sin(sa)*w.y;wy=sy+math.sin(sa)*w.x+math.cos(sa)*w.y
        gap=self.wheel.twist.twist.linear.x-m.twist.twist.linear.x
        row=dict(sim_time=t,x=p.x,y=p.y,z=p.z,roll_deg=math.degrees(roll),pitch_deg=math.degrees(pitch),yaw=yaw,
                 true_vx=m.twist.twist.linear.x,true_vy=m.twist.twist.linear.y,true_vz=m.twist.twist.linear.z,true_wz=m.twist.twist.angular.z,
                 wheel_vx=self.wheel.twist.twist.linear.x,command_vx=self.command.linear.x,command_wz=self.command.angular.z,
                 wheel_minus_true_vx=gap,wheel_pose_error_m=math.hypot(wx-p.x,wy-p.y),plan_distance_m=nearest_path_distance(p.x,p.y,self.plan))
        row['lateral_offset_m']=None;row['estimated_side_clearance_m']=None;row['wheel_sweep_clearance_bound_m']=None
        if self.corridor_stats:
            lateral=center_offset(self.scene,p.x,p.y)
            quaternion=m.pose.pose.orientation
            q=(quaternion.x,quaternion.y,quaternion.z,quaternion.w)
            clearance=envelope_clearance(self.scene,p.x,p.y,q)
            row.update(lateral_offset_m=lateral,estimated_side_clearance_m=clearance)
            c=self.corridor_stats;c['max_abs_lateral_offset_m']=max(c['max_abs_lateral_offset_m'],abs(lateral))
            c['min_estimated_side_clearance_m']=clearance if c['min_estimated_side_clearance_m'] is None else min(c['min_estimated_side_clearance_m'],clearance)
            if clearance<=0:c['potential_side_wall_overlap_samples']+=1
            if self.scene.get('wheel_sweep_width_bound_m'):
                bound=envelope_clearance(self.scene,p.x,p.y,q,self.scene['wheel_sweep_width_bound_m'])
                row['wheel_sweep_clearance_bound_m']=bound
                c['min_wheel_sweep_clearance_bound_m']=bound if c['min_wheel_sweep_clearance_bound_m'] is None else min(c['min_wheel_sweep_clearance_bound_m'],bound)
            if self.scene.get('reference_geometry',{}).get('type','straight')=='straight' and p.x>=self.scene['spawn'][0]+2 and p.x<=6 and m.twist.twist.linear.x>.05:
                c['max_abs_lateral_offset_forward_settled_m']=max(c['max_abs_lateral_offset_forward_settled_m'] or 0.,abs(lateral))
            command_sign=1 if self.command.angular.z>.005 else (-1 if self.command.angular.z<-.005 else 0)
            if command_sign and self.previous_steer_sign and command_sign!=self.previous_steer_sign:c['steering_reversal_count']+=1
            if command_sign:self.previous_steer_sign=command_sign
        self.writer.writerow(row);self.rows+=1;self.gap_sq+=gap*gap
        stats=self.stats
        for key,value in [('max_abs_pitch_deg',abs(row['pitch_deg'])),('max_abs_roll_deg',abs(row['roll_deg'])),
                          ('max_height_m',p.z),('max_abs_wheel_speed_gap',abs(gap)),('max_wheel_pose_error_m',row['wheel_pose_error_m'])]:stats[key]=max(stats[key],value)
        if row['plan_distance_m'] is not None:stats['max_plan_distance_m']=max(stats['max_plan_distance_m'] or 0.,row['plan_distance_m'])
        if self.last is not None:
            last_t,last_x,last_y,last_z=self.last;dt=t-last_t
            if 0<dt<1:
                stats['true_distance_m']+=math.sqrt((p.x-last_x)**2+(p.y-last_y)**2+(p.z-last_z)**2)
                stats['wheel_distance_m']+=abs(self.wheel.twist.twist.linear.x)*dt
        self.last=(t,p.x,p.y,p.z);self.end=t
        if self.start is None:self.start=t
        if time.monotonic()-self.last_flush>2:self.file.flush();self.last_flush=time.monotonic()
    def save(self):
        if self.patrol_report and self.patrol_report.exists():
            try:
                self.patrol_result=json.loads(self.patrol_report.read_text()).get('result')
                if self.patrol_result not in (None,'running'):self.finished=True
            except (OSError,ValueError):pass
        self.file.flush();self.stats['wheel_speed_gap_rms']=math.sqrt(self.gap_sq/max(1,self.rows))
        out=dict(scene=self.scene['name'],samples=self.rows,sim_duration_s=0. if self.start is None else self.end-self.start,
                 localization=self.localization,patrol_result=self.patrol_result,**self.stats,**self.corridor_stats,
                 note='Wheel/true speed difference is a slip indicator, not a calibrated tyre slip ratio. No pass/fail thresholds assumed.')
        self.output.with_suffix('.json').write_text(json.dumps(out,indent=2))

def nearest_path_distance(x,y,path):
    if not path:return None
    if len(path)==1:return math.hypot(x-path[0][0],y-path[0][1])
    best=float('inf')
    for (ax,ay),(bx,by) in zip(path,path[1:]):
        dx,dy=bx-ax,by-ay;d=dx*dx+dy*dy
        u=0. if d==0 else max(0.,min(1.,((x-ax)*dx+(y-ay)*dy)/d))
        best=min(best,math.hypot(x-ax-u*dx,y-ay-u*dy))
    return best

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--scene',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--localization',default='ground_truth');p.add_argument('--patrol-report',type=Path);a=p.parse_args()
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO);node=Recorder(a.scene,a.output,a.localization,a.patrol_report)
    try:
        while rclpy.ok() and not node.finished:rclpy.spin_once(node,timeout_sec=.5)
    except KeyboardInterrupt:pass
    finally:
        node.save();node.file.close();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
