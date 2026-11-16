#!/usr/bin/python3
"""Planar L2 scan odometry with measured gyro rotation; no simulator pose."""
from collections import deque
import json,math,time
import numpy as np
from scipy.spatial import cKDTree


def register_translation(reference,current,rotation,seed):
    tree=cKDTree(reference)
    _,indices=tree.query(reference,k=7)
    neighbors=reference[indices];centered=neighbors-neighbors.mean(axis=1,keepdims=True)
    covariance=np.einsum('nki,nkj->nij',centered,centered)
    values,vectors=np.linalg.eigh(covariance);normals=vectors[:,:,0]
    planar=values[:,0]/np.maximum(values[:,1],1e-9)<.12
    rotated=current@rotation.T;translation=np.asarray(seed,dtype=float).copy()
    for _ in range(15):
        distance,match=tree.query(rotated+translation)
        mask=(distance<.4)&planar[match]
        if mask.sum()<40:raise ValueError('Insufficient scan correspondences')
        n=normals[match[mask]];q=reference[match[mask]];p=rotated[mask]
        residual=np.sum(n*(p+translation-q),axis=1)
        weights=np.minimum(1.,.03/np.maximum(np.abs(residual),1e-8))
        h=n.T@(weights[:,None]*n);rhs=np.einsum('ni,ni->n',n,q-p)
        eigen=np.linalg.eigvalsh(h)
        if eigen[0]<.03:raise ValueError('Scan translation is unobservable')
        updated=np.linalg.solve(h+.01*np.eye(2),n.T@(weights*rhs)+.01*np.asarray(seed))
        if np.linalg.norm(updated-translation)<1e-5:translation=updated;break
        translation=updated
    residual=np.sum(n*(p+translation-q),axis=1)
    inliers=np.abs(residual)<.08
    if inliers.sum()<40:raise ValueError('Insufficient scan inliers')
    rmse=float(np.sqrt(np.mean(residual[inliers]**2)))
    if rmse>.05:raise ValueError('High scan residual')
    return translation,dict(inliers=int(inliers.sum()),rmse_m=rmse,condition=float(eigen[-1]/eigen[0]))


def gyro_integral(samples,start,end):
    if len(samples)<2 or samples[0][0]>start+.03 or samples[-1][0]<end-.03:raise ValueError('IMU history does not cover scan interval')
    stamps=np.array([v[0] for v in samples]);rates=np.array([v[1] for v in samples])
    inside=(stamps>start)&(stamps<end)
    t=np.r_[start,stamps[inside],end];w=np.interp(t,stamps,rates)
    return float(np.trapz(w,t))


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan,Imu
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    from tf2_ros import Buffer,TransformListener
    from scipy.spatial.transform import Rotation
    class Lidar(Node):
        def __init__(self):
            super().__init__('sim_planar_lidar_odometry');self.buffer=Buffer();self.listener=TransformListener(self.buffer,self)
            self.gyro=deque(maxlen=2000);self.previous=None;self.position=np.zeros(2);self.yaw=0.;self.velocity=np.zeros(2);self.visual=None;self.accepted=0;self.rejected=0
            self.pub=self.create_publisher(Odometry,'/localization/lidar_odom',10)
            self.status=self.create_publisher(String,'/localization/lio_status',10)
            self.create_subscription(Imu,'/imu/data_raw',self.on_imu,qos_profile_sensor_data)
            self.create_subscription(LaserScan,'/scan',self.on_scan,qos_profile_sensor_data)
            self.create_subscription(Odometry,'/localization/visual_odom',self.on_visual,qos_profile_sensor_data)
        def on_visual(self,msg):self.visual=msg
        def on_imu(self,msg):
            stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
            if self.gyro and stamp<=self.gyro[-1][0]:return
            try:
                t=self.buffer.lookup_transform('base_link',msg.header.frame_id,rclpy.time.Time()).transform;q=t.rotation
                angular=Rotation.from_quat([q.x,q.y,q.z,q.w]).apply([msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z])
            except Exception:return
            if np.isfinite(angular).all():self.gyro.append((stamp,float(angular[2])))
        def on_scan(self,msg):
            started=time.monotonic();stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
            now=self.get_clock().now().nanoseconds*1e-9
            if not -.05<=now-stamp<=.75:return
            try:
                ranges=np.array(msg.ranges);angles=msg.angle_min+np.arange(len(ranges))*msg.angle_increment
                valid=np.isfinite(ranges)&(ranges>.2)&(ranges<min(15.,msg.range_max))
                if valid.sum()<80:raise ValueError('Insufficient laser returns')
                points=np.stack([ranges[valid]*np.cos(angles[valid]),ranges[valid]*np.sin(angles[valid]),np.zeros(valid.sum())],axis=1)
                t=self.buffer.lookup_transform('base_link',msg.header.frame_id,rclpy.time.Time()).transform;q=t.rotation
                points=(Rotation.from_quat([q.x,q.y,q.z,q.w]).apply(points)+[t.translation.x,t.translation.y,t.translation.z])[:,:2]
                quality={}
                if self.previous:
                    old_stamp,old_points=self.previous;dt=stamp-old_stamp
                    if dt<=0:return
                    if dt>1.5:raise ValueError('Scan interval too long')
                    delta=gyro_integral(list(self.gyro),old_stamp,stamp)
                    c,s=math.cos(delta),math.sin(delta);rotation=np.array([[c,-s],[s,c]])
                    c,s=math.cos(self.yaw),math.sin(self.yaw);world_rotation=np.array([[c,-s],[s,c]])
                    seed=world_rotation.T@self.velocity*dt
                    if self.visual:
                        vs=self.visual.header.stamp.sec+self.visual.header.stamp.nanosec*1e-9
                        if abs(stamp-vs)<.5:
                            seed=np.array([self.visual.twist.twist.linear.x,self.visual.twist.twist.linear.y])*dt
                    translation,quality=register_translation(old_points,points,rotation,seed)
                    if np.linalg.norm(translation)>.6*dt+.03 or abs(delta)>.7*dt+.03:raise ValueError('Implausible measured scan motion')
                    self.position+=world_rotation@translation;self.velocity=world_rotation@translation/dt;self.yaw+=delta
                elif not self.gyro:return
                self.previous=(stamp,points);self.accepted+=1
                out=Odometry();out.header.stamp=msg.header.stamp;out.header.frame_id='odom';out.child_frame_id='base_link'
                out.pose.pose.position.x=float(self.position[0]);out.pose.pose.position.y=float(self.position[1]);out.pose.pose.orientation.z=math.sin(self.yaw/2);out.pose.pose.orientation.w=math.cos(self.yaw/2)
                c,s=math.cos(self.yaw),math.sin(self.yaw);velocity=np.array([[c,s],[-s,c]])@self.velocity
                out.twist.twist.linear.x=float(velocity[0]);out.twist.twist.linear.y=float(velocity[1])
                out.pose.covariance=[0.]*36;out.twist.covariance=[0.]*36
                for i in range(6):out.pose.covariance[i*7]=.03**2 if i<2 else .05**2;out.twist.covariance[i*7]=.1**2
                self.pub.publish(out)
                status=dict(state='ready',algorithm='planar point-to-line scan matching + measured IMU gyro',received=self.accepted,rejected=self.rejected,source_age_s=now-stamp,processing_ms=(time.monotonic()-started)*1000,**quality)
            except Exception as e:
                self.rejected+=1;status=dict(state='scan_matching_lost',reason=str(e),received=self.accepted,rejected=self.rejected)
            self.status.publish(String(data=json.dumps(status)))
    rclpy.init();node=Lidar()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
