#!/usr/bin/python3
"""Simulation-only sensor dashboard and advisory proximity alarm. No cmd_vel."""
import argparse
from collections import OrderedDict
import functools
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import os
from pathlib import Path
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Range, LaserScan, PointCloud2, Image
from nav_msgs.msg import OccupancyGrid,Odometry,Path as RosPath
from std_msgs.msg import Bool, String
from l2_sim import read_xyz
from proximity_alarm import Proximity

WEB = Path(__file__).resolve().parents[1]/'sensor_viewer/index.html'


class Monitor(Node):
    def __init__(self, threshold):
        super().__init__('sim_sensor_monitor')
        self.lock = threading.RLock()
        self.proximity = Proximity(threshold, threshold+.02)
        self.views = {}
        self.last_cloud = {}
        self.map_points = OrderedDict()
        self.map_frame = None
        self.map_stamp = None
        self.previous = None
        self.images={};self.localization={};self.odometry={}
        self.alarm_pub = self.create_publisher(Bool, '/sim/proximity_alarm', 10)
        self.status_pub = self.create_publisher(String, '/sim/proximity_status', 10)
        self.subs = []
        for channel in range(1, 5):
            self.subs.append(self.create_subscription(Range, f'/ultrasonic_{channel}',
                functools.partial(self.on_range, channel), qos_profile_sensor_data))
        self.subs.append(self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data))
        for topic in ('/unilidar/cloud', '/pointlio/cloud_registered', '/pointlio/local_map'):
            self.subs.append(self.create_subscription(PointCloud2, topic,
                functools.partial(self.on_cloud, topic), qos_profile_sensor_data))
        self.subs.append(self.create_subscription(OccupancyGrid,'/map',self.on_map,qos_profile_sensor_data))
        for side in ('left','right'):
            self.subs.append(self.create_subscription(Image,f'/camera_{side}/image_rect',functools.partial(self.on_image,side),qos_profile_sensor_data))
        self.subs.append(self.create_subscription(RosPath,'/localization/patrol_path',self.on_patrol_path,10))
        for source in ('lio','visual','gate','patrol'):
            self.subs.append(self.create_subscription(String,f'/localization/{source}_status',functools.partial(self.on_localization,source),10))
        for source,topic in [('lio','/localization/lidar_odom'),('visual','/localization/visual_odom'),('fused','/chassis/odom')]:
            self.subs.append(self.create_subscription(Odometry,topic,functools.partial(self.on_odometry,source),qos_profile_sensor_data))
        self.create_timer(.5, self.publish_status, clock=rclpy.clock.Clock())

    def on_patrol_path(self,msg):
        self.store('/localization/patrol_path',msg.header.frame_id,[[p.pose.position.x,p.pose.position.y,0.] for p in msg.poses])

    def on_range(self, channel, msg):
        stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        age = self.get_clock().now().nanoseconds*1e-9-stamp
        if age > self.proximity.timeout or age < -.1:
            return  # Delayed/replayed messages must not clear a previous alarm.
        with self.lock:
            self.proximity.update(channel, msg.range, msg.min_range, msg.max_range,
                                  time.monotonic(), stamp)

    def on_map(self,msg):
        grid=np.asarray(msg.data).reshape(msg.info.height,msg.info.width)
        rows,cols=np.where(grid>=65);stride=max(1,math.ceil(len(rows)/4000))
        rows,cols=rows[::stride],cols[::stride]
        q=msg.info.origin.orientation;yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        x=(cols+.5)*msg.info.resolution;y=(rows+.5)*msg.info.resolution
        points=np.stack([msg.info.origin.position.x+math.cos(yaw)*x-math.sin(yaw)*y,
            msg.info.origin.position.y+math.sin(yaw)*x+math.cos(yaw)*y,np.zeros(len(rows))],axis=1)
        self.store('/map',msg.header.frame_id,np.round(points,3).tolist())

    def on_image(self,side,msg):
        now=time.monotonic()
        with self.lock:
            previous=self.images.get(side)
            if previous and now-previous['received']<1.:return
        import cv2
        from stereo_sim import image_array
        try:
            pixels=image_array(msg)
            if pixels.shape[1]>640:pixels=cv2.resize(pixels,(640,round(640*pixels.shape[0]/pixels.shape[1])))
            ok,jpeg=cv2.imencode('.jpg',pixels,[cv2.IMWRITE_JPEG_QUALITY,75])
            if ok:
                with self.lock:self.images[side]=dict(received=now,jpeg=jpeg.tobytes(),width=msg.width,height=msg.height)
        except (ValueError,cv2.error):pass

    def on_localization(self,source,msg):
        try:status=json.loads(msg.data)
        except (ValueError,TypeError):return
        with self.lock:self.localization[source]=dict(status=status,received=time.monotonic())

    def on_odometry(self,source,msg):
        q=msg.pose.pose.orientation
        with self.lock:self.odometry[source]=dict(x=msg.pose.pose.position.x,y=msg.pose.pose.position.y,
            yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)),vx=msg.twist.twist.linear.x,
            frame=msg.header.frame_id,received=time.monotonic())

    def store(self, topic, frame, points):
        with self.lock:
            self.views[topic] = dict(frame=frame, points=points, received=time.monotonic())

    def on_scan(self, msg):
        points = []
        for i, radius in enumerate(msg.ranges):
            if math.isfinite(radius) and msg.range_min <= radius <= msg.range_max:
                angle = msg.angle_min+i*msg.angle_increment
                points.append([radius*math.cos(angle), radius*math.sin(angle), 0])
        self.store('/scan', msg.header.frame_id, points)

    def on_cloud(self, topic, msg):
        now = time.monotonic()
        if now-self.last_cloud.get(topic, -math.inf) < 1.:
            return
        self.last_cloud[topic] = now
        try:
            xyz = read_xyz(msg)
            # Bound CPU, HTTP payload and canvas rendering independently of map size.
            xyz = xyz[::max(1, math.ceil(len(xyz)/4000))]
            xyz = xyz[np.isfinite(xyz).all(axis=1)]
            self.store(topic, msg.header.frame_id, np.round(xyz, 3).tolist())
            if topic == '/pointlio/cloud_registered':
                stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
                if self.map_frame != msg.header.frame_id or (self.map_stamp is not None and stamp < self.map_stamp):
                    self.map_points.clear()
                self.map_frame, self.map_stamp = msg.header.frame_id, stamp
                for point in xyz:
                    key = tuple(np.floor(point/.1).astype(int))
                    self.map_points[key] = np.round(point, 3).tolist()
                    self.map_points.move_to_end(key)
                while len(self.map_points) > 12000:
                    self.map_points.popitem(last=False)
                points = list(self.map_points.values())
                self.store('/viewer/accumulated_map', self.map_frame,
                           points[::max(1, math.ceil(len(points)/4000))])
        except (ValueError, TypeError) as error:
            self.get_logger().warning(f'{topic}: {error}')

    def snapshot(self):
        now = time.monotonic()
        with self.lock:
            status = self.proximity.snapshot(now, self.get_clock().now().nanoseconds*1e-9)
            status['views'] = {key: dict(frame=value['frame'], points=value['points'],
                age=now-value['received']) for key, value in self.views.items()}
            status['domain'] = os.environ.get('ROS_DOMAIN_ID')
            status['images']={side:dict(age=now-value['received'],width=value['width'],height=value['height']) for side,value in self.images.items()}
            status['localization']={key:dict(status=value['status'],age=now-value['received']) for key,value in self.localization.items()}
            status['odometry']={key:dict({k:v for k,v in value.items() if k!='received'},age=now-value['received']) for key,value in self.odometry.items()}
            return status

    def publish_status(self):
        status = self.snapshot()
        self.alarm_pub.publish(Bool(data=status['alarm']))
        self.status_pub.publish(String(data=json.dumps({k:status[k] for k in ('threshold','release','alarm','unknown','channels','domain')}, allow_nan=False)))
        key = status['alarm'], status['unknown']
        if key != self.previous:
            if status['alarm']:
                self.get_logger().warning('超声波近障警报：距离 <= %.2f m；仅提示，不自动停车。' % self.proximity.threshold)
            elif status['unknown']:
                self.get_logger().warning('超声波数据缺失、无效或过期，请检查仿真是否运行。')
            else:
                self.get_logger().info('超声波数据恢复，近障警报已解除。')
            self.previous = key


class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        path = self.path.split('?')[0]
        if path == '/api/state':
            body = json.dumps(self.server.monitor.snapshot(), allow_nan=False).encode()
            content_type = 'application/json'
        elif path == '/':
            body = WEB.read_bytes(); content_type = 'text/html; charset=utf-8'
        elif path in ('/api/image/left','/api/image/right'):
            with self.server.monitor.lock:frame=self.server.monitor.images.get(path.rsplit('/',1)[1])
            if frame is None:self.send_error(404);return
            body=frame['jpeg'];content_type='image/jpeg'
        else:
            self.send_error(404); return
        self.send_response(200)
        self.send_header('Content-Type', content_type)
        self.send_header('Cache-Control', 'no-store')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers(); self.wfile.write(body)

    def log_message(self, *_):
        pass


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=8768)
    parser.add_argument('--threshold', type=float, default=.1)
    args, ros_args = parser.parse_known_args()
    # Physical robot traffic must never be used by this simulation dashboard.
    if os.environ.get('ROS_DOMAIN_ID', '0') not in ('42','43'):
        parser.error('This monitor requires simulation ROS_DOMAIN_ID=42 or sensor simulation domain 43')
    rclpy.init(args=ros_args)
    node = Monitor(args.threshold)
    server = None
    try:
        server = ThreadingHTTPServer(('127.0.0.1', args.port), Handler)
        server.monitor = node
        threading.Thread(target=server.serve_forever, daemon=True).start()
        node.get_logger().info(f'Sensor viewer: http://127.0.0.1:{args.port} (simulation domain {os.environ.get("ROS_DOMAIN_ID")})')
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if server:
            server.shutdown(); server.server_close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
