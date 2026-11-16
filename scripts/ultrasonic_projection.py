#!/usr/bin/python3
"""Convert standard sonar Range to conservative uncertainty arcs for Nav2."""
import rclpy
import copy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Range
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from ultrasonic_config import range_arc,finite_navigation_range


class Projection(Node):
    def __init__(self):
        super().__init__('ultrasonic_navigation_projection');self.subscriptions_=[]
        from sensor_msgs.msg import PointCloud2
        for i in range(1,5):
            obstacle=self.create_publisher(PointCloud2,f'/ultrasonic_{i}/obstacles',10)
            clearing=self.create_publisher(PointCloud2,f'/ultrasonic_{i}/clearing',10)
            navigation=self.create_publisher(Range,f'/ultrasonic_{i}/navigation_range',10)
            def receive(message,obstacle=obstacle,clearing=clearing,navigation=navigation):
                value=finite_navigation_range(message.range,message.min_range,message.max_range)
                if value is not None:
                    normalized=copy.deepcopy(message);normalized.range=value;navigation.publish(normalized)
                points,marking=range_arc(message.range,message.min_range,message.max_range,message.field_of_view)
                if not points:return
                cloud=create_cloud_xyz32(message.header,points)
                empty=create_cloud_xyz32(message.header,[])
                obstacle.publish(cloud if marking else empty)
                clearing.publish(empty if marking else cloud)
            self.subscriptions_.append(self.create_subscription(Range,f'/ultrasonic_{i}',receive,qos_profile_sensor_data))


def main():
    rclpy.init();node=Projection()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
