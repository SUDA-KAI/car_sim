"""Audit boundary between simulator state and sensor-only localization."""
SENSOR_MESSAGES={'rosgraph_msgs/msg/Clock','geometry_msgs/msg/Twist','sensor_msgs/msg/LaserScan',
                 'sensor_msgs/msg/PointCloud2','sensor_msgs/msg/Imu','sensor_msgs/msg/Image'}


def sensor_only_bridges(bridges):
    result=[]
    for bridge in bridges:
        if bridge['ros_type_name'] not in SENSOR_MESSAGES:continue
        bridge=dict(bridge)
        if bridge['ros_type_name']=='geometry_msgs/msg/Twist' and bridge.get('gz_topic_name')=='/sim/cmd_vel':
            bridge['ros_topic_name']='/localization/safe_cmd_vel'
        result.append(bridge)
    return result
