from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ROOT = Path(__file__).resolve().parents[2]


def start(context):
    world = str(Path(LaunchConfiguration("world").perform(context)).resolve())
    verbosity = LaunchConfiguration("verbosity").perform(context)
    return [
        ExecuteProcess(
            cmd=["ign", "gazebo", "-r", "-s", "--headless-rendering", "-v", verbosity, world],
            output="screen", on_exit=Shutdown()),
        Node(package="ros_gz_bridge", executable="parameter_bridge",
             parameters=[{"config_file": str(ROOT / "config/bridge.yaml"),
                          "use_sim_time": True}], output="screen"),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("world", default_value=str(
            ROOT / "worlds/map_20260817_154310_filtered.sdf")),
        DeclareLaunchArgument("verbosity", default_value="2"),
        OpaqueFunction(function=start),
    ])
