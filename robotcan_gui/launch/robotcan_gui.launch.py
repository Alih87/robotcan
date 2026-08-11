#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    can_port_arg = DeclareLaunchArgument(
        "can_port",
        default_value="can0",
        description="CAN interface name: vcan0 for testing, can0 for real CAN"
    )
    
    robot_odom_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("robotcan"), "launch", "robot_odom.launch.py")
        )
    )

    robotcan_srv_node = Node(
        package="robotcan",
        executable="robot_srv",
        name="robotcan_srv",
        output="screen",
        parameters=[
            {
                "can_port": LaunchConfiguration("can_port"),
                "read_period_ms": 50,
                "reverse_delay_ms": 1500
            }
        ],
    )

    robotcan_gui_node = Node(
        package="robotcan_gui",
        executable="can_gui",
        name="robotcan_gui",
        output="screen",
    )

    return LaunchDescription([
        can_port_arg,
        robot_odom_launch,
        robotcan_srv_node,
        robotcan_gui_node,
    ])
