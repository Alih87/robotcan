#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    can_port_arg = DeclareLaunchArgument(
        "can_port",
        default_value="vcan0",
        description="CAN interface name: vcan0 for testing, can0 for real CAN"
    )

    robot_twist_node = Node(
        package="robotcan",
        executable="robot_pub",
        name="robotcan_pub",
        output="screen",
        parameters=[
            {
                "can_port": LaunchConfiguration("can_port"),
                "publish_rate_hz": 20.,
                "steering_gain": 0.02
            }
        ],
    )

    robot_odom_node = Node(
        package="robotcan",
        executable="robot_odom",
        name="robotcan_odom",
        output="screen",
        parameters=[
            {
                "publish_rate_hz": 20.,
                "tau_accel": 2.5,
                "tau_decel": 1.5,
                "velocity_scale": 1.0,
                "deadband": 0.0,
                "use_imu_yaw_rate": False,
                "odom_frame": "odom",
                "base_frame": "base_link"
            }
        ],
    )

    return LaunchDescription([
        can_port_arg,
        robot_twist_node,
        robot_odom_node
    ])