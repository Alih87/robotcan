#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

ROBOTCAN_ODOM_PARAMS = os.path.join(get_package_share_directory("robotcan"), "params", "robotcan_odom_params.yaml")
ROBOTCAN_GPS_PARAMS = os.path.join(get_package_share_directory("robotcan"), "params", "zed_f9p.yaml")

def generate_launch_description():
    robot_pub_node = Node(
        package="robotcan",
        executable="robot_pub",
        name="robotcan_pub_node",
        output="screen",
        parameters=[{"robotcan_pub_node":""}, ROBOTCAN_ODOM_PARAMS]
    )

    robot_odom_node = Node(
        package="robotcan",
        executable="robot_odom",
        name="robotcan_odom_node",
        output="screen",
        parameters=[{"robotcan_odom_node":""}, ROBOTCAN_ODOM_PARAMS],
    )

    ublox_gps_node = Node(
        package="ublox_gps",
        executable="ublox_gps_node",
        name="robot_gps_node",
        output="screen",
        parameters=[ROBOTCAN_GPS_PARAMS]
    )

    robot_gps_can_node = Node(
            package="robotcan",
            executable="gps_can_tx",
            name="gps_can_tx_node",
            output="screen",
            )

    return LaunchDescription([
        robot_pub_node,
        robot_odom_node,
        ublox_gps_node,
        robot_gps_can_node
    ])