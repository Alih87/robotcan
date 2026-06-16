from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    can_port_arg = DeclareLaunchArgument(
        "can_port",
        default_value="can1",
        description="CAN interface name: vcan0 for testing, can0 for real CAN"
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
        robotcan_srv_node,
        robotcan_gui_node,
    ])