from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os
import sys

def generate_launch_description():
    if not os.environ.get("G1_INTERFACE"):
        sys.exit("ERROR: G1_INTERFACE environment variable is not set.\n"
                 "Set it to your network interface, e.g.: export G1_INTERFACE=eno2")
    interface = LaunchConfiguration("interface")
    use_robot = LaunchConfiguration("use_robot")
    use_sim_time = LaunchConfiguration("use_sim_time")
    arm_controlled = LaunchConfiguration("arm_controlled")
    enable_arm_ui = LaunchConfiguration("enable_arm_ui")

    return LaunchDescription([
        DeclareLaunchArgument("interface", default_value=EnvironmentVariable("G1_INTERFACE")),
        DeclareLaunchArgument("use_robot", default_value="true"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("arm_controlled", default_value="both"),
        DeclareLaunchArgument("enable_arm_ui", default_value="true"),

        Node(
            package='g1pilot',
            executable='loco_client',
            name='loco_client',
            parameters=[{
                'interface': interface,
                'use_robot': ParameterValue(use_robot, value_type=bool),
                'arm_controlled': arm_controlled,  # string ('left'|'right'|'both')
                'enable_arm_ui': ParameterValue(enable_arm_ui, value_type=bool),
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            }],
            output='screen'
        ),

        Node(
            package='g1pilot',
            executable='nav2point',
            name='nav2point',
            parameters=[{
                'interface': interface,
                'use_robot': ParameterValue(use_robot, value_type=bool),
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            }],
            remappings=[
                ('/lidar_odometry/pose_fixed', '/lidar_odometry/pose'),
            ],
            output='screen'
        ),
    ])
