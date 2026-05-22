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
    ik_use_waist = LaunchConfiguration("ik_use_waist")
    ik_alpha = LaunchConfiguration("ik_alpha")
    ik_max_dq_step = LaunchConfiguration("ik_max_dq_step")
    arm_velocity_limit = LaunchConfiguration("arm_velocity_limit")
    enable_collision_avoidance = LaunchConfiguration("enable_collision_avoidance")
    send_cmds_to_robot = LaunchConfiguration("send_cmds_to_robot")
    publish_joint_states_opensot = LaunchConfiguration("publish_joint_states_opensot")

    return LaunchDescription([
        DeclareLaunchArgument("interface", default_value=EnvironmentVariable("G1_INTERFACE")),
        DeclareLaunchArgument("use_robot", default_value="true"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("arm_controlled", default_value="both"),
        DeclareLaunchArgument("enable_arm_ui", default_value="true"),
        DeclareLaunchArgument("ik_use_waist", default_value="false"),
        DeclareLaunchArgument("ik_alpha", default_value="0.2"),
        DeclareLaunchArgument("ik_max_dq_step", default_value="0.05"),
        DeclareLaunchArgument("arm_velocity_limit", default_value="2.0"),
        DeclareLaunchArgument("enable_collision_avoidance", default_value="false"),
        DeclareLaunchArgument("send_cmds_to_robot", default_value="true"),
        DeclareLaunchArgument("publish_joint_states_opensot", default_value="false"),

        Node(
            package='g1pilot',
            executable='opensot_solver',
            name='opensot_solver',
            parameters=[{
                'interface': interface,
                'use_robot': ParameterValue(use_robot, value_type=bool),
                'enable_collision_avoidance': ParameterValue(enable_collision_avoidance, value_type=bool),
                'send_cmds_to_robot': ParameterValue(send_cmds_to_robot, value_type=bool),
                'publish_joint_states_opensot': ParameterValue(publish_joint_states_opensot, value_type=bool),
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            }],
            output='screen'
        ),

        Node(
            package='g1pilot',
            executable='dx3_controller',
            name='dx3_controller',
            parameters=[{
                'arm_controlled': ParameterValue(LaunchConfiguration("arm_controlled"), value_type=str),
                'interface': ParameterValue(LaunchConfiguration("interface"), value_type=str),
                'use_robot': ParameterValue(use_robot, value_type=bool),
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            }],
            output='screen'
        ),
    ])
