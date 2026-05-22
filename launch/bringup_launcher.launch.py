from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, EnvironmentVariable, PythonExpression
import os
import sys

def generate_launch_description():
    pkg1_share = FindPackageShare('g1pilot').find('g1pilot')
    if not os.environ.get("G1_INTERFACE"):
        sys.exit("ERROR: G1_INTERFACE environment variable is not set.\n"
                 "Set it to your network interface, e.g.: export G1_INTERFACE=eno2")

    use_robot = LaunchConfiguration("use_robot") # "true" if use_robot is true
    interface = LaunchConfiguration("interface")
    use_sim_time = LaunchConfiguration("use_sim_time")

    navigation_launcher = os.path.join(pkg1_share, 'launch', 'navigation_launcher.launch.py')
    robot_state_launcher = os.path.join(pkg1_share, 'launch', 'robot_state_launcher.launch.py')
    teleoperation_launcher = os.path.join(pkg1_share, 'launch', 'teleoperation_launcher.launch.py')
    manipulation_launcher = os.path.join(pkg1_share, 'launch', 'manipulation_launcher.launch.py')

    return LaunchDescription([
        DeclareLaunchArgument("use_robot", default_value="true"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("enable_collision_avoidance", default_value="true"),
        DeclareLaunchArgument(
            "interface",
            default_value=EnvironmentVariable("G1_INTERFACE"),
            description="Network interface for Unitree SDK",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(navigation_launcher),
            launch_arguments={
                "interface": interface,
                "use_robot": use_robot,
                "use_sim_time": use_sim_time,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(robot_state_launcher),
            launch_arguments={
                'interface': interface,
                'use_robot': use_robot,
                'publish_joint_states': use_robot,
                "use_sim_time": use_sim_time,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(teleoperation_launcher),
            launch_arguments={
                "interface": interface,
                "use_robot": use_robot,
                "use_sim_time": use_sim_time,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(manipulation_launcher),
            launch_arguments={
                'interface': interface,
                'use_robot': use_robot,
                'enable_collision_avoidance': LaunchConfiguration('enable_collision_avoidance'),
                'send_cmds_to_robot': use_robot,
                'publish_joint_states_opensot': PythonExpression(['"true" if "', use_robot, '" == "false" else "false"']),
                "use_sim_time": use_sim_time,
            }.items(),
        ),
    ])