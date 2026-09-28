from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare('g1pilot')
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    mola_map = LaunchConfiguration('mola_map')
    rviz_config = LaunchConfiguration('rviz_config')
    use_robot = LaunchConfiguration('use_robot')
    start_mapping_enabled = LaunchConfiguration('start_mapping_enabled')
    nav_config = LaunchConfiguration('nav_config')

    # Arguments
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false')
    mola_map_arg = DeclareLaunchArgument('mola_map', default_value='/ros2_ws/src/g1pilot/final_map.mm')
    rviz_config_arg = DeclareLaunchArgument('rviz_config', default_value='/ros2_ws/src/g1pilot/rviz.rviz')
    use_robot_arg = DeclareLaunchArgument('use_robot', default_value='false')
    start_mapping_enabled_arg = DeclareLaunchArgument('start_mapping_enabled', default_value='false')
    nav_config_arg = DeclareLaunchArgument('nav_config', default_value='/ros2_ws/src/g1pilot/nav.yaml')

    # MOLA Localization
    mola_launcher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_share, 'launch', 'mola_launcher.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'start_mapping_enabled': start_mapping_enabled,
            'generate_simplemap': 'false',
            'mola_initial_map_mm_file': mola_map,
            'use_rviz': 'true',
            'rviz_config': rviz_config
        }.items()
    )

    # 3D PCL to 2D Occupancy Grid Bridge
    pcl_to_grid = Node(
        package='g1pilot',
        executable='pcl_to_grid',
        name='pcl_to_grid',
        parameters=[{
            'use_sim_time': use_sim_time,
            'min_z': 0.08,
            'max_z': 1.2,
            'mola_map': mola_map,
            'width_m': 100.0,
            'height_m': 100.0,
            'min_points_per_cell': 7,
            'min_obstacle_height': 0.08,
        }]
    )

    # Dijkstra Path Planner
    planner = Node(
        package='g1pilot',
        executable='dijkstra_planner',
        name='dijkstra_planner',
        parameters=[{'use_sim_time': use_sim_time}, nav_config],
        remappings=[
            ('/g1pilot/goal', '/goal_pose'),  # RViz default tool topic
            ('/lidar_odometry/pose_fixed', '/lidar_odometry/pose'), # MOLA output topic
        ]
    )

    # Navigation Control (Loco Client + Nav2Point)
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_share, 'launch', 'navigation_launcher.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'use_robot': use_robot,
            'nav_config': nav_config,
        }.items()
    )

    return LaunchDescription([
        use_sim_time_arg,
        mola_map_arg,
        rviz_config_arg,
        use_robot_arg,
        start_mapping_enabled_arg,
        nav_config_arg,
        mola_launcher,
        pcl_to_grid,
        planner,
        navigation
    ])
