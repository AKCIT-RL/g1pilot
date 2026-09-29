from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import yaml

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

    mola_lo_pipeline = LaunchConfiguration('mola_lo_pipeline')
    mola_lo_pipeline_arg = DeclareLaunchArgument(
        'mola_lo_pipeline', default_value='/opt/ros/jazzy/share/mola_lidar_odometry/pipelines/lidar3d-default.yaml',
        description=(
            'ICP pipeline YAML. Default is MOLA\'s own GICP pipeline, paired with a mola_map '
            'built from sm2mm_pipeline.yaml.'
        )
    )
    icp_cloud_decimation = LaunchConfiguration('icp_cloud_decimation_m')
    icp_cloud_decimation_arg = DeclareLaunchArgument(
        'icp_cloud_decimation_m', default_value='0.30',
        description=(
            'Voxel size (m) for ICP correspondence decimation. MOLA\'s default floor is 0.60m; '
            'finer values keep more distinguishing detail between similar-looking areas.'
        )
    )
    icp_cloud_decimation_env_var = SetEnvironmentVariable(
        name='MOLA_ICP_CLOUD_DECIMATION', value=icp_cloud_decimation
    )

    vehicle_frame_env_var = SetEnvironmentVariable(
        name='MOLA_LO_PUBLISH_VEHICLE_FRAME', value='base_footprint'
    )

    def _mola_initial_pose_env_vars(context, *args, **kwargs):
        """
            Reads home_x/home_y/home_yaw_deg from the resolved nav_config file and returns the
            matching MOLA_INITIAL_X/Y/YAW SetEnvironmentVariable actions. Falls back to (0,0,0)
            (MOLA's own default) with a warning if the file or those keys aren't there.

        Args:
            context (launch.LaunchContext): Used to resolve nav_config's actual path.

        Returns:
            list[launch.actions.SetEnvironmentVariable]: The three env var actions.
        """
        nav_config_path = LaunchConfiguration('nav_config').perform(context)
        x, y, yaw_deg = 0.0, 0.0, 0.0
        try:
            with open(nav_config_path) as f:
                params = yaml.safe_load(f)['dijkstra_planner']['ros__parameters']
            x, y, yaw_deg = params['home_x'], params['home_y'], params['home_yaw_deg']
        except (OSError, KeyError, TypeError, yaml.YAMLError) as e:
            print(
                f"[mission_launcher] Could not read home_x/y/yaw_deg from {nav_config_path} "
                f"({e}) -- MOLA_INITIAL_X/Y/YAW falling back to (0,0,0)."
            )
        return [
            SetEnvironmentVariable(name='MOLA_INITIAL_X', value=str(x)),
            SetEnvironmentVariable(name='MOLA_INITIAL_Y', value=str(y)),
            SetEnvironmentVariable(name='MOLA_INITIAL_YAW', value=str(yaw_deg)),
        ]
    mola_initial_pose_env_vars = OpaqueFunction(function=_mola_initial_pose_env_vars)

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
            'rviz_config': rviz_config,
            'mola_lo_pipeline': mola_lo_pipeline,
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
        }, nav_config]
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
        mola_lo_pipeline_arg,
        icp_cloud_decimation_arg,
        icp_cloud_decimation_env_var,
        vehicle_frame_env_var,
        mola_initial_pose_env_vars,
        mola_launcher,
        pcl_to_grid,
        planner,
        navigation
    ])
