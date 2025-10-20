import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml
import xacro

# Proxy server for Google Maps:
# https://github.com/danielsnider/MapViz-Tile-Map-Google-Maps-Satellite
# $ sudo docker run -p 8080:8080 -d -t -v ~/mapproxy:/mapproxy danielsnider/mapproxy


def generate_launch_description():
    share_dir = get_package_share_directory('rovesugv_navsim')
    nav2_dir = get_package_share_directory('nav2_bringup')

    config_bridge = os.path.join(share_dir, 'config', 'bridge_config.yaml')
    config_mapviz = os.path.join(share_dir, 'config', 'config.mvc')
    config_nav2 = os.path.join(share_dir, 'config', 'nav2_params.yaml')
    config_rl = os.path.join(share_dir, 'config', 'config_localization.yaml')
    config_rviz = os.path.join(share_dir, 'config', 'config.rviz')
    launch_dir = os.path.join(share_dir, 'launch')
    world_file = os.path.join(share_dir, 'worlds', 'frami.sdf')

    nav2_params = RewrittenYaml(
        source_file=config_nav2,
        root_key='',
        param_rewrites='',
        convert_types=True,
    )

    urdf_file = os.path.join(share_dir, 'models', 'rover', 'rover.urdf')
    robot_description = xacro.process_file(urdf_file).toxml()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_description},
        ],
    )
    # Gazebo simulation
    gazebo_server = ExecuteProcess(
        cmd=['ign', 'gazebo', '-s', '-r', world_file],
        cwd=[launch_dir],
        output='both',
    )
    gazebo_client = ExecuteProcess(
        cmd=['ign', 'gazebo', '-g'],
        cwd=[launch_dir],
        output='both',
    )
    ros_gazebo_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': config_bridge}],
        output='screen',
    )
    spawn_entity_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-world', 'frami',
            '-entity_name', 'rover',
            '-topic', '/robot_description'
        ],
        output='screen',
    )
    # RViz2
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', config_rviz],
        output='screen',
    )
    # Robot localization
    ekf_filter_odom_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_odom',
        output="screen",
        parameters=[config_rl, {'use_sim_time': True}],
        remappings=[('odometry/filtered', 'odometry/local')]
    )
    ekf_filter_map_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_map',
        output='screen',
        parameters=[config_rl, {'use_sim_time': True}],
        remappings=[('odometry/filtered', 'odometry/global')]
    )
    navsat_transform_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform',
        output='screen',
        parameters=[config_rl, {'use_sim_time': True}],
        remappings=[
            ('imu/data', 'imu/data'),
            ('gps/fix', 'gps/fix'),
            ('gps/filtered', 'gps/filtered'),
            ('odometry/gps', 'odometry/gps'),
            ('odometry/filtered', 'odometry/global'),
        ],
    )
    # Navigation2
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'True',
            'params_file': nav2_params,
            'autostart': 'True',
        }.items(),
    )
    # RViz2
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_dir, 'launch', 'rviz_launch.py')
        ),
    )
    # Mapviz
    mapviz_node = Node(
        package="mapviz",
        executable="mapviz",
        name="mapviz",
        parameters=[{"config": config_mapviz}]
    )
    swri_transform_node = Node(
        package="swri_transform_util",
        executable="initialize_origin.py",
        name="initialize_origin",
        remappings=[
            ("fix", "gps/fix"),
        ],
    )
    static_transform_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="swri_transform",
        arguments=["0", "0", "0", "0", "0", "0", "map", "origin"]
    )

    ld = LaunchDescription()

    ld.add_action(gazebo_server)
    ld.add_action(gazebo_client)
    ld.add_action(ros_gazebo_bridge_node)

    ld.add_action(robot_state_publisher_node)

    ld.add_action(ekf_filter_odom_node)
    ld.add_action(ekf_filter_map_node)
    ld.add_action(navsat_transform_node)

    ld.add_action(nav2_launch)
    ld.add_action(rviz_launch)

    ld.add_action(mapviz_node)
    ld.add_action(swri_transform_node)
    ld.add_action(static_transform_node)

    return ld
