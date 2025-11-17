from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Package directories
    nav2_share = get_package_share_directory('amr_nav2')

    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(nav2_share, 'maps', 'my_room_map.yaml'),
        description='Full path to map yaml file to load'
    )
    
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(nav2_share, 'config', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes'
    )
    
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack'
    )
    
    # Map server node
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            params_file,
            {'yaml_filename': map_yaml_file},
            {'use_sim_time': use_sim_time}
        ]
    )
    
    # AMCL node
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            os.path.join(nav2_share, 'config', 'amcl.yaml'),
            {'use_sim_time': use_sim_time}
        ]
    )
    
    # Controller server
    controller_server_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[params_file],
        remappings=[
            ('/cmd_vel', '/cmd_vel')
        ]
    )
    
    # Planner server
    planner_server_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file]
    )
    
    # Smoother server
    smoother_server_node = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[params_file]
    )
    
    # Behavior server
    behavior_server_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[params_file]
    )
    
    # BT Navigator
    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[params_file]
    )
    
    # Waypoint follower
    waypoint_follower_node = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[params_file]
    )
    
    # Velocity smoother
    velocity_smoother_node = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[params_file],
        remappings=[
            ('/cmd_vel', '/cmd_vel_nav'),
            ('/cmd_vel_smoothed', '/cmd_vel')
        ]
    )
    
    # Collision monitor
    collision_monitor_node = Node(
        package='nav2_collision_monitor',
        executable='collision_monitor',
        name='collision_monitor',
        output='screen',
        parameters=[params_file]
    )
    
    # Lifecycle manager for localization
    lifecycle_manager_localization = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[
            {'autostart': autostart},
            {'node_names': ['map_server', 'amcl']},
            {'use_sim_time': use_sim_time}
        ]
    )
    
    # Lifecycle manager for navigation
    lifecycle_manager_navigation = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[
            {'autostart': autostart},
            {'node_names': [
                'controller_server',
                'smoother_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
                'waypoint_follower',
                'velocity_smoother',
                'collision_monitor'
            ]},
            {'use_sim_time': use_sim_time}
        ]
    )
    
    return LaunchDescription([
        # Declare launch arguments
        declare_use_sim_time_cmd,
        declare_map_yaml_cmd,
        declare_params_file_cmd,
        declare_autostart_cmd,
        
        # Localization nodes
        map_server_node,
        amcl_node,
        lifecycle_manager_localization,
        
        # Navigation nodes
        controller_server_node,
        planner_server_node,
        smoother_server_node,
        behavior_server_node,
        bt_navigator_node,
        waypoint_follower_node,
        velocity_smoother_node,
        collision_monitor_node,
        lifecycle_manager_navigation,
    ])