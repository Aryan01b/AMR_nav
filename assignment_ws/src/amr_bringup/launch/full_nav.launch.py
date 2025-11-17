from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package directories
    description_pkg = get_package_share_directory('amr_sim')
    nav2_pkg = get_package_share_directory('amr_nav2')
    
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml_file = LaunchConfiguration('map')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(nav2_pkg, 'maps', 'my_room_map.yaml'),
        description='Full path to map yaml file to load'
    )
    
    # Include Gazebo + Robot + Localization launch
    # This is your existing launch file that starts Gazebo, spawns robot, and runs EKF
    # UPDATE THIS FILENAME if your launch file has a different name
    gazebo_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(description_pkg, 'launch', 'display.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )
    
    # Include Navigation launch
    # This starts map_server, AMCL, and all Nav2 nodes
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(nav2_pkg, 'launch', 'navigation.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_yaml_file
        }.items()
    )
    
    return LaunchDescription([
        # Declare launch arguments
        declare_use_sim_time_cmd,
        declare_map_yaml_cmd,
        
        # Launch Gazebo + Robot + EKF
        gazebo_robot_launch,
        
        # Launch Navigation stack
        navigation_launch,
    ])