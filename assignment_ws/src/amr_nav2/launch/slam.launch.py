from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import os
def generate_launch_description():

    nav2_share = get_package_share_directory('amr_nav2')
    slam_toolbox_params = os.path.join(nav2_share, 'config', 'slam_mapping.yaml')

    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
    output='screen',
        remappings=[
            ('odom', '/odometry/filtered')
        ],
        parameters=[
            slam_toolbox_params,
            {'use_sim_time': True}
        ],
    )

    return LaunchDescription([slam_toolbox_node])
