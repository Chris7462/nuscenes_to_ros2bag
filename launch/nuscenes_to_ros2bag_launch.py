from os.path import join

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    param = join(
        get_package_share_directory('nuscenes_to_ros2bag'), 'param', 'nuscenes_to_ros2bag.yaml'
    )

    nuscenes_to_ros2bag_node = Node(
        package='nuscenes_to_ros2bag',
        executable='nuscenes_to_ros2bag_node',
        name='nuscenes_to_ros2bag_node',
        output='screen',
        parameters=[param]
    )

    return LaunchDescription([
        nuscenes_to_ros2bag_node
    ])
