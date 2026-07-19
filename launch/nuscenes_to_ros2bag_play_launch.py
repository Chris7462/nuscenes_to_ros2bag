from os.path import join

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_share = get_package_share_directory('nuscenes_to_ros2bag')

    bag_path_arg = DeclareLaunchArgument(
        'bag_path',
        description='Path to the bag directory produced by nuscenes_to_ros2bag (e.g. scene-0061_bag)'
    )

    rate_arg = DeclareLaunchArgument(
        'rate',
        default_value='1.0',
        description='Playback rate multiplier'
    )

    bag_exec = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', '-r', LaunchConfiguration('rate'),
             LaunchConfiguration('bag_path'),
             '-l',
             '--clock',
             '--qos-profile-overrides-path',
             join(pkg_share, 'config', 'qos_override_offline.yaml')]
    )

    return LaunchDescription([
        bag_path_arg,
        rate_arg,
        bag_exec
    ])
