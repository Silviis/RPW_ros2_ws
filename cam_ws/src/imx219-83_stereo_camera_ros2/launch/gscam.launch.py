from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg_share = get_package_share_directory('imx219_stereo_camera_ros2')
    left_config = os.path.join(pkg_share, 'config', 'left.yaml')
    right_config = os.path.join(pkg_share, 'config', 'right.yaml')

    return LaunchDescription([
        # GSCAM nodes
        Node(
            package='gscam',
            executable='gscam_node',
            name='imx_219_left',
            parameters=[left_config],
            output='screen'
        ),

        Node(
            package='gscam',
            executable='gscam_node',
            name='imx_219_right',
            parameters=[right_config],
            output='screen'
        ),
    ])
