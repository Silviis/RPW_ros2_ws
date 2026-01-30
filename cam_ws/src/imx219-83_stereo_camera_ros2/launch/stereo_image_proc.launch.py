from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    return LaunchDescription([

        # =======================
        # Image rectification
        # =======================
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('stereo_image_proc'),
                    'launch',
                    'stereo_image_proc.launch.py'
                ])
            ]),
            launch_arguments={
                'left_namespace': '/stereo/left/',
                'right_namespace': '/stereo/right/',
                'launch_image_proc': 'true',
                'approximate_sync': 'true'
            }.items(),
        ),
    ])
