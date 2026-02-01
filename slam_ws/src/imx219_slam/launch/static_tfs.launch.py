from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([

        # -------------------------------------------------
        # base_link -> camera_link
        # -------------------------------------------------
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_imu',
            arguments=[
                '0.0', '0.0', '0.0',   # x y z (meters)
                '-0.5', '0.5', '-0.5', '0.5',  # qx qy qz qw
                'base_link',
                'imu_link'
            ]
        ),

        # -------------------------------------------------
        # camera_link -> left_camera_optical
        # -------------------------------------------------
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_to_left_optical',
            arguments=[
                '0.0345', '0.0', '0.0',
                '0.0', '0.0', '0.0', '1.0',
                'imu_link',
                'imx_219_left_link'
            ]
        ),

        # -------------------------------------------------
        # camera_link -> right_camera_optical
        # -------------------------------------------------
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_to_right_optical',
            arguments=[
                '-0.0345', '0.0', '0.0',  # baseline (meters!)
                '0.0', '0.0', '0.0', '1.0'
                'imu_link',
                'imx_219_right_link'
            ]
        ),
    ])
