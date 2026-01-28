from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([

        # -------------------------------------------------
        # odom -> base_link
        # -------------------------------------------------
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_base',
            arguments=[
                '0.0', '0.0', '0.0',   # x y z (meters)
                '0.0', '0.0', '0.0', '1.0',  # qx qy qz qw
                'odom',
                'base_link'
            ]
        ),

        # -------------------------------------------------
        # base_link -> camera_link
        # -------------------------------------------------
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_camera',
            arguments=[
                '0.0', '0.0', '0.0',   # x y z (meters)
                '0.0', '0.0', '0.0', '1.0',  # qx qy qz qw
                'base_link',
                'camera_link'
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
                '0.0', '0.0', '0.0',
                '-0.5', '0.5', '-0.5', '0.5',  # optical frame rotation
                'camera_link',
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
                '0.0618', '0.0', '0.0',  # baseline (meters!)
                '-0.5', '0.5', '-0.5', '0.5',
                'camera_link',
                'imx_219_right_link'
            ]
        ),

        # -------------------------------------------------
        # camera_link -> imu_link
        # (PLACEHOLDER — replace with Kalibr values!)
        # -------------------------------------------------
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_to_imu',
            arguments=[
                '0.0', '0.0', '0.0',
                '0.0', '0.0', '0.0', '1.0',
                'camera_link',
                'imu_link'
            ]
        ),
    ])
