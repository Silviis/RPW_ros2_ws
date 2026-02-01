from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    #
    # ===== base_link -> imu_link =====
    # (Assumed collocated & aligned — adjust if needed)
    #
    base_to_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_imu_tf',
        arguments=[
            '0.0', '0.0', '0.0',
            '0.0', '0.0', '0.0', '1.0',
            'base_link',
            'imu_link',
        ]
    )

    #
    # ===== imu_link -> cam0_link (LEFT) =====
    #
    imu_to_cam0 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='imu_to_cam0_tf',
        arguments=[
            '0.049478147395245295',
            '0.01598654933663175',
            '-0.1202255639182398',
            '-0.008956',
            '-0.044558',
            '0.001388',
            '0.998966',
            'imu_link',
            'cam0_link',
        ]
    )

    #
    # ===== imu_link -> cam1_link (RIGHT) =====
    #
    imu_to_cam1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='imu_to_cam1_tf',
        arguments=[
            '-0.009546705128411478',
            '0.009116973581815568',
            '-0.11809177210442737',
            '-0.041064',
            '-0.056004',
            '0.002376',
            '0.997588',
            'imu_link',
            'cam1_link',
        ]
    )

    #
    # ===== cam0_link -> cam0_optical =====
    # Optical frame rotation: (-90°, 0°, -90°)
    #
    cam0_optical = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='cam0_optical_tf',
        arguments=[
            '0.0', '0.0', '0.0',
            '-0.5', '0.5', '-0.5', '0.5',
            'cam0_link',
            'cam0_optical',
        ]
    )

    #
    # ===== cam1_link -> cam1_optical =====
    #
    cam1_optical = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='cam1_optical_tf',
        arguments=[
            '0.0', '0.0', '0.0',
            '-0.5', '0.5', '-0.5', '0.5',
            'cam1_link',
            'cam1_optical',
        ]
    )

    return LaunchDescription([
        base_to_imu,
        imu_to_cam0,
        imu_to_cam1,
        cam0_optical,
        cam1_optical,
    ])
