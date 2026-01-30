from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    camera_params = PathJoinSubstitution([
        FindPackageShare('imx219-83_stereo_camera_ros2'),
        'config',
        'stereo_camera.yaml'
    ])

    # =======================
    # GSCAM nodes
    # =======================
    left_camera_publisher = Node(
        package='gscam',
        executable='gscam_node',
        name='imx_219_left',
        parameters=[camera_params],
        remappings=[
            ('/camera/image_raw', '/stereo/left/image_raw'),
            ('/camera/camera_info', '/stereo/left/camera_info'),
            ('/camera/image_raw/compressed',
             '/stereo/left/image_raw/compressed'),
            ('/camera/image_raw/compressedDepth',
             '/stereo/left/image_raw/compressedDepth'),
        ],
        output='screen'
    )

    right_camera_publisher = Node(
        package='gscam',
        executable='gscam_node',
        name='imx_219_right',
        parameters=[camera_params],
        remappings=[
            ('/camera/image_raw', '/stereo/right/image_raw'),
            ('/camera/camera_info', '/stereo/right/camera_info'),
            ('/camera/image_raw/compressed',
             '/stereo/right/image_raw/compressed'),
            ('/camera/image_raw/compressedDepth',
             '/stereo/right/image_raw/compressedDepth'),
        ],
        output='screen'
    )

    # =======================
    # Image rectification
    # =======================
    rectification = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('stereo_image_proc'),
                'launch',
                'stereo_image_proc.launch.py'
            ])
        ]),
        launch_arguments={
            'left_namespace': '/stereo/left',
            'right_namespace': '/stereo/right',
            'launch_image_proc': 'true',
            'approximate_sync': 'true'
        }.items()
    )

    # =======================
    # IMU wrapper node
    # =======================
    imu_publisher = Node(
        package='imx219-83_stereo_camera_ros2',
        executable='icm20948_node',
        name='imu_publisher_node',
        output='screen'
    )

    return LaunchDescription([
        left_camera_publisher,
        right_camera_publisher,
        rectification,
        imu_publisher
    ])
