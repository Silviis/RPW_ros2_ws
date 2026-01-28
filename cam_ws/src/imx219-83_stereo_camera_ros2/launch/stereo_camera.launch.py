from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():

    camera_params = PathJoinSubstitution([
        FindPackageShare('imx219-83_stereo_camera_ros2'),
        'config',
        'stereo_camera.yaml'
    ])

    return LaunchDescription([

        # =======================
        # GSCAM nodes
        # =======================
        Node(
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
        ),

        Node(
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
        ),

        # =======================
        # Stereo image processing
        # =======================
        Node(
            package='stereo_image_proc',
            executable='stereo_image_proc',
            name='stereo_image_proc',
            remappings=[
                ('left/image_raw',  '/stereo/left/image_raw'),
                ('left/camera_info', '/stereo/left/camera_info'),
                ('right/image_raw', '/stereo/right/image_raw'),
                ('right/camera_info', '/stereo/right/camera_info'),
            ],
            output='screen'
        ),

        # =======================
        # IMU wrapper node
        # =======================
        Node(
            package='imx219-83_stereo_camera_ros2',
            executable='icm20948_node',
            name='imu_publisher_node',
            output='screen'
        ),
    ])
