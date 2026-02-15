from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    # =======================
    # Stereo camera publisher
    # =======================
    camera_params = PathJoinSubstitution([
        FindPackageShare('imx219-83_stereo_camera_ros2'),
        'config',
        'cameras.yaml'
    ])

    stereo_cam_publisher = Node(
        package='imx219-83_stereo_camera_ros2',
        executable='stereo_camera_node',
        name='stereo_camera_node',
        output='screen',
        parameters=[camera_params]
    )

    # =======================
    # Image rectification
    # =======================
    image_proc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('imx219-83_stereo_camera_ros2'),
                'launch',
                'image_proc.launch.py'
            ])
        ])
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
        stereo_cam_publisher,
        image_proc,
        imu_publisher
    ])
