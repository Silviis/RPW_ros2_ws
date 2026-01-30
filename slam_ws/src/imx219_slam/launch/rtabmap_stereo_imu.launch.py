from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():

    # ----------------------------
    # Static TFs
    # ----------------------------
    tf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('imx219_slam'),
                'launch',
                'static_tfs.launch.py'
            ])
        )
    )

    parameters = {
        'frame_id': 'base_link',
        'odom_frame_id': 'odom',
        'publish_tf': True,
        'subscribe_depth': False,
        'subscribe_stereo': True,
        'subscribe_imu': True,
        'approx_sync': True,
        'sync_queue_size': 10,
        'use_sim_time': False,
    }

    remappings = [
        ('left/image_rect', '/stereo/left/image_raw'),
        ('right/image_rect', '/stereo/right/image_raw'),
        ('left/camera_info', '/stereo/left/camera_info'),
        ('right/camera_info', '/stereo/right/camera_info'),
        ('imu', '/imu/data'),
    ]
    # ----------------------------
    # Stereo Odometry
    # ----------------------------
    stereo_odom = Node(
        package='rtabmap_odom',
        executable='stereo_odometry',
        name='stereo_odometry',
        output='screen',
        parameters=[parameters],
        remappings=remappings
    )

    rtabmap_viz = Node(
        package='rtabmap_viz', executable='rtabmap_viz', output='screen',
        parameters=[parameters,
                    {'odometry_node_name': "stereo_odometry"}],
        remappings=remappings
    )

    # ----------------------------
    # RTAB-Map SLAM
    # ----------------------------
    rtabmap = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[parameters],
        remappings=remappings
    )

    return LaunchDescription([
        tf_launch,
        stereo_odom,
        rtabmap,
        rtabmap_viz
    ])
