from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():

    madgwick_filter = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('imx219_slam'),
                'launch',
                'madgwick_imu.launch.py'
            ])
        )
    )

    tf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('imx219_slam'),
                'launch',
                'static_tfs.launch.py'
            ])
        )
    )

    odom_parameters = {
        # common params
        'frame_id': 'base_link',
        'odom_frame_id': 'odom',
        'publish_tf': True,
        'tf_prefix': "",
        'initial_pose': "",
        'sync_queue_size': 50,
        'publish_null_when_lost': True,
        'ground_truth_frame_id': "",
        'ground_truth_base_frame_id': "",
        'guess_frame_id': "",
        'guess_min_translation': 0.0,
        'guess_min_rotation': 0.0,
        'config_path': "",
        'wait_imu_to_init': True,
        'use_sim_time': False,
        # Stereo odom params
        'approx_sync': True,
        'approx_sync_max_interval': 0.03,
        'subscribe_rgbd': False,
        'Odom/Strategy': '9',
        'OdomVINSFusion/ConfigPath': '/workspaces/RPW_vislam/slam_ws/src/imx219_slam/config/vins_config.yaml'
    }

    slam_parameters = {
        # common params
        'subscribe_depth': False,
        'subscribe_scan': False,
        'subscribe_scan_cloud': False,
        'subscribe_stereo': True,
        'subscribe_rgbd': False,
        'subscribe_rgb': False,
        'frame_id': "base_link",
        'map_frame_id': "map",
        'sync_queue_size': 10,
        'publish_tf': True,
        'tf_delay': 0.10,
        'tf_prefix': "",
        'approx_sync': True,
        'approx_sync_max_interval': 0.01,
        'odom_sensor_sync': False,
        'use_sim_time': False,
    }

    remappings = [
        ('left/image_rect', '/stereo/left/image_rect'),
        ('right/image_rect', '/stereo/right/image_rect'),
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
        parameters=[odom_parameters],
        remappings=remappings
    )

    rtabmap_viz = Node(
        package='rtabmap_viz', executable='rtabmap_viz', output='screen',
        parameters=[odom_parameters,
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
        parameters=[slam_parameters],
        remappings=remappings
    )

    return LaunchDescription([
        madgwick_filter,
        tf_launch,
        stereo_odom,
        rtabmap,
        # rtabmap_viz
    ])
