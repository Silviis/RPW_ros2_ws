from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():

    camera_params = PathJoinSubstitution([
                FindPackageShare('imx219-83_stereo_camera_ros2'), 'config', 'stereo_camera.yaml'])
    
    return LaunchDescription([
        # GSCAM nodes
        Node(
            package='gscam',
            executable='gscam_node',
            name='imx_219_left',
            parameters=[camera_params],
            output='screen'
        ),

        Node(
            package='gscam',
            executable='gscam_node',
            name='imx_219_right',
            parameters=[camera_params],
            output='screen'
        ),

        # IMU wrapper node
        Node(
            package='imx219-83_stereo_camera_ros2',
            executable='icm_wrapper_node',
            name='imu_wrapper_node',
            output='screen'
        )
    ])
