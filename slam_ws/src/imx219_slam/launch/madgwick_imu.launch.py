from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    madgwick_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='madgwick_filter',
        output='screen',
        parameters=[
            {
                'gain': 0.1,
                'zeta': 0.0,
                'mag_bias_x': 0.0,
                'mag_bias_y': 0.0,
                'mag_bias_z': 0.0,
                'orientation_stddev': 0.0,
                'world_frame': 'nwu',
                'use_mag': False,
                'use_magnetic_field_msg': False,
                'fixed_frame': 'odom',
                'publish_tf': False,
                'reverse_tf': False,
                'constant_dt': 0.0,
                'publish_debug_topics': False,
                'stateless': False,
                'remove_gravity_vector': False
            }
        ]
    )

    ld = LaunchDescription()
    ld.add_action(madgwick_node)
    return ld
