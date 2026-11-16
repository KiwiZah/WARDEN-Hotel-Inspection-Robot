import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    mux_config = os.path.join(
        get_package_share_directory('warden_line_follow'),
        'config', 'twist_mux.yaml')

    return LaunchDescription([
        Node(
            package='twist_mux', executable='twist_mux', name='twist_mux',
            output='screen', parameters=[mux_config],
            remappings=[('cmd_vel_out', 'cmd_vel')],
	),

        # --- Joystick teleop (delete these two if you run your own) ---
        Node(
            package='joy', executable='joy_node', name='joy_node', output='screen',
        ),
        Node(
            package='teleop_twist_joy', executable='teleop_node', name='teleop_twist_joy',
            output='screen',
            parameters=[{
                'require_enable_button': True,
                'enable_button': 0,
                'axis_linear.x': 1,
                'axis_angular.yaw': 3,
                'scale_linear.x': 0.18,
                'scale_angular.yaw': 1.0,
            }],
            remappings=[('cmd_vel', 'cmd_vel_joy')],
        ),

        # --- Autonomous line follower ---
        Node(
            package='warden_line_follow', executable='line_follower', name='line_follower',
            output='screen',
            parameters=[{
                'sensor_pins': [26, 19, 13, 6, 5],   # physical L->R; reverse if Stage-3 test is mirrored
                'line_detected_level': 1,            # black reads 1 on your board
                'base_speed': 0.06,
                'kp': 0.9,
                'kd': 0.5,
            }],
        ),
    ])
