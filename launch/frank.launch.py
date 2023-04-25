from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='francor_frank_base',
            namespace='francor_frank_base',
            executable='frank_base',
            name='frank_base',
            output="screen",
            emulate_tty=True,
            parameters=[
                {
                    "can": "can0",
                    "cmd_vel_max_timeout_s" : 1.0,
                    "enable_cmd_vel_timeout" : True,
                    "auto_enable_on_start": False,
                    "auto_enable": True,
                    "accel_limit": 25.0,
                    "gear_ratio": 6.0,
                    "wheel_diameter_m": 0.0315,
                    "wheel_separation_x_m": 0.41,
                    "wheel_separation_y_m": 0.53,
                    "odom_fac": 1.3,
                    "odom_stamp_offset": 0.0,
                }
            ],
            arguments=['--ros-args', '--log-level', 'info']
        ),
        Node(
            package='francor_frank_base',
            namespace='francor_frank_base',
            executable='odom_to_tf',
            name='odom_to_tf',
            output="screen",
            emulate_tty=True,
            parameters=[
                {
                    "odom_stamp_offset: 0.0",
                }
            ],
            arguments=['--ros-args', '--log-level', 'info']
        )
    ])
