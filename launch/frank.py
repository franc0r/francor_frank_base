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
                    "auto_enable_on_start": False,
                    "auto_enable": False,
                    "accel_limit": 25.0,
                    "gear_ratio": 6.0,
                    "wheel_diameter_m": 0.02,
                    "wheel_separation_x_m": 0.5,
                    "wheel_separation_y_m": 0.5,
                }
            ],
            arguments=['--ros-args', '--log-level', 'info']
        ),
    ])