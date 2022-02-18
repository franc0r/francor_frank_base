from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='francor_robot_base_ros2',
            namespace='francor_robot_base',
            executable='frank_base',
            name='frank_base',
            output="screen",
            emulate_tty=True,
            parameters=[
                {"can": "can0"},
            ],
            arguments=['--ros-args', '--log-level', 'info']
        ),
    ])