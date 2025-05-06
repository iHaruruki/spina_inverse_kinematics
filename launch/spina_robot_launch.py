from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_spine_robot',
            executable='inverse_kinematics_node',
            name='ik_node'
        )
    ])
