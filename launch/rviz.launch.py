import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch_ros.actions

def generate_launch_description():
    package_name = 'spina_inverse_kinematics'

    # File paths
    rviz_config_dir = os.path.join(get_package_share_directory('lucia_cartographer'), 'rviz', 'spina_robot.rviz')

    return LaunchDescription([
    launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            output='screen'
        ),
    ])

if __name__ == '__main__':
    generate_launch_description()
