import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch_ros.actions

def generate_launch_description():
    package_name = 'spina_inverse_kinematics'

    # File paths
    urdf_file = os.path.join(get_package_share_directory(package_name), 'urdf', 'spina_robot.urdf')

    # Read URDF contents
    with open(urdf_file, 'r') as infp:
        robot_description = infp.read()

    return LaunchDescription([
        launch_ros.actions.Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),
        launch_ros.actions.Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),
        launch_ros.actions.Node(
            package=package_name,
            executable='forward_kinematics_node',
            name='forward_kinematics_node',
            output='screen'
        )
    ])

if __name__ == '__main__':
    generate_launch_description()
