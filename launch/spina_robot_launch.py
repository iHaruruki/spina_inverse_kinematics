import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('spina_inverse_kinematics')
    urdf_path = os.path.join(pkg_share, 'urdf', 'spina_robot.urdf')

    # URDF を文字列として読み込む
    with open(urdf_path, 'r') as infp:
        robot_description_content = infp.read()

    return LaunchDescription([
        # robot_state_publisher ノード
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_content}]
        ),

        # joint_state_publisher_gui（デバッグ用。不要なら消す）
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),

        # IK ノード
        Node(
            package='spina_inverse_kinematics',
            executable='inverse_kinematics_node',
            name='ik_node',
            output='screen'
        ),

        # rviz2
        #Node(
        #    package='rviz2',
        #    executable='rviz2',
        #    name='rviz2',
        #    output='screen',
        #    arguments=['-d', os.path.join(pkg_share, 'rviz', 'spina_robot.rviz')]
        #),
    ])
