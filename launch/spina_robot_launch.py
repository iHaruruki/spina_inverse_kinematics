from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare('spina_inverse_kinematics').find('spina_inverse_kinematics')

    # URDF を robot_description パラメータとして読み込む
    robot_description = Command([
        'xacro ', PathJoinSubstitution([pkg_share, 'urdf', 'spine_robot.urdf'])
    ])

    return LaunchDescription([
        # robot_state_publisher ノード
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),

        # joint_state_publisher_gui （必要であれば）
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

        # rviz2 ノード
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', PathJoinSubstitution([pkg_share, 'rviz', 'spine_robot.rviz'])]
        ),
    ])
