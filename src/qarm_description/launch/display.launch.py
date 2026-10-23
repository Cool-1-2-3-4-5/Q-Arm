import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    pkg_name = 'qarm_description'
    pkg_share = get_package_share_directory(pkg_name)

    # Process Xacro into a URDF string
    xacro_file = os.path.join(pkg_share, 'urdf/qarm.urdf.xacro')
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    # Path to the saved RViz config
    rviz_config = os.path.join(pkg_share, 'rviz', 'qarm.rviz')

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_raw,
        }],
    )

    node_joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
    )

    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
    )

    return LaunchDescription([
        node_robot_state_publisher,
        node_joint_state_publisher_gui,
        node_rviz,
    ])
