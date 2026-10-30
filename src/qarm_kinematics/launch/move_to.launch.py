r"""Bring up the arm with IK driving it: state publisher, move_to and gripper.

    ros2 launch qarm_kinematics move_to.launch.py
    ros2 launch qarm_kinematics move_to.launch.py rviz:=true

Then send a target point and watch the arm go there:

    ros2 topic pub --once /target_point geometry_msgs/msg/PointStamped \
        "{header: {frame_id: base_link}, point: {x: 0.3, y: 0.2, z: 0.5}}"

No joint_state_publisher here. qarm_move_to publishes the four arm joints and
qarm_gripper publishes the two finger joints, both on /joint_states;
robot_state_publisher keeps the state of every joint it has seen, so two
publishers owning different joints is all that is needed. The slider GUI would
only fight the IK for control of the same joints.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import xacro


def generate_launch_description():
    """Build the launch description."""
    share = get_package_share_directory('qarm_description')
    urdf = xacro.process_file(os.path.join(share, 'urdf', 'qarm.urdf.xacro')).toxml()
    rviz_config = os.path.join(share, 'rviz', 'qarm.rviz')

    return LaunchDescription([
        # Off by default: the headless checks and WSL without X11 both need it off.
        DeclareLaunchArgument('rviz', default_value='false',
                              description='also start RViz'),
        DeclareLaunchArgument('speed_scale', default_value='0.5',
                              description='fraction of the rated joint speed to move at'),
        DeclareLaunchArgument('wrist', default_value='N/A',
                              description="wrist command: 'N/A', 'home' or degrees 0-360"),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': urdf}],
        ),
        Node(
            package='qarm_kinematics',
            executable='qarm_move_to',
            output='screen',
            parameters=[{
                # A launch argument arrives as a string. The node declares
                # speed_scale with a float default, so it must be converted
                # here or the parameter server rejects the type.
                'speed_scale': ParameterValue(
                    LaunchConfiguration('speed_scale'), value_type=float),
                'wrist': ParameterValue(
                    LaunchConfiguration('wrist'), value_type=str),
            }],
        ),
        Node(
            package='qarm_description',
            executable='qarm_gripper',
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
