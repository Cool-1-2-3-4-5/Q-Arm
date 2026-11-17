r"""Drive the arm through Quanser's driver: IK -> bridge -> qarm_hardware.

    # No arm: Quanser's driver replaced by a stand-in with the same topics.
    ros2 launch qarm_kinematics hardware.launch.py

    # Start the stand-in from Quanser's SLEEP_POSE, to see the start-up rule.
    ros2 launch qarm_kinematics hardware.launch.py \
        initial_pose:="[0.0, -1.4835, 1.309, 0.0]"

    # REAL ARM. Only after the field sheet's Part 4 is complete, and with
    # Quanser's workspace sourced so qarm_nodes can be found:
    #   source ~/Documents/Quanser/5_research/qarm/"ROS Examples"/ros2/install/setup.bash
    ros2 launch qarm_kinematics hardware.launch.py real:=true

Then send a target exactly as in simulation:

    ros2 topic pub --once /target_point geometry_msgs/msg/PointStamped \
        "{header: {frame_id: base_link}, point: {x: 0.3, y: 0.2, z: 0.5}}"

DATA FLOW
---------
    target_point -> qarm_move_to -> joint_command -> qarm_bridge
                 -> /qarm/joint_position_cmd -> qarm_hardware (real or fake)
                 -> /qarm/joint_states -> qarm_bridge -> joint_states
                 -> robot_state_publisher -> TF (and qarm_move_to's start pose)

joint_states here carries the MEASURED pose, not the command, so RViz and TF
show where the arm really is.

START-UP, AND WHY IT IS WRITTEN THIS WAY
----------------------------------------
Quanser's qarm_hardware commands the home pose from the moment it starts. With
start_from_measured, qarm_move_to sends nothing until it has read one
measured state, then holds that pose. So on start-up the arm may begin moving
toward home for a fraction of a second and then hold wherever it was when the
first measurement arrived. Support the arm and keep clear of it when
launching.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import xacro


def generate_launch_description():
    """Build the launch description."""
    share = get_package_share_directory('qarm_description')
    urdf = xacro.process_file(os.path.join(share, 'urdf', 'qarm.urdf.xacro')).toxml()
    rviz_config = os.path.join(share, 'rviz', 'qarm.rviz')
    real = LaunchConfiguration('real')

    return LaunchDescription([
        DeclareLaunchArgument('real', default_value='false',
                              description='true = Quanser qarm_hardware (REAL ARM), '
                                          'false = fake stand-in'),
        DeclareLaunchArgument('initial_pose', default_value='[0.0, 0.0, 0.0, 0.0]',
                              description='fake driver only: start pose, rad'),
        DeclareLaunchArgument('speed_scale', default_value='0.25',
                              description='fraction of the rated joint speed to move at'),
        DeclareLaunchArgument('wrist', default_value='N/A',
                              description="wrist command: 'N/A', 'home' or degrees 0-360"),
        DeclareLaunchArgument('rviz', default_value='false',
                              description='also start RViz'),

        Node(
            package='qarm_nodes',
            executable='qarm_hardware',
            output='screen',
            condition=IfCondition(real),
        ),
        Node(
            package='qarm_kinematics',
            executable='qarm_fake_hardware',
            output='screen',
            parameters=[{'initial_pose': ParameterValue(
                LaunchConfiguration('initial_pose'), value_type=None)}],
            condition=UnlessCondition(real),
        ),
        Node(
            package='qarm_kinematics',
            executable='qarm_bridge',
            output='screen',
        ),
        Node(
            package='qarm_kinematics',
            executable='qarm_move_to',
            output='screen',
            parameters=[{
                'command_topic': 'joint_command',
                'start_from_measured': True,
                'speed_scale': ParameterValue(
                    LaunchConfiguration('speed_scale'), value_type=float),
                'wrist': ParameterValue(LaunchConfiguration('wrist'), value_type=str),
            }],
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': urdf}],
        ),
        # Finger joints for TF/RViz only. The real gripper is commanded by
        # qarm_bridge, which listens on the same gripper/command topic.
        Node(
            package='qarm_description',
            executable='qarm_gripper',
            output='screen',
            remappings=[('gripper/joint_states', 'joint_states')],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
