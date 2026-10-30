#!/usr/bin/env python3
"""
Open/close toggle for the simulated QArm gripper.

Listens for an open/close command and publishes the matching finger joint
position, so the fingers in RViz move. The fingers travel at the real
gripper's speed (Table 7 of the manual: 100 percent per second, so a full
stroke takes 1 s) instead of jumping.

    # open
    ros2 topic pub --once /gripper/command std_msgs/msg/Bool "{data: true}"
    # close
    ros2 topic pub --once /gripper/command std_msgs/msg/Bool "{data: false}"

Topics:
    subscribes  gripper/command       std_msgs/Bool           true = open
    publishes   gripper/joint_states  sensor_msgs/JointState  finger positions

display.launch.py feeds gripper/joint_states into joint_state_publisher as a
source; it merges the finger positions with the arm joints into /joint_states,
which robot_state_publisher turns into TF. Simulation only: on the real arm the
same command will go to the gripper motor instead.
"""

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState

from std_msgs.msg import Bool


class GripperToggle(Node):
    """Turn a boolean open/close command into finger joint states."""

    def __init__(self):
        """Declare parameters, then set up the subscriber, publisher and timer."""
        super().__init__('qarm_gripper')

        # These two must match finger_travel and gripper_stroke_time in
        # qarm.urdf.xacro. finger_travel is per finger: 0 = closed,
        # finger_travel = fully open (a 40 mm gap with the default 0.02).
        self._travel = float(self.declare_parameter('finger_travel', 0.02).value)
        stroke_time = float(self.declare_parameter('stroke_time', 1.0).value)
        rate = float(self.declare_parameter('rate', 20.0).value)
        start_open = bool(self.declare_parameter('start_open', False).value)

        # Both fingers are published explicitly. joint_state_publisher would
        # fill in the mimic joint by itself, but publishing both also lets
        # robot_state_publisher use this topic directly when no
        # joint_state_publisher is running.
        self._joints = ['finger_left_joint', 'finger_right_joint']

        self._target = self._travel if start_open else 0.0
        self._position = self._target
        self._step = self._travel / stroke_time / rate   # metres per tick

        self._pub = self.create_publisher(JointState, 'gripper/joint_states', 10)
        self.create_subscription(Bool, 'gripper/command', self._on_command, 10)
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            'gripper ready ({}). Send std_msgs/Bool on gripper/command: '
            'true = open, false = close.'.format('open' if start_open else 'closed'))

    def _on_command(self, msg):
        """Set the target to fully open (true) or fully closed (false)."""
        self._target = self._travel if msg.data else 0.0
        self.get_logger().info('gripper command: {}'.format('OPEN' if msg.data else 'CLOSE'))

    def _tick(self):
        """Step the fingers toward the target and publish where they are."""
        error = self._target - self._position
        self._position += max(-self._step, min(self._step, error))

        state = JointState()
        state.header.stamp = self.get_clock().now().to_msg()
        state.name = list(self._joints)
        state.position = [self._position] * len(self._joints)
        self._pub.publish(state)


def main(args=None):
    """Run the gripper toggle node until Ctrl+C."""
    rclpy.init(args=args)
    node = GripperToggle()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
