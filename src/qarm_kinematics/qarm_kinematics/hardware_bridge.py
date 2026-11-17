r"""Connect our joint commands to Quanser's QArm ROS 2 driver (``qarm_hardware``).

Quanser's ``qarm_nodes`` package ships the node that actually talks to the arm
over USB. This bridge is the thin layer between that node and ours:

    our side                                  Quanser's qarm_hardware
    --------                                  -----------------------
    joint_command   sensor_msgs/JointState -> /qarm/joint_position_cmd
                    joint_1..joint_4, rad     std_msgs/Float64MultiArray
                                              [base, shoulder, elbow, wrist], rad
    gripper/command std_msgs/Bool          -> /qarm/gripper_cmd   std_msgs/Float64
                    true = open               0.1 = open, 0.9 = closed
    joint_states    sensor_msgs/JointState <- /qarm/joint_states  sensor_msgs/JointState
                    joint_1..joint_4          base_joint, shoulder_joint,
                                              arm_joint, wrist_joint, gripper_joint

The joint angle conventions need no conversion. Quanser's own forward
kinematics (hal.products.qarm.QArmUtilities) agrees with ours to 3e-16 m over
2006 poses, with the same zero pose and the same positive direction for every
joint, and pal.products.qarm.QArm uses the same four joint ranges as
robot_params.JOINT_LIMITS. Only the names and message types differ.

The gripper numbers come from pal.products.qarm.QArm.read_write_std, which
clips the command to 0.1-0.9 and comments 0.1 as open and 0.9 as closed. Field
sheet step 4.4 confirms that on the real arm.

SAFETY CHECKS
-------------
Every joint command is checked before it is forwarded. A command is dropped,
with an error in the log, when it

  - is missing one of joint_1..joint_4,
  - holds a NaN or infinity,
  - is outside robot_params.JOINT_LIMITS.

Nothing is clamped. A command that is out of range is a bug upstream, and
quietly moving the arm somewhere else would hide it.

The bridge only forwards; it does not plan. The arm's firmware limits every
joint to 90 deg/s with 60 deg/s^2 acceleration (the profile set in
pal.products.qarm.QArm), so a single far target still produces a smooth move.

WHAT QUANSER'S DRIVER DOES THAT THIS CANNOT CHANGE
--------------------------------------------------
``qarm_hardware`` starts with an all-zero command and writes it at 200 Hz, so
the arm drives to the home pose the moment that node starts, before any
command from us arrives. It also has no command timeout: if we stop
publishing, it holds the last target.
"""

import math

from qarm_kinematics.robot_params import JOINT_LIMITS, JOINT_NAMES

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState

from std_msgs.msg import Bool, Float64, Float64MultiArray

#: Joint names Quanser's qarm_hardware publishes, in the order of JOINT_NAMES.
QUANSER_JOINT_NAMES = ('base_joint', 'shoulder_joint', 'arm_joint', 'wrist_joint')

#: Quanser's name for the gripper entry in /qarm/joint_states.
QUANSER_GRIPPER_NAME = 'gripper_joint'

#: Gripper command values from pal.products.qarm.QArm (it clips to 0.1-0.9).
GRIPPER_OPEN = 0.1
GRIPPER_CLOSED = 0.9


def command_to_quanser(names, positions):
    """Turn a joint_1..joint_4 command into Quanser's 4-value list.

    :param names: joint names from a sensor_msgs/JointState.
    :param positions: matching joint positions in radians.
    :returns: ``[base, shoulder, elbow, wrist]`` in radians.
    :raises ValueError: if a joint is missing, not finite or out of range.
    """
    if len(names) != len(positions):
        raise ValueError('{} names but {} positions'.format(len(names), len(positions)))
    by_name = dict(zip(names, positions))
    out = []
    for i, name in enumerate(JOINT_NAMES):
        if name not in by_name:
            raise ValueError('command has no {}'.format(name))
        value = float(by_name[name])
        if not math.isfinite(value):
            raise ValueError('{} is {}'.format(name, value))
        lower, upper = JOINT_LIMITS[i]
        if not lower <= value <= upper:
            raise ValueError('{} = {:.4f} rad ({:.2f} deg) is outside [{:.2f}, {:.2f}] deg'.format(
                name, value, math.degrees(value), math.degrees(lower), math.degrees(upper)))
        out.append(value)
    return out


def quanser_to_state(names, positions, velocities=()):
    """Turn Quanser's measured joint state into joint_1..joint_4.

    :param names: names from /qarm/joint_states (base_joint, ...).
    :param positions: matching measured positions in radians.
    :param velocities: matching measured speeds in rad/s, or empty.
    :returns: ``(positions, velocities)``, each four values in JOINT_NAMES
        order; velocities is empty if none were given.
    :raises ValueError: if one of the four arm joints is missing.
    """
    index = {name: i for i, name in enumerate(names)}
    pos = []
    vel = []
    for name in QUANSER_JOINT_NAMES:
        if name not in index or index[name] >= len(positions):
            raise ValueError('measured state has no {}'.format(name))
        i = index[name]
        pos.append(float(positions[i]))
        if len(velocities) == len(positions):
            vel.append(float(velocities[i]))
    return pos, vel


class HardwareBridge(Node):
    """Forward checked joint and gripper commands; republish measured joints."""

    def __init__(self):
        """Declare topic parameters and set up the publishers and subscribers."""
        super().__init__('qarm_bridge')

        cmd_out = str(self.declare_parameter(
            'quanser_joint_cmd_topic', '/qarm/joint_position_cmd').value)
        gripper_out = str(self.declare_parameter(
            'quanser_gripper_cmd_topic', '/qarm/gripper_cmd').value)
        state_in = str(self.declare_parameter(
            'quanser_joint_state_topic', '/qarm/joint_states').value)

        self._cmd_pub = self.create_publisher(Float64MultiArray, cmd_out, 10)
        self._gripper_pub = self.create_publisher(Float64, gripper_out, 10)
        self._state_pub = self.create_publisher(JointState, 'joint_states', 10)

        self.create_subscription(JointState, 'joint_command', self._on_command, 10)
        self.create_subscription(Bool, 'gripper/command', self._on_gripper, 10)
        self.create_subscription(JointState, state_in, self._on_measured, 10)

        self._rejected = 0
        self._have_state = False
        self.get_logger().info(
            'bridge ready: joint_command -> {}, gripper/command -> {}, {} -> joint_states'.format(
                cmd_out, gripper_out, state_in))

    def _on_command(self, msg):
        """Check one joint command and forward it, or log why it was dropped."""
        try:
            data = command_to_quanser(list(msg.name), list(msg.position))
        except ValueError as exc:
            self._rejected += 1
            # Throttled: a bad upstream publisher at 50 Hz would flood the log.
            self.get_logger().error(
                'dropped joint command ({} so far): {}'.format(self._rejected, exc),
                throttle_duration_sec=1.0)
            return
        self._cmd_pub.publish(Float64MultiArray(data=data))

    def _on_gripper(self, msg):
        """Map the open/close Bool onto Quanser's 0.1 (open) / 0.9 (closed)."""
        value = GRIPPER_OPEN if msg.data else GRIPPER_CLOSED
        self._gripper_pub.publish(Float64(data=value))
        self.get_logger().info('gripper {} -> {:.1f}'.format(
            'OPEN' if msg.data else 'CLOSE', value))

    def _on_measured(self, msg):
        """Republish Quanser's measured arm joints under our joint names."""
        try:
            pos, vel = quanser_to_state(list(msg.name), list(msg.position),
                                        list(msg.velocity))
        except ValueError as exc:
            self.get_logger().error('bad measured state: {}'.format(exc),
                                    throttle_duration_sec=1.0)
            return
        if not self._have_state:
            self._have_state = True
            self.get_logger().info('first measured state: [{}] deg'.format(
                ', '.join('{:.2f}'.format(math.degrees(v)) for v in pos)))
        out = JointState()
        out.header.stamp = msg.header.stamp
        out.name = list(JOINT_NAMES)
        out.position = pos
        out.velocity = vel
        self._state_pub.publish(out)


def main(args=None):
    """Run the bridge until Ctrl+C."""
    rclpy.init(args=args)
    node = HardwareBridge()
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
