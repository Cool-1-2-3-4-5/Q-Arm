r"""Stand-in for Quanser's ``qarm_hardware`` node, for testing without the arm.

Same topics, message types, joint names and 200 Hz rate as
``qarm_nodes/qarm_hardware.py`` in Quanser Academic Resources, so
hardware_bridge cannot tell the difference:

    subscribes  /qarm/joint_position_cmd  std_msgs/Float64MultiArray  4 values, rad
                /qarm/gripper_cmd         std_msgs/Float64            clipped to 0.1-0.9
    publishes   /qarm/joint_states        sensor_msgs/JointState      5 names, 200 Hz

It also copies the two behaviours of the real driver that matter for safety:

  - The command starts at all zeros, so the "arm" heads for the home pose as
    soon as this node starts, exactly as the real one does. Start it away from
    home (``initial_pose``) to see that happen.
  - Each joint moves toward its command at no more than 90 deg/s with
    60 deg/s^2 acceleration, the profile pal.products.qarm.QArm programs into
    the firmware. It is a simple trapezoid, not a model of the motors.

    ros2 run qarm_kinematics qarm_fake_hardware
    ros2 run qarm_kinematics qarm_fake_hardware --ros-args \
        -p initial_pose:="[0.0, -1.4835, 1.309, 0.0]"     # Quanser SLEEP_POSE
"""

import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState

from std_msgs.msg import Float64, Float64MultiArray

#: Names and order used by Quanser's qarm_hardware.
NAMES = ('base_joint', 'shoulder_joint', 'arm_joint', 'wrist_joint', 'gripper_joint')

#: Firmware motion profile from pal.products.qarm.QArm (rad/s and rad/s^2).
PROFILE_VELOCITY = 1.5708
PROFILE_ACCELERATION = 1.0472


def profile_step(position, velocity, target, dt,
                 v_max=PROFILE_VELOCITY, a_max=PROFILE_ACCELERATION):
    """Advance one joint by ``dt`` toward ``target`` with a trapezoidal profile.

    :returns: ``(new_position, new_velocity)``.
    """
    error = target - position
    if abs(error) < 1e-9 and abs(velocity) < a_max * dt:
        return target, 0.0
    # Fastest speed from which we can still stop at the target.
    v_stop = math.sqrt(2.0 * a_max * abs(error))
    v_want = math.copysign(min(v_max, v_stop), error)
    dv = max(-a_max * dt, min(a_max * dt, v_want - velocity))
    velocity += dv
    new = position + velocity * dt
    # Do not overshoot.
    if (target - position) * (target - new) < 0.0:
        return target, 0.0
    return new, velocity


class FakeQArmHardware(Node):
    """Imitate qarm_hardware's topics and timing with a simple motion profile."""

    def __init__(self):
        """Declare parameters, set the start pose and create topics and the timer."""
        super().__init__('qarm_hardware')
        initial = [float(v) for v in self.declare_parameter(
            'initial_pose', [0.0, 0.0, 0.0, 0.0]).value]
        if len(initial) != 4:
            raise ValueError('initial_pose needs 4 values, got {}'.format(len(initial)))
        self._dt = 1.0 / 200.0

        self._pos = initial
        self._vel = [0.0] * 4
        self._cmd = [0.0] * 4          # the real driver starts commanding home
        self._gripper = 0.1

        self._state_pub = self.create_publisher(JointState, '/qarm/joint_states', 10)
        self.create_subscription(Float64MultiArray, '/qarm/joint_position_cmd',
                                 self._on_cmd, 10)
        self.create_subscription(Float64, '/qarm/gripper_cmd', self._on_gripper, 10)
        self.create_timer(self._dt, self._tick)
        self.get_logger().info(
            'FAKE qarm_hardware (no arm). Start pose [{}] deg, commanding home.'.format(
                ', '.join('{:.1f}'.format(math.degrees(v)) for v in initial)))

    def _on_cmd(self, msg):
        """Copy the first four values, as the real driver does."""
        if len(msg.data) >= 4:
            self._cmd = [float(v) for v in msg.data[:4]]

    def _on_gripper(self, msg):
        """Clip to 0.1-0.9, as pal.products.qarm.QArm does."""
        self._gripper = min(0.9, max(0.1, float(msg.data)))

    def _tick(self):
        """Move every joint one step and publish the 'measured' state."""
        for i in range(4):
            self._pos[i], self._vel[i] = profile_step(
                self._pos[i], self._vel[i], self._cmd[i], self._dt)
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(NAMES)
        msg.position = self._pos + [self._gripper]
        msg.velocity = self._vel + [0.0]
        msg.effort = [0.0] * 5
        self._state_pub.publish(msg)


def main(args=None):
    """Run the fake driver until Ctrl+C."""
    rclpy.init(args=args)
    node = FakeQArmHardware()
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
