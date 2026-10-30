r"""Drive the arm to a target position using the analytical IK.

Give it a point, it works out the joint angles and walks the arm there at the
arm's own speed limit, publishing ``/joint_states`` so RViz (and TF) follow
along. This is the first piece that makes the kinematics usable rather than
just correct.

    ros2 launch qarm_kinematics move_to.launch.py
    ros2 topic pub --once /target_point geometry_msgs/msg/PointStamped \
        "{header: {frame_id: base_link}, point: {x: 0.3, y: 0.2, z: 0.5}}"

WHY A TOPIC AND A STANDARD MESSAGE
----------------------------------
``geometry_msgs/PointStamped`` already means "a point in a named frame", which
is exactly the input. Inventing a custom service for this would add a message
package, a build dependency and a bespoke type for no new information. A topic
also lets the next layer up (a sequence of moves) publish without blocking on a
service call, and ``ros2 topic pub`` is enough to drive it by hand.

The cost is that a topic gives no reply, so the outcome goes to the log and to
``/move_to/done``: ``true`` when the arm reaches the goal, ``false`` when the
target is rejected. That is enough to sequence moves later, and if a real
request/response is ever needed the IK itself is a plain function call.

WHAT IT DOES NOT DO
-------------------
Moves joint-by-joint in a straight line through JOINT space, not Cartesian
space, so the gripper's path between two points is a curve, not a line. That is
fine for "go there", wrong for "draw a straight edge" - Cartesian paths need
trajectory generation (a later goal). It also knows nothing about collisions;
there is no collision model yet.
"""

import math

from geometry_msgs.msg import PointStamped

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState

from std_msgs.msg import Bool

from qarm_kinematics.inverse_kinematics import (
    IKError,
    frame_position,
    solve,
    wrist_from_command,
)
from qarm_kinematics.robot_params import (
    BASE_FRAME,
    EE_FRAME,
    JOINT_NAMES,
    MAX_JOINT_SPEED,
    TIP_FRAME,
)


class MoveToPoint(Node):
    """Turn target points into joint-space moves and publish the joint states."""

    def __init__(self):
        """Declare parameters, then set up the publishers, subscriber and timer."""
        super().__init__('qarm_move_to')

        rate = float(self.declare_parameter('rate', 50.0).value)
        # Fraction of the arm's rated speed to move at. Table 7 allows
        # pi/2 rad/s; half of that is calm enough to watch in RViz and still
        # crosses the workspace in a couple of seconds.
        self._scale = float(self.declare_parameter('speed_scale', 0.5).value)
        # 'N/A' keeps whatever the wrist is already at, so a plain target point
        # does not disturb the gripper's orientation. 'home' or degrees also
        # work; see wrist_from_command.
        self._wrist = str(self.declare_parameter('wrist', 'N/A').value)
        self._frame = str(self.declare_parameter('frame', EE_FRAME).value)
        if self._frame not in (EE_FRAME, TIP_FRAME):
            raise ValueError('frame must be {!r} or {!r}, got {!r}'.format(
                EE_FRAME, TIP_FRAME, self._frame))

        self._position = [0.0] * len(JOINT_NAMES)      # home pose
        self._goal = list(self._position)
        self._moving = False
        self._step = MAX_JOINT_SPEED * self._scale / rate      # radians per tick

        self._states = self.create_publisher(JointState, 'joint_states', 10)
        self._done = self.create_publisher(Bool, 'move_to/done', 10)
        self.create_subscription(PointStamped, 'target_point', self._on_target, 10)
        self.create_timer(1.0 / rate, self._tick)

        self.get_logger().info(
            'move_to ready at the home pose, targeting {} at {:.0f}% of rated speed. '
            'Publish a geometry_msgs/PointStamped on target_point.'.format(
                self._frame, 100.0 * self._scale))

    def _on_target(self, msg):
        """Solve IK for the requested point and start moving, or explain why not."""
        frame_id = msg.header.frame_id
        if frame_id and frame_id != BASE_FRAME:
            self.get_logger().warning(
                "target is stamped in frame '{}'; this node only understands '{}'. "
                'Ignoring it - transform the point first.'.format(frame_id, BASE_FRAME))
            self._done.publish(Bool(data=False))
            return

        target = (msg.point.x, msg.point.y, msg.point.z)
        try:
            wrist = wrist_from_command(self._wrist, self._position[3])
            solution = solve(target, current_q=self._position, wrist=wrist,
                             frame=self._frame)
        except IKError as exc:
            self.get_logger().error('cannot reach ({:.4f}, {:.4f}, {:.4f}): {}'.format(
                *target, exc))
            self._done.publish(Bool(data=False))
            return
        except ValueError as exc:
            self.get_logger().error('bad request: {}'.format(exc))
            self._done.publish(Bool(data=False))
            return

        self._goal = list(solution.q)
        self._moving = True
        travel = max(abs(a - b) for a, b in zip(self._goal, self._position))
        self.get_logger().info(
            'target ({:.4f}, {:.4f}, {:.4f}) -> {} elbow-{}, q = [{}] deg, '
            'largest joint move {:.1f} deg, about {:.1f} s'.format(
                *target, solution.reach, solution.elbow,
                ', '.join('{:.2f}'.format(math.degrees(v)) for v in self._goal),
                math.degrees(travel),
                travel / (MAX_JOINT_SPEED * self._scale)))

    def _tick(self):
        """Step every joint toward the goal and publish where the arm is."""
        if self._moving:
            remaining = 0.0
            for i, goal in enumerate(self._goal):
                error = goal - self._position[i]
                if abs(error) <= self._step:
                    # Snap, rather than adding the remainder. In floating point
                    # a + (b - a) is not always exactly b, so adding would
                    # leave a sliver of error and "arrived" would never fire.
                    self._position[i] = goal
                else:
                    self._position[i] += self._step if error > 0.0 else -self._step
                remaining = max(remaining, abs(goal - self._position[i]))
            if remaining == 0.0:
                self._moving = False
                reached = frame_position(self._position, self._frame)
                self.get_logger().info(
                    'arrived: {} at ({:.6f}, {:.6f}, {:.6f}) m'.format(
                        self._frame, *reached))
                self._done.publish(Bool(data=True))

        state = JointState()
        state.header.stamp = self.get_clock().now().to_msg()
        state.name = list(JOINT_NAMES)
        state.position = list(self._position)
        self._states.publish(state)


def main(args=None):
    """Run the move-to-point node until Ctrl+C."""
    rclpy.init(args=args)
    node = MoveToPoint()
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
