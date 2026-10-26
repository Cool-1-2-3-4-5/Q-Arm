"""Forward kinematics for the QArm using the standard Denavit-Hartenberg method.

WHAT FORWARD KINEMATICS IS
--------------------------
You give it the four joint angles. It tells you where the gripper ends up.
That is it. (Inverse kinematics is the opposite question - "what angles put the
gripper THERE" - and is deliberately not in this file.)

WHY DENAVIT-HARTENBERG
----------------------
A robot arm is a chain of rigid bodies. Each body's pose relative to the
previous one is a 4x4 homogeneous transform, and the pose of the gripper in the
base frame is just those transforms multiplied together, in order:

    T_base_to_ee = T1 * T2 * T3 * T4

The only question is how to write each T. You could write 6 numbers per link
(x, y, z, roll, pitch, yaw) like a URDF does. Denavit and Hartenberg showed you
only ever need FOUR, provided you attach the frames to the links by a rule
instead of arbitrarily. The rule:

    * z_i  points along the axis of joint i+1  (the thing that rotates)
    * x_i  points along the common normal between z_(i-1) and z_i
           (the shortest line joining the two joint axes)
    * y_i  completes the right-handed set

With frames placed that way, the transform from frame i-1 to frame i is always

    T_i = Rot(z, theta_i) * Trans(z, d_i) * Trans(x, a_i) * Rot(x, alpha_i)

and the four parameters mean, physically:

    theta_i  joint angle       how far joint i has rotated about its own axis
                               (this is the VARIABLE one on a revolute joint)
    d_i      link offset       slide ALONG the joint axis to reach the next
                               link's common normal
    a_i      link length       the actual distance between joint axis i and
                               joint axis i+1 (our lambda values)
    alpha_i  link twist        the angle between joint axis i and axis i+1

Note the strict order: z-things first (theta, d), then x-things (a, alpha).
Swapping them gives a different, wrong transform.

HOW THIS TABLE WAS OBTAINED
---------------------------
Not guessed from a figure. It was DERIVED from the URDF chain by algebra, so it
is provably the same robot. The URDF chain is

    T = Rz(q1) Tz(L1) Ry(q2) T(L3,0,L2) Ry(beta) Ry(q3) Tz(L4) Rz(q4) Tz(L5)

The key step is that the kink offset (L3, 0, L2) is itself a pure rotation of a
pure translation, because (L3, 0, L2) = lambda_2 * (sin beta, 0, cos beta):

    T(L3,0,L2) = Ry(beta) Tz(lambda_2) Ry(-beta)

Substituting that, the Ry(-beta) cancels the structural Ry(beta) that follows
it, and Rz/Tz about the same axis commute, so the chain collapses to

    T = Rz(q1) Tz(lambda_1) Ry(q2 + beta) Tz(lambda_2) Ry(q3) Tz(lambda_3 +
        lambda_4) Rz(q4)

which is exactly four joints acting on three effective lengths - the DH form.
The constant -pi/2 in row 2 and +pi/2 in row 3 are NOT physical; they are the
bookkeeping that converts "pitch about y" into DH's "rotate about z, twist
about x". ``test_dh_matches_urdf_chain`` re-checks this algebra numerically, and
``test_fk_vs_tf.py`` checks it against the real TF tree.
"""

import math
import sys

import numpy as np

from qarm_kinematics.robot_params import (
    BETA,
    EE_FRAME,
    JOINT_LIMITS,
    JOINT_NAMES,
    LAMBDA_1,
    LAMBDA_2,
    L_GRIPPER,
    NUM_JOINTS,
    TOOL_LENGTH,
)

# Names of the frames produced by link_transforms(), in order. Frame i is the
# frame rigidly attached to the link moved by joint i+1; the last one is the
# FK/IK target.
FRAME_NAMES = ('dh_frame_1', 'dh_frame_2', 'dh_frame_3', EE_FRAME)


def dh_transform(theta, d, a, alpha):
    """Build the 4x4 homogeneous transform for one standard DH row.

    Implements, in this exact order,

        T = Rot(z, theta) * Trans(z, d) * Trans(x, a) * Rot(x, alpha)

    Multiplying those four out by hand gives the closed form below. Writing the
    closed form instead of four matrix products is both faster and easier to
    compare against a textbook, where this matrix is printed verbatim.

    The top-left 3x3 block is the rotation; the top-right 3x1 column is the
    translation; the bottom row [0 0 0 1] is what makes a rotation and a
    translation composable with a single matrix multiply.

    :param theta: rotation about the current z axis (rad) - the joint variable.
    :param d: translation along the current z axis (m) - the link offset.
    :param a: translation along the new x axis (m) - the link length.
    :param alpha: rotation about the new x axis (rad) - the link twist.
    :returns: 4x4 numpy array.
    """
    ct, st = math.cos(theta), math.sin(theta)
    ca, sa = math.cos(alpha), math.sin(alpha)
    return np.array([
        [ct, -st * ca, st * sa, a * ct],
        [st, ct * ca, -ct * sa, a * st],
        [0.0, sa, ca, d],
        [0.0, 0.0, 0.0, 1.0],
    ])


def dh_table(q):
    """Return the four standard DH rows ``(theta, d, a, alpha)`` for pose ``q``.

    Row-by-row, what each one is physically and which URDF joint it is:

    Row 1 - base roll, URDF ``joint_1`` (axis +Z of ``base_link``).
        theta = q1          the turntable angle, straight through.
        d     = lambda_1    climb L1 = 0.14 m from the worksurface up to the
                            shoulder axis. A pure slide along the joint axis,
                            which is what d is for.
        a     = 0           the base axis and the shoulder axis INTERSECT (the
                            shoulder sits directly on the column's centreline),
                            and a is the distance between consecutive axes, so
                            it is zero.
        alpha = -pi/2       the base axis is vertical, the shoulder axis is
                            horizontal: a 90 deg twist between them. The sign
                            is negative so the new z lands on +y of the base,
                            which is the direction ``joint_2``'s axis actually
                            points in the URDF. Choosing +pi/2 would land on
                            -y and force a sign flip on q2 later.

    Row 2 - shoulder pitch, URDF ``joint_2`` (axis +Y of ``link_1``).
        theta = q2 + beta - pi/2
                            THREE things superimposed, all rotations about the
                            same shoulder axis so they simply add:
                              q2     the commanded shoulder angle;
                              +beta  the structural 8.13 deg elbow kink. It
                                     appears HERE, once, because tilting the
                                     effective upper arm lambda_2 off the
                                     vertical is exactly what the kink does;
                              -pi/2  frame bookkeeping - rotate x into the
                                     direction the upper arm points at the zero
                                     pose.
        d     = 0           shoulder and elbow axes lie in the same plane; no
                            slide along the shoulder axis is needed.
        a     = lambda_2    the real shoulder-to-elbow distance, 0.353553 m -
                            the HYPOTENUSE sqrt(L2^2 + L3^2), not L2. This is
                            where the kink's length contribution lives.
        alpha = 0           shoulder and elbow axes are PARALLEL (both pitch
                            axes), so there is no twist. This is the classic
                            two-link planar pair.

    Row 3 - elbow pitch, URDF ``joint_3`` (axis +Y of ``link_2``).
        theta = q3 + pi/2   q3 plus bookkeeping. Note there is NO beta here:
                            the kink was already spent in row 2. Adding it
                            again is precisely the double-count bug the old
                            URDF had.
        d     = 0           elbow and wrist axes intersect at the elbow.
        a     = 0           likewise zero distance along the common normal.
        alpha = +pi/2       the elbow axis is perpendicular to the forearm,
                            while the next axis (the wrist ROLL) runs straight
                            down the forearm. 90 deg twist.

    Row 4 - wrist roll, URDF ``joint_4`` (axis +Z of ``link_3``).
        theta = q4          the roll, straight through.
        d     = lambda_3 + lambda_4
                            0.25 + 0.15 = 0.40 m out to ``end_effector``.
                            It is a d (slide along the joint axis) and not an
                            a, because the wrist axis runs ALONG the forearm.
                            That is also the geometric reason q4 cannot move
                            the end effector: rotating about an axis you then
                            travel along leaves you in the same place. The
                            test suite asserts this explicitly.
        a     = 0
        alpha = 0           nothing after the wrist to twist towards.

    :param q: sequence of four joint angles in radians.
    :returns: tuple of four ``(theta, d, a, alpha)`` tuples.
    """
    q1, q2, q3, q4 = (float(v) for v in q)
    return (
        (q1, LAMBDA_1, 0.0, -math.pi / 2.0),
        (q2 + BETA - math.pi / 2.0, 0.0, LAMBDA_2, 0.0),
        (q3 + math.pi / 2.0, 0.0, 0.0, math.pi / 2.0),
        (q4, TOOL_LENGTH, 0.0, 0.0),
    )


def validate_joint_vector(q, check_limits=True):
    """Check ``q`` and return it as a float numpy array of length four.

    Raises ValueError naming the offending joint, so an error message is
    actionable instead of a bare traceback.

    :param q: candidate joint vector.
    :param check_limits: when False, skip the joint-limit range check. Use this
        deliberately to explore poses outside the mechanical range (studying
        workspace boundaries, say). Position checks - length and finiteness -
        are never skippable, because they would produce garbage, not merely an
        unreachable pose.
    """
    arr = np.asarray(q, dtype=float).reshape(-1)
    if arr.size != NUM_JOINTS:
        raise ValueError(
            'expected {} joint angles {}, got {}'.format(
                NUM_JOINTS, list(JOINT_NAMES), arr.size))

    for i, value in enumerate(arr):
        if not math.isfinite(value):
            raise ValueError(
                '{} (index {}) is not finite: {!r}'.format(JOINT_NAMES[i], i, value))

    if check_limits:
        for i, value in enumerate(arr):
            lower, upper = JOINT_LIMITS[i]
            if not (lower <= value <= upper):
                raise ValueError(
                    '{} (index {}) = {:.6f} rad ({:.2f} deg) is outside its limits '
                    '[{:.6f}, {:.6f}] rad ([{:.2f}, {:.2f}] deg). Pass '
                    'check_limits=False to override.'.format(
                        JOINT_NAMES[i], i, value, math.degrees(value),
                        lower, upper, math.degrees(lower), math.degrees(upper)))
    return arr


def link_transforms(q, check_limits=True):
    """Return the cumulative transforms of each DH frame in ``base_link``.

    Element i is ``T_base_to_frame(i+1)``, i.e. the running product
    ``T1 * T2 * ... * T(i+1)``. The last element is the end-effector pose, so
    ``link_transforms(q)[-1]`` and ``forward_kinematics(q)`` agree by
    construction.

    These intermediates are not needed to answer "where is the gripper", but
    they are needed for anything built on top of FK: a geometric Jacobian needs
    each joint's axis and origin in base coordinates, and when FK disagrees with
    TF these let you find the first frame that diverges instead of guessing.

    :param q: four joint angles in radians.
    :param check_limits: see :func:`validate_joint_vector`.
    :returns: list of four 4x4 numpy arrays.
    """
    angles = validate_joint_vector(q, check_limits=check_limits)
    transforms = []
    running = np.eye(4)
    for row in dh_table(angles):
        running = running @ dh_transform(*row)
        transforms.append(running.copy())
    return transforms


def forward_kinematics(q, check_limits=True):
    """Return the 4x4 pose of ``end_effector`` in ``base_link`` for pose ``q``.

    :param q: four joint angles in radians, ordered as
        ``('joint_1', 'joint_2', 'joint_3', 'joint_4')``.
    :param check_limits: see :func:`validate_joint_vector`.
    :returns: 4x4 numpy array.
    """
    return link_transforms(q, check_limits=check_limits)[-1]


def gripper_tip_pose(q, check_limits=True):
    """Return the 4x4 pose of ``gripper_tip`` (the fingertip) in ``base_link``.

    The fingertip sits ``L_GRIPPER`` further along the tool axis (+z of the
    end-effector frame) than ``end_effector``, which is the gripper mounting
    face. Only used for reproducing the manual's quoted reach; the FK/IK target
    stays ``end_effector``.
    """
    tool = forward_kinematics(q, check_limits=check_limits)
    offset = np.eye(4)
    offset[2, 3] = L_GRIPPER
    return tool @ offset


def position(transform):
    """Pull the 3x1 translation (x, y, z) out of a 4x4 homogeneous transform."""
    return np.asarray(transform, dtype=float)[:3, 3].copy()


def rpy_from_transform(transform):
    """Convert the rotation block of a transform to roll/pitch/yaw in radians.

    Uses the same convention as a URDF ``rpy`` attribute: fixed-axis XYZ, i.e.
    the rotation matrix is ``R = Rz(yaw) * Ry(pitch) * Rx(roll)``. Reading the
    angles back out of R is just picking the entries where one angle appears
    alone:

        R[2, 0] = -sin(pitch)                   -> pitch
        R[2, 1], R[2, 2] = cos(pitch) * (sin roll, cos roll) -> roll
        R[0, 0], R[1, 0] = cos(pitch) * (cos yaw, sin yaw)   -> yaw

    When cos(pitch) is ~0 the arm is in gimbal lock: roll and yaw stop being
    separable (only their sum is defined) so we pin roll to 0 and put the whole
    rotation into yaw. That is the conventional choice, not extra information.

    :returns: numpy array ``[roll, pitch, yaw]`` in radians.
    """
    r = np.asarray(transform, dtype=float)[:3, :3]
    sin_pitch = -r[2, 0]
    sin_pitch = min(1.0, max(-1.0, sin_pitch))      # guard against 1+1e-16
    cos_pitch = math.sqrt(max(0.0, 1.0 - sin_pitch * sin_pitch))
    pitch = math.asin(sin_pitch)
    if cos_pitch < 1e-9:
        roll = 0.0
        yaw = math.atan2(-r[0, 1], r[1, 1])
    else:
        roll = math.atan2(r[2, 1], r[2, 2])
        yaw = math.atan2(r[1, 0], r[0, 0])
    return np.array([roll, pitch, yaw])


def format_pose(q, check_limits=True):
    """Return a human-readable multi-line report of the pose produced by ``q``."""
    angles = validate_joint_vector(q, check_limits=check_limits)
    tool = forward_kinematics(angles, check_limits=check_limits)
    tip = gripper_tip_pose(angles, check_limits=check_limits)
    p = position(tool)
    rpy = rpy_from_transform(tool)
    tip_p = position(tip)

    lines = ['joint angles']
    for name, value in zip(JOINT_NAMES, angles):
        lines.append('  {:<8s} {:>9.4f} deg   {:>9.6f} rad'.format(
            name, math.degrees(value), value))
    lines.append('')
    lines.append('end_effector in base_link (gripper mounting face)')
    lines.append('  xyz  {:>10.6f} {:>10.6f} {:>10.6f}  m'.format(*p))
    lines.append('  rpy  {:>10.4f} {:>10.4f} {:>10.4f}  deg'.format(
        *[math.degrees(v) for v in rpy]))
    lines.append('  horizontal radius sqrt(x^2+y^2) = {:.6f} m'.format(
        math.hypot(p[0], p[1])))
    lines.append('')
    lines.append('gripper_tip in base_link (fingertip, reach reference)')
    lines.append('  xyz  {:>10.6f} {:>10.6f} {:>10.6f}  m'.format(*tip_p))
    lines.append('')
    lines.append('transform T_base_end_effector')
    for row in tool:
        lines.append('  [{:>10.6f} {:>10.6f} {:>10.6f} {:>10.6f}]'.format(*row))
    return '\n'.join(lines)


def main(argv=None):
    """Console entry point: four joint angles in DEGREES -> printed pose.

    Degrees rather than radians because this exists for eyeballing results
    against the physical arm, and nobody reads a protractor in radians.

        ros2 run qarm_kinematics qarm_fk 0 0 0 0
        ros2 run qarm_kinematics qarm_fk 0 -8.13 0 0 --no-limits
    """
    argv = list(sys.argv[1:] if argv is None else argv)

    check_limits = True
    for flag in ('--no-limits', '--ignore-limits'):
        if flag in argv:
            argv.remove(flag)
            check_limits = False

    if len(argv) != NUM_JOINTS:
        print('usage: qarm_fk Q1_DEG Q2_DEG Q3_DEG Q4_DEG [--no-limits]',
              file=sys.stderr)
        return 2

    try:
        degrees = [float(v) for v in argv]
    except ValueError:
        print('all four joint angles must be numbers (degrees)', file=sys.stderr)
        return 2

    radians = [math.radians(v) for v in degrees]
    try:
        print(format_pose(radians, check_limits=check_limits))
    except ValueError as exc:
        print('error: {}'.format(exc), file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
