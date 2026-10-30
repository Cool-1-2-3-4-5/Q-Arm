"""Analytical (closed-form) inverse kinematics for the QArm.

WHAT INVERSE KINEMATICS IS
--------------------------
Forward kinematics answers "given the joint angles, where is the gripper?" and
always has exactly one answer. Inverse kinematics answers the reverse question,
"what joint angles put the gripper THERE?", and can have several answers, or
none. This file computes all of them with a formula (no iteration), labels
them, and picks one.

WHY IT SPLITS INTO TWO EASY PIECES
----------------------------------
The wrist roll (joint 4) spins about the forearm's own axis, and the end
effector sits on that axis, so q4 cannot move the end effector (asserted in the
FK tests). Position depends on q1, q2, q3 only:

    target position (x, y, z)  ->  q1, q2, q3    solved below
    wrist angle                ->  q4            passed straight through

STEP 1 - THE BASE ANGLE q1
--------------------------
The shoulder and elbow both pitch about horizontal axes, so the whole arm always
lies in ONE vertical plane through the base axis. The base turns that plane to
face the target:

    q1 = atan2(y, x)

and the rest is 2D, in that plane:

    r = horizontal distance from the base axis = sqrt(x^2 + y^2)
    h = height above the shoulder axis          = z - lambda_1

If x = y = 0 (target straight above the base) atan2 has no answer: every base
angle works, because the arm just spins about the target. We then keep the
current base angle, so the base does not move for nothing.

STEP 2 - A TWO-LINK ARM IN THE PLANE
------------------------------------
In the plane the arm is two straight links (the lambda lengths of the DH table):

    upper arm  lambda_2 = 0.353553 m   shoulder axis -> elbow axis
    forearm    lambda_3 = 0.400000 m   elbow axis -> end_effector
               (+ L_gripper = 0.046447 m if targeting gripper_tip)

Measure every direction as an angle FROM VERTICAL, positive toward +r:

    phi_2 = direction of the upper arm = q2 + beta  (beta = kink, DH row 2)
    gamma = elbow bend away from straight          (0 = arm dead straight)

The tip is then at

    r = lambda_2 sin(phi_2) + lambda_3 sin(phi_2 + gamma)
    h = lambda_2 cos(phi_2) + lambda_3 cos(phi_2 + gamma)

The distance D from shoulder to target only depends on the bend (law of cosines
on the shoulder-elbow-target triangle):

    D^2 = lambda_2^2 + lambda_3^2 + 2 lambda_2 lambda_3 cos(gamma)
    cos(gamma) = (D^2 - lambda_2^2 - lambda_3^2) / (2 lambda_2 lambda_3)

so gamma = +acos(...) or -acos(...): the elbow can sit on either side of the
shoulder-target line. If |cos(gamma)| > 1 no triangle exists: the target is
TOO FAR (D > lambda_2 + lambda_3) or TOO CLOSE (D < |lambda_3 - lambda_2|).

With gamma known, the upper arm points "at the target, minus the angle the bent
forearm adds":

    phi_2 = atan2(r, h) - atan2(lambda_3 sin(gamma), lambda_2 + lambda_3 cos(gamma))

STEP 3 - BACK TO JOINT ANGLES
-----------------------------
    q2 = phi_2 - beta                 undo the kink (DH row 2)
    q3 = gamma + ELBOW_STRAIGHT       bend-from-straight -> joint angle, because
                                      the elbow's zero is the home pose and the
                                      arm is straight at ELBOW_STRAIGHT (DH row 3)

THE TWO FAMILIES: FACING THE TARGET, AND REACHING BACKWARD
----------------------------------------------------------
The base can also turn the OTHER way, q1 + 180 deg, and lean back over the
shoulder; in the plane that is r = -sqrt(x^2 + y^2). Both families are
computed and the joint limits decide which survive.

Geometric fact worth knowing: because the shoulder sits ON the base axis, the
"backward" solution with bend +gamma is physically the SAME arm shape as the
facing solution with bend -gamma (mirror the plane, flip the base). So there
are really two arm shapes - elbow up and elbow down - each reachable two ways.
The QArm elbow cannot bend more than 13.13 deg past straight in the negative
direction, so the facing family is almost always elbow-up only, and the
backward family is how the arm gets the other shape. Labels are physical: the
shape whose elbow is HIGHER is "up".

PICKING ONE SOLUTION
--------------------
:func:`solve` keeps the in-limit solutions and picks the one that is the
shortest move from the current pose. All four joints share the same maximum
speed (pi/2 rad/s, Table 7), so move time is set by the joint with the
furthest to go; "shortest" is therefore the smallest largest-single-joint
change, max |dq|. Ties (within 1e-9 rad) go to elbow up, then to facing.

UNREACHABLE TARGETS - never a silent NaN
----------------------------------------
    TargetTooFarError     beyond lambda_2 + lambda_3 from the shoulder
    TargetTooCloseError   inside |lambda_3 - lambda_2| of the shoulder
    JointLimitError       geometry works, but every solution breaks a limit

All three subclass IKError (itself a ValueError), so callers can catch one
specific case or all of them. Not checked: collisions with the table or the
arm's own base - there is no collision model yet.
"""

import collections
import math
import sys

from qarm_kinematics.forward_kinematics import (
    forward_kinematics,
    gripper_tip_pose,
    position,
    validate_joint_vector,
)
from qarm_kinematics.robot_params import (
    BETA,
    EE_FRAME,
    ELBOW_STRAIGHT,
    JOINT_LIMITS,
    JOINT_NAMES,
    LAMBDA_1,
    LAMBDA_2,
    LAMBDA_3,
    L_GRIPPER,
    TIP_FRAME,
)

#: Numerical slack for "exactly on the boundary" cases: a target computed by FK
#: at full reach lands within ~1e-16 of the limit, on either side.
EPS = 1e-9

#: Seen from BEHIND the wrist, looking out along the forearm toward the
#: fingertips, a positive q4 (right-hand rule about the tool axis) turns the
#: gripper CLOCKWISE. Flip to -1.0 if the hardware turns out to disagree.
WRIST_CLOCKWISE_SIGN = 1.0

#: One IK answer. ``q`` is the four joint angles in radians, ``reach`` is
#: 'facing' or 'backward', ``elbow`` is 'up', 'down' or 'straight'.
IKSolution = collections.namedtuple('IKSolution', ['q', 'reach', 'elbow'])


class IKError(ValueError):
    """Base class: the target cannot be reached."""


class TargetTooFarError(IKError):
    """The target is further from the shoulder than the stretched-out arm."""


class TargetTooCloseError(IKError):
    """The target is inside the small dead zone around the shoulder."""


class JointLimitError(IKError):
    """The geometry works, but every solution violates a joint limit.

    ``candidates`` holds every geometric solution (as IKSolution) and
    ``violations`` the matching lists of joint names that are out of range.
    """

    def __init__(self, message, candidates=(), violations=()):
        """Store the rejected candidates alongside the message."""
        super().__init__(message)
        self.candidates = list(candidates)
        self.violations = list(violations)


def tool_length(frame=EE_FRAME):
    """Return the elbow-to-target length for ``frame``.

    end_effector: lambda_3 = L4 + L5. gripper_tip: L_gripper further along the
    same tool axis, so the forearm is simply that much longer.
    """
    if frame == EE_FRAME:
        return LAMBDA_3
    if frame == TIP_FRAME:
        return LAMBDA_3 + L_GRIPPER
    raise ValueError('frame must be {!r} or {!r}, got {!r}'.format(EE_FRAME, TIP_FRAME, frame))


def wrap(angle):
    """Wrap an angle into (-pi, pi]. Same rotation, unique representation."""
    wrapped = math.atan2(math.sin(angle), math.cos(angle))
    return math.pi if wrapped == -math.pi else wrapped


def limit_violations(q):
    """Return the names of the joints in ``q`` that are outside their limits."""
    return [name for name, value, (lo, hi) in zip(JOINT_NAMES, q, JOINT_LIMITS)
            if not (lo - EPS <= value <= hi + EPS)]


def wrist_from_command(command, current_q4=0.0):
    """Turn a user wrist command into a joint angle q4 in radians.

    ``'N/A'`` (or None / empty): keep ``current_q4``.
    ``'home'``: 0.
    A number of DEGREES in [0, 360]: the gripper's angle measured CLOCKWISE
    from home, seen from behind the wrist (see WRIST_CLOCKWISE_SIGN). The wrist
    can turn at most 160 deg either way, so an angle past 180 is reached the
    short way round: 270 clockwise is the same orientation as 90
    counter-clockwise. 161..199 deg cannot be reached; :func:`solve` reports
    that as a JointLimitError on joint_4.
    """
    if command is None:
        return float(current_q4)
    text = str(command).strip().lower()
    if text in ('', 'n/a', 'na', 'none', 'keep'):
        return float(current_q4)
    if text == 'home':
        return 0.0
    try:
        degrees = float(text)
    except ValueError:
        raise ValueError("wrist must be 'N/A', 'home' or degrees 0-360, got {!r}".format(command))
    if not (math.isfinite(degrees) and 0.0 <= degrees <= 360.0):
        raise ValueError('wrist angle must be between 0 and 360 degrees, got {}'.format(degrees))
    if degrees > 180.0:
        degrees -= 360.0          # e.g. 270 clockwise == 90 counter-clockwise
    return WRIST_CLOCKWISE_SIGN * math.radians(degrees)


def _validate_target(target):
    """Return ``target`` as three finite floats, or raise ValueError."""
    values = [float(v) for v in target]
    if len(values) != 3:
        raise ValueError('target must be (x, y, z), got {} values'.format(len(values)))
    for axis, value in zip('xyz', values):
        if not math.isfinite(value):
            raise ValueError('target {} is not finite: {!r}'.format(axis, value))
    return values


def _planar(r, h, forearm):
    """Return the ``(phi_2, gamma)`` pairs that put a two-link tip at (r, h).

    Assumes the caller has already checked the distance is reachable; cos(gamma)
    is clamped only to absorb rounding right at the boundary.

    "Straight" is decided on cos(gamma), not on gamma: acos is so steep near 1
    that 1e-16 of rounding in a full-reach target turns into gamma ~ 1e-8 rad,
    which would wrongly split one straight arm into two bent ones. Treating
    1 - cos(gamma) < 1e-12 as straight moves the tip by under 1e-12 m.
    """
    cos_gamma = ((r * r + h * h - LAMBDA_2 * LAMBDA_2 - forearm * forearm)
                 / (2.0 * LAMBDA_2 * forearm))
    if cos_gamma >= 1.0 - 1e-12:
        bends = (0.0,)
    else:
        gamma = math.acos(max(-1.0, cos_gamma))
        bends = (gamma, -gamma)
    return [(math.atan2(r, h) - math.atan2(forearm * math.sin(g),
                                           LAMBDA_2 + forearm * math.cos(g)), g)
            for g in bends]


def solve_all(target, q4=0.0, frame=EE_FRAME, base_hint=0.0, check_limits=True):
    """Return every IK solution for ``target``, labelled, as IKSolution tuples.

    :param target: (x, y, z) of ``frame`` in ``base_link``, metres.
    :param q4: wrist joint angle to put in every solution, radians.
    :param frame: ``'end_effector'`` (default) or ``'gripper_tip'``.
    :param base_hint: base angle to use when the target is straight above the
        base, where every base angle works.
    :param check_limits: drop solutions outside the joint limits, and raise
        JointLimitError if that leaves none. False returns all candidates.
    :raises TargetTooFarError, TargetTooCloseError, JointLimitError:
    """
    x, y, z = _validate_target(target)
    q4 = float(q4)
    if not math.isfinite(q4):
        raise ValueError('q4 is not finite: {!r}'.format(q4))
    forearm = tool_length(frame)

    rho = math.hypot(x, y)
    h = z - LAMBDA_1
    distance = math.hypot(rho, h)
    reach_max = LAMBDA_2 + forearm
    reach_min = abs(forearm - LAMBDA_2)
    if distance > reach_max + EPS:
        raise TargetTooFarError(
            'target ({:.4f}, {:.4f}, {:.4f}) is {:.4f} m from the shoulder; the arm '
            'reaches at most {:.4f} m ({})'.format(x, y, z, distance, reach_max, frame))
    if distance < reach_min - EPS:
        raise TargetTooCloseError(
            'target ({:.4f}, {:.4f}, {:.4f}) is {:.4f} m from the shoulder, inside the '
            '{:.4f} m dead zone around it ({})'.format(x, y, z, distance, reach_min, frame))

    base = float(base_hint) if rho < EPS else math.atan2(y, x)

    # Every geometric candidate, both families. Remember the elbow height of
    # each: it is physical, so it is the same for the two representations of
    # one arm shape, and it decides the up/down label.
    raw = []
    for reach, q1, r in (('facing', base, rho), ('backward', base + math.pi, -rho)):
        for phi_2, gamma in _planar(r, h, forearm):
            q = (wrap(q1), wrap(phi_2 - BETA), wrap(gamma + ELBOW_STRAIGHT), q4)
            elbow_height = LAMBDA_1 + LAMBDA_2 * math.cos(phi_2)
            raw.append((q, reach, gamma, elbow_height))

    top = max(height for _, _, _, height in raw)
    candidates = []
    for q, reach, gamma, height in raw:
        if abs(gamma) <= EPS:
            elbow = 'straight'
        else:
            elbow = 'up' if height >= top - EPS else 'down'
        candidates.append(IKSolution(q, reach, elbow))

    if not check_limits:
        return candidates

    valid = [s for s in candidates if not limit_violations(s.q)]
    if not valid:
        violations = [limit_violations(s.q) for s in candidates]
        details = '; '.join(
            '{} elbow-{}: {}'.format(s.reach, s.elbow, ', '.join(v))
            for s, v in zip(candidates, violations))
        raise JointLimitError(
            'target ({:.4f}, {:.4f}, {:.4f}) is within reach, but every solution breaks a '
            'joint limit ({})'.format(x, y, z, details), candidates, violations)
    return valid


def choose(solutions, current_q):
    """Pick the solution that is the shortest move from ``current_q``.

    Shortest = smallest max |dq| over the four joints, since every joint has
    the same top speed and the slowest joint sets the move time. Ties (within
    1e-9 rad) go to elbow up, then to the facing family.
    """
    if not solutions:
        raise ValueError('no solutions to choose from')

    def cost(solution):
        return max(abs(a - b) for a, b in zip(solution.q, current_q))

    best = min(cost(s) for s in solutions)
    tied = [s for s in solutions if cost(s) <= best + EPS]
    return min(tied, key=lambda s: (s.elbow != 'up', s.reach != 'facing'))


def solve(target, current_q=None, wrist=None, frame=EE_FRAME):
    """Return the single best IKSolution for ``target``.

    :param target: (x, y, z) of ``frame`` in ``base_link``, metres.
    :param current_q: the arm's present joint angles (radians). Defaults to the
        home pose. Used to pick the shortest move, and its base angle is kept
        when the target is straight above the base.
    :param wrist: q4 in radians, or None to keep ``current_q``'s wrist angle.
        Use :func:`wrist_from_command` to turn 'N/A' / 'home' / degrees into this.
    :param frame: ``'end_effector'`` (default) or ``'gripper_tip'``.
    :raises TargetTooFarError, TargetTooCloseError, JointLimitError:
    """
    current = [0.0] * 4 if current_q is None else list(
        validate_joint_vector(current_q, check_limits=False))
    q4 = current[3] if wrist is None else float(wrist)
    solutions = solve_all(target, q4=q4, frame=frame, base_hint=current[0])
    return choose(solutions, current)


def frame_position(q, frame=EE_FRAME):
    """Return the (x, y, z) that FK puts ``frame`` at for joint angles ``q``."""
    if frame == TIP_FRAME:
        return position(gripper_tip_pose(q, check_limits=False))
    return position(forward_kinematics(q, check_limits=False))


def _format_q(q):
    """Format joint angles (radians) as degrees for printing."""
    return ' '.join('{:>8.3f}'.format(math.degrees(v)) for v in q)


def main(argv=None):
    """Console entry point: target position -> joint angles.

    ros2 run qarm_kinematics qarm_ik X Y Z [WRIST] [--tip] [--current Q1 Q2 Q3 Q4]

    X Y Z in metres in base_link. WRIST is 'N/A' (keep, the default), 'home', or
    degrees 0-360 clockwise from home. --current gives the present joint angles
    in DEGREES (default: home pose). --tip targets gripper_tip instead of
    end_effector.
    """
    args = list(sys.argv[1:] if argv is None else argv)
    frame = EE_FRAME
    if '--tip' in args:
        args.remove('--tip')
        frame = TIP_FRAME
    current = [0.0] * 4
    if '--current' in args:
        i = args.index('--current')
        try:
            current = [math.radians(float(v)) for v in args[i + 1:i + 5]]
        except ValueError:
            current = []
        if len(current) != 4:
            print('--current needs four joint angles in degrees', file=sys.stderr)
            return 2
        del args[i:i + 5]
    if len(args) not in (3, 4):
        print('usage: qarm_ik X Y Z [WRIST] [--tip] [--current Q1 Q2 Q3 Q4]', file=sys.stderr)
        return 2

    try:
        target = [float(v) for v in args[:3]]
        wrist = wrist_from_command(args[3] if len(args) == 4 else None, current[3])
        everything = solve_all(target, q4=wrist, frame=frame, base_hint=current[0],
                               check_limits=False)
        best = solve(target, current_q=current, wrist=wrist, frame=frame)
    except IKError as exc:
        print('unreachable: {}'.format(exc), file=sys.stderr)
        return 1
    except ValueError as exc:
        print('error: {}'.format(exc), file=sys.stderr)
        return 2

    print('target {} ({:.4f}, {:.4f}, {:.4f}) m'.format(frame, *target))
    print('current  [deg] {}'.format(_format_q(current)))
    print('')
    print('{:<10s} {:<9s} {:<36s} {}'.format('reach', 'elbow', 'q1..q4 [deg]', 'limits'))
    for s in everything:
        bad = limit_violations(s.q)
        status = 'out of range: ' + ', '.join(bad) if bad else 'ok'
        print('{:<10s} {:<9s} {:<36s} {}'.format(s.reach, s.elbow, _format_q(s.q), status))
    print('')
    print('chosen   [deg] {}   ({}, elbow {})'.format(_format_q(best.q), best.reach, best.elbow))
    check = frame_position(best.q, frame)
    print('FK check       ({:.6f}, {:.6f}, {:.6f}) m, error {:.2e} m'.format(
        *check, math.dist(check, target)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
