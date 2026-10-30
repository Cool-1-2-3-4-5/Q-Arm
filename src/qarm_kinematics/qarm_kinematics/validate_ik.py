"""IK validation: every check prints as one row of a PASS/FAIL table.

The same checks run under ``colcon test`` (``test/test_inverse_kinematics.py``
parametrizes over :func:`run_checks`), so the printed table and the test suite
cannot disagree.

    ros2 run qarm_kinematics qarm_validate_ik

What "correct" means here, in order of importance:
  * ROUND TRIP: for known joint angles q, IK(FK(q)) must give q back. This is
    the single most important check, because it does not trust any IK formula -
    only FK, which is already validated against the URDF and TF.
  * EVERY returned solution must actually reach the target (FK of it equals the
    target). A "solution" that misses is the main failure mode of IK code.
  * The boundary cases each produce their own, distinguishable result.
"""

import math
import sys

import numpy as np

from qarm_kinematics.inverse_kinematics import (
    JointLimitError,
    TargetTooCloseError,
    TargetTooFarError,
    choose,
    frame_position,
    limit_violations,
    solve,
    solve_all,
    wrist_from_command,
)
from qarm_kinematics.robot_params import (
    BETA,
    EE_FRAME,
    ELBOW_STRAIGHT,
    JOINT_LIMITS,
    LAMBDA_1,
    LAMBDA_2,
    LAMBDA_3,
    TIP_FRAME,
)

#: Joint agreement demanded in the round trip, radians.
JOINT_TOL = 1e-9
#: Position agreement demanded of every solution, metres.
POS_TOL = 1e-9


def _deg(*values):
    """Degrees -> radians, for writing test poses readably."""
    return [math.radians(v) for v in values]


def _matches(q, solutions):
    """Return the smallest max-joint difference between ``q`` and any solution."""
    return min(max(abs(a - b) for a, b in zip(q, s.q)) for s in solutions)


def _grid():
    """In-limit grid of joint vectors, avoiding the exactly-straight elbow."""
    q1s = _deg(-160, -90, 0, 45, 150)
    q2s = _deg(-80, -40, 0, 40, 80)
    q3s = _deg(-90, -50, 0, 40, 70)
    q4s = [0.0, 1.2]
    return [[a, b, c, d] for a in q1s for b in q2s for c in q3s for d in q4s]


def _random(n, seed):
    """``n`` reproducible random in-limit joint vectors."""
    rng = np.random.default_rng(seed)
    return [[float(rng.uniform(lo, hi)) for lo, hi in JOINT_LIMITS] for _ in range(n)]


def _expect(exc_type, func):
    """Run ``func``; return (ok, detail) for 'it raised exc_type'."""
    try:
        func()
    except exc_type as exc:
        return True, '{}: {}'.format(exc_type.__name__, str(exc)[:70])
    except Exception as exc:     # noqa: B902 - report the wrong exception type
        return False, 'raised {} instead: {}'.format(type(exc).__name__, exc)
    return False, 'no exception raised'


def check_round_trip_grid():
    """IK(FK(q)) returns a solution equal to q, for a 250-pose grid."""
    worst, misses = 0.0, 0
    for q in _grid():
        target = frame_position(q)
        err = _matches(q, solve_all(target, q4=q[3], base_hint=q[0]))
        worst = max(worst, err)
        misses += err > JOINT_TOL
    n = len(_grid())
    return misses == 0, '{} poses, {} recovered, worst joint error {:.1e} rad'.format(
        n, n - misses, worst)


def check_every_solution_reaches_target():
    """FK of every returned solution lands on the target (1000 random poses)."""
    worst, count = 0.0, 0
    for q in _random(1000, seed=3):
        target = frame_position(q)
        for s in solve_all(target, q4=q[3], base_hint=q[0]):
            worst = max(worst, math.dist(frame_position(s.q), target))
            count += 1
    return worst <= POS_TOL, '{} solutions from 1000 targets, worst miss {:.1e} m'.format(
        count, worst)


def check_random_round_trip():
    """Random poses come back too, away from the straight-elbow singularity."""
    worst, n = 0.0, 0
    for q in _random(1000, seed=11):
        if abs(q[2] - ELBOW_STRAIGHT) < math.radians(1.0):
            continue       # near-straight: position barely depends on the bend
        worst = max(worst, _matches(q, solve_all(frame_position(q), q4=q[3], base_hint=q[0])))
        n += 1
    return worst <= 1e-8, '{} poses, worst joint error {:.1e} rad'.format(n, worst)


def check_elbow_up_and_down():
    """Both arm shapes reach the same point with different joint angles."""
    q_true = _deg(45, -35, -20, 0)
    target = frame_position(q_true)
    sols = solve_all(target)
    elbows = sorted(s.elbow for s in sols)
    miss = max(math.dist(frame_position(s.q), target) for s in sols)
    spread = max(abs(a - b) for a, b in zip(sols[0].q, sols[-1].q)) if len(sols) > 1 else 0.0
    ok = elbows == ['down', 'up'] and miss <= POS_TOL and spread > 0.1
    return ok, 'elbows {}, both within {:.1e} m, joints differ by up to {:.1f} deg'.format(
        elbows, miss, math.degrees(spread))


def check_exact_max_reach():
    """Reach exactly full stretch: the only solution is the dead-straight arm."""
    q_reach = [0.0, math.pi / 2.0 - BETA, ELBOW_STRAIGHT, 0.0]
    target = frame_position(q_reach)
    best = solve(target)
    err = max(abs(a - b) for a, b in zip(best.q, q_reach))
    ok = best.elbow == 'straight' and err < 1e-6
    return ok, 'reach {:.6f} m -> elbow {}, joint error {:.1e} rad'.format(
        math.hypot(target[0], target[1]), best.elbow, err)


def check_vertical_max_reach():
    """Straight up at full reach: also on the base axis, so base angle is kept."""
    target = (0.0, 0.0, LAMBDA_1 + LAMBDA_2 + LAMBDA_3)
    best = solve(target, current_q=_deg(30, 0, 0, 0))
    miss = math.dist(frame_position(best.q), target)
    ok = abs(best.q[0] - math.radians(30)) < 1e-12 and miss < 1e-6
    return ok, 'z = {:.6f} m -> q1 {:.1f} deg (kept), miss {:.1e} m'.format(
        target[2], math.degrees(best.q[0]), miss)


def check_just_beyond_reach():
    """0.1 mm past full reach raises TargetTooFarError."""
    return _expect(TargetTooFarError, lambda: solve((LAMBDA_2 + LAMBDA_3 + 1e-4, 0.0, LAMBDA_1)))


def check_overhead_keeps_base():
    """Straight above the base: keep the current base angle, do not spin."""
    best = solve((0.0, 0.0, 0.6), current_q=_deg(30, 0, 0, 0))
    miss = math.dist(frame_position(best.q), (0.0, 0.0, 0.6))
    ok = abs(best.q[0] - math.radians(30)) < 1e-12 and miss <= POS_TOL
    return ok, 'current q1 30 deg -> chosen q1 {:.3f} deg, miss {:.1e} m'.format(
        math.degrees(best.q[0]), miss)


def check_too_close():
    """Inside the dead zone around the shoulder raises TargetTooCloseError."""
    return _expect(TargetTooCloseError, lambda: solve((0.0, 0.0, LAMBDA_1 + 0.02)))


def check_out_of_joint_range():
    """Geometrically reachable but needs a fold the elbow cannot do."""
    return _expect(JointLimitError, lambda: solve((0.1, 0.0, LAMBDA_1)))


def check_behind_the_base():
    """Reach behind the base: only the backward family stays inside the limits."""
    q_true = _deg(0, -75, -20, 0)
    target = frame_position(q_true)
    sols = solve_all(target)
    err = _matches(q_true, sols)
    ok = target[0] < 0 and all(s.reach == 'backward' for s in sols) and err <= JOINT_TOL
    return ok, 'target x = {:.4f} m, families {}, joint error {:.1e} rad'.format(
        target[0], sorted({s.reach for s in sols}), err)


def check_gripper_tip_frame():
    """IK on gripper_tip: 200 random round trips, every solution on target."""
    worst = 0.0
    for q in _random(200, seed=5):
        target = frame_position(q, TIP_FRAME)
        for s in solve_all(target, q4=q[3], frame=TIP_FRAME, base_hint=q[0]):
            worst = max(worst, math.dist(frame_position(s.q, TIP_FRAME), target))
    return worst <= POS_TOL, 'worst miss {:.1e} m'.format(worst)


def check_wrist_commands():
    """'N/A' keeps, 'home' is 0, degrees are clockwise, 270 cw == 90 ccw."""
    cases = [('N/A', 0.7, 0.7), ('home', 0.7, 0.0), ('90', 0.0, math.pi / 2.0),
             ('270', 0.0, -math.pi / 2.0), ('0', 0.3, 0.0), ('360', 0.3, 0.0)]
    bad = [c for c in cases if abs(wrist_from_command(c[0], c[1]) - c[2]) > 1e-12]
    ok_180, _ = _expect(JointLimitError, lambda: solve(
        (0.3, 0.0, 0.5), wrist=wrist_from_command('180')))
    ok_bad, _ = _expect(ValueError, lambda: wrist_from_command('400'))
    ok = not bad and ok_180 and ok_bad
    return ok, "{} conversions ok, 180 deg -> JointLimitError, '400' -> ValueError".format(
        len(cases) - len(bad))


def check_shortest_move_and_tie_break():
    """Nearest solution wins; exact ties go to elbow up, then facing."""
    target = frame_position(_deg(45, -35, -20, 0))
    sols = solve_all(target)
    # Standing exactly at a solution, that solution must be the one picked.
    picked = sum(choose(sols, s.q) == s for s in sols)
    # Same joint angles labelled down and up: an exact tie, elbow up must win.
    tie = [sols[0]._replace(elbow='down'), sols[0]._replace(elbow='up')]
    tie_ok = choose(tie, [0.0] * 4).elbow == 'up'
    ok = picked == len(sols) and tie_ok
    return ok, 'nearest-to-current picked {}/{}, exact tie -> elbow up: {}'.format(
        picked, len(sols), tie_ok)


def check_all_solutions_within_limits():
    """solve_all never returns an out-of-limit solution (1000 random targets)."""
    bad = sum(bool(limit_violations(s.q))
              for q in _random(1000, seed=8) for s in solve_all(frame_position(q), q4=q[3]))
    return bad == 0, '{} out-of-limit solutions returned'.format(bad)


CHECKS = [
    ('round trip, grid', check_round_trip_grid),
    ('round trip, random', check_random_round_trip),
    ('every solution on target', check_every_solution_reaches_target),
    ('only in-limit solutions', check_all_solutions_within_limits),
    ('elbow up and down', check_elbow_up_and_down),
    ('behind the base', check_behind_the_base),
    ('exactly at max reach', check_exact_max_reach),
    ('vertical max reach', check_vertical_max_reach),
    ('just beyond reach', check_just_beyond_reach),
    ('straight overhead', check_overhead_keeps_base),
    ('too close', check_too_close),
    ('out of joint range', check_out_of_joint_range),
    ('gripper_tip frame', check_gripper_tip_frame),
    ('wrist commands', check_wrist_commands),
    ('shortest move, tie-break', check_shortest_move_and_tie_break),
]


def run_checks():
    """Run every check; return a list of ``(name, ok, detail)``."""
    rows = []
    for name, func in CHECKS:
        try:
            ok, detail = func()
        except Exception as exc:     # noqa: B902 - a crash is a FAIL, not a stop
            ok, detail = False, 'crashed: {}: {}'.format(type(exc).__name__, exc)
        rows.append((name, ok, detail))
    return rows


def main(argv=None):
    """Print the IK validation table; exit non-zero if any check fails."""
    del argv
    rows = run_checks()
    print('IK validation ({} frame unless noted)'.format(EE_FRAME))
    print('')
    for name, ok, detail in rows:
        print('{:<26s} {:<5s} {}'.format(name, 'PASS' if ok else 'FAIL', detail))
    print('')
    failed = [name for name, ok, _ in rows if not ok]
    print('RESULT: ' + ('FAIL ({})'.format(', '.join(failed)) if failed
                        else 'all {} checks PASS'.format(len(rows))))
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
