"""Drive the real QArm with OUR inverse kinematics, cross-checked by Quanser's.

Windows or Linux, no ROS. Needs numpy, the Quanser Python API, Quanser
Academic Resources (Documents/Quanser), and this repo's
src/qarm_kinematics/qarm_kinematics folder.

    python ik_test.py             # real arm
    python ik_test.py --dry-run   # no arm: only the checks, nothing moves

Type a target as x,y,z in METRES (end_effector frame, base_link), optionally
followed by a wrist angle in degrees. Before anything moves, OUR answer must
pass every gate:

  1. Quanser's IK (hal QArmUtilities) finds the same joint angles among its
     own solutions, to within 0.5 deg.
  2. Quanser's FK of our joint angles lands within 1 mm of the target.
  3. Our joint angles are inside Quanser's limits (pal QArm LIMITS_MIN/MAX).
  4. You type y.

The arm then moves in steps of at most 5 deg. Every move is logged to
ik_test_log.csv with the target, the measured pose and the error, for the
accuracy table. On start it HOLDS the measured pose (no jump to home) and
opens the gripper. Hold the arm before quitting.
"""

import csv
import math
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(HERE, '..', 'src', 'qarm_kinematics'))  # repo layout
sys.path.append(HERE)                                                 # folder copied here
sys.path.append(os.path.expanduser(os.path.join('~', 'Documents', 'Quanser',
                                                '0_libraries', 'python')))

from hal.products.qarm import QArmUtilities  # noqa: E402
from pal.products.qarm import QArm  # noqa: E402
from qarm_kinematics.inverse_kinematics import IKError, solve  # noqa: E402

AGREE_RAD = math.radians(0.5)
FK_TOL_M = 0.001
STEP_RAD = math.radians(5.0)
LIMIT_TOL = 1e-6          # rad; Quanser's own SLEEP_POSE sits exactly on a limit
LOG = os.path.join(HERE, 'ik_test_log.csv')


def wrap(a):
    """Wrap angle(s) to (-pi, pi]."""
    return (np.asarray(a) + np.pi) % (2.0 * np.pi) - np.pi


def check_target(target, q_now, wrist, util):
    """Run every gate. Return (our joint angles, report lines), or (None, lines)."""
    lines = []
    try:
        ours = np.array(solve(target, current_q=q_now, wrist=wrist).q, dtype=float)
    except (IKError, ValueError) as exc:
        return None, ['OUR IK refused: {}'.format(exc)]
    lines.append('our IK      : [{}] deg'.format(
        ', '.join('{:7.2f}'.format(v) for v in np.degrees(ours))))

    gamma = ours[3]
    phi_all, phi_opt = util.inverse_kinematics(np.array(target, dtype=float), gamma, q_now)
    phi_all = np.asarray(phi_all, dtype=float).reshape(4, -1)
    phi_opt = np.asarray(phi_opt, dtype=float).reshape(4)
    best = math.inf
    for col in phi_all.T:
        if np.all(np.isfinite(col)):
            best = min(best, float(np.max(np.abs(wrap(col - ours)))))
    if np.all(np.isfinite(phi_opt)) and np.any(phi_opt != 0.0):
        lines.append('Quanser IK  : [{}] deg (its pick)'.format(
            ', '.join('{:7.2f}'.format(v) for v in np.degrees(phi_opt))))
    else:
        lines.append('Quanser IK  : no in-limit solution')
    if best > AGREE_RAD:
        lines.append('GATE 1 FAIL : no Quanser solution within 0.5 deg of ours '
                     '(closest {:.2f} deg)'.format(math.degrees(best)))
        return None, lines
    lines.append('gate 1 ok   : Quanser has our solution (diff {:.2e} deg)'.format(
        math.degrees(best)))

    p4, _ = util.forward_kinematics(ours)
    miss = float(np.linalg.norm(np.asarray(p4, dtype=float).reshape(3) - np.array(target)))
    if miss > FK_TOL_M:
        lines.append('GATE 2 FAIL : Quanser FK of our angles misses by {:.1f} mm'.format(
            miss * 1000.0))
        return None, lines
    lines.append('gate 2 ok   : Quanser FK of our angles misses by {:.2e} mm'.format(
        miss * 1000.0))

    low = ours < QArm.LIMITS_MIN - LIMIT_TOL
    high = ours > QArm.LIMITS_MAX + LIMIT_TOL
    if np.any(low | high):
        lines.append('GATE 3 FAIL : outside Quanser limits at joint(s) {}'.format(
            [i + 1 for i in np.nonzero(low | high)[0]]))
        return None, lines
    lines.append('gate 3 ok   : inside Quanser joint limits')
    return ours, lines


def settle(arm, seconds):
    """Keep reading so the arm can move; return the measured joint angles."""
    for _ in range(int(seconds / 0.01)):
        arm.read_std()
        time.sleep(0.01)
    return np.array(arm.measJointPosition[:4], dtype=float)


def move(arm, q_from, q_to, gripper):
    """Step from q_from to q_to, no joint more than STEP_RAD per command."""
    n = max(1, int(math.ceil(float(np.max(np.abs(q_to - q_from))) / STEP_RAD)))
    for k in range(1, n + 1):
        arm.read_write_std(phiCMD=q_from + (q_to - q_from) * k / n, gprCMD=gripper)
        settle(arm, 0.15)
    return settle(arm, 1.5)


def ask_target():
    """Prompt for x,y,z[,wrist]; return (target, wrist rad or None), or None to quit."""
    try:
        s = input('\nx,y,z in metres [,wrist deg]  (blank = quit): ').strip()
    except EOFError:
        return None
    if not s:
        return None
    v = [float(x) for x in s.split(',')]
    if len(v) not in (3, 4):
        raise ValueError('need 3 or 4 numbers, e.g. 0.45,0,0.49')
    return tuple(v[:3]), (math.radians(v[3]) if len(v) == 4 else None)


def main():
    """Hold the arm, then check and run targets typed by the user."""
    dry = '--dry-run' in sys.argv
    util = QArmUtilities()
    new_log = not os.path.exists(LOG)
    log_file = open(LOG, 'a', newline='')
    log = csv.writer(log_file)
    if new_log:
        log.writerow(['time', 'x', 'y', 'z', 'q_cmd_deg', 'q_meas_deg',
                      'x_meas', 'y_meas', 'z_meas', 'error_mm'])

    if dry:
        print('DRY RUN: no arm, nothing moves. Starting from home.')
        arm = None
        q = np.zeros(4)
    else:
        arm = QArm(hardware=1)
        if not arm.status:
            sys.exit('Could not open the QArm. Is it plugged in and powered on?')
        arm.read_std()
        q = np.array(arm.measJointPosition[:4], dtype=float)
    gripper = np.array([0.1])
    try:
        if arm is not None:
            arm.read_write_std(phiCMD=q, gprCMD=gripper)          # hold where it is
            q = settle(arm, 1.0)
        print('Start pose [{}] deg. Home end_effector is 0.45,0,0.49.'.format(
            ', '.join('{:.1f}'.format(v) for v in np.degrees(q))))

        while True:
            try:
                request = ask_target()
            except ValueError as exc:
                print('bad input: {}'.format(exc))
                continue
            if request is None:
                break
            target, wrist = request
            q_cmd, lines = check_target(target, q, wrist, util)
            print('\n'.join(lines))
            if q_cmd is None:
                print('NOT MOVING.')
                continue
            if dry:
                q = q_cmd
                continue
            if input('gate 4      : move the arm? type y: ').strip().lower() != 'y':
                print('NOT MOVING.')
                continue
            q_meas = move(arm, q, q_cmd, gripper)
            p_meas, _ = util.forward_kinematics(q_meas)
            p_meas = np.asarray(p_meas, dtype=float).reshape(3)
            err = float(np.linalg.norm(p_meas - np.array(target))) * 1000.0
            print('measured    : [{}] deg -> ({:.4f}, {:.4f}, {:.4f}) m, '
                  'error {:.1f} mm from encoders'.format(
                      ', '.join('{:.2f}'.format(v) for v in np.degrees(q_meas)),
                      *p_meas, err))
            print('Now measure the real position with a ruler and note it next to this.')
            log.writerow([time.strftime('%H:%M:%S'), *target,
                          ' '.join('{:.3f}'.format(v) for v in np.degrees(q_cmd)),
                          ' '.join('{:.3f}'.format(v) for v in np.degrees(q_meas)),
                          *('{:.5f}'.format(v) for v in p_meas), '{:.2f}'.format(err)])
            log_file.flush()
            q = q_cmd
    finally:
        log_file.close()
        if arm is not None:
            input('HOLD THE ARM, then press Enter to disconnect...')
            arm.terminate()


if __name__ == '__main__':
    main()
