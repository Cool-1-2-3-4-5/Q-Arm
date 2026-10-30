"""Validate the Python FK against the TF tree that ROS itself computes.

WHY THIS EXISTS
---------------
``forward_kinematics.py`` and ``qarm.urdf.xacro`` are two independent
descriptions of the same arm. Either one could be wrong. Comparing them against
each other is the cheapest real test available: if the hand-derived DH chain and
the URDF that robot_state_publisher walks produce the same pose for every joint
vector we throw at them, then either both are right or both are wrong in exactly
the same way - and they were written by different means, so the second is
unlikely.

HOW IT WORKS (all headless - no RViz, no joint_state_publisher_gui)
-------------------------------------------------------------------
1. Expand the Xacro to a URDF string, in-process via the ``xacro`` module.
2. Start ``robot_state_publisher`` as a subprocess with that URDF. Its whole job
   is: subscribe to ``/joint_states``, walk the URDF, publish ``/tf``.
3. From a small rclpy node, publish a ``sensor_msgs/JointState`` holding the
   joint vector under test, stamped with the current time.
4. Look up ``base_link -> end_effector`` from a ``tf2_ros.Buffer`` AT THAT EXACT
   STAMP. Asking for the stamp rather than "latest" is what guarantees we are
   reading the pose for the q we just sent, and not a leftover from the previous
   row.
5. Compare against ``forward_kinematics(q)``.

Translations are compared directly. Rotations are compared as quaternions, with
the sign ambiguity handled: a quaternion and its negation name the SAME
rotation (the unit quaternions double-cover SO(3)), so q and -q must both count
as a match.
"""

import math
import os
import random
import subprocess
import sys
import tempfile
import time

from ament_index_python.packages import get_package_prefix, get_package_share_directory

import numpy as np

import rclpy
from rclpy.time import Time

from sensor_msgs.msg import JointState

import tf2_ros

import xacro

import yaml

from qarm_kinematics.forward_kinematics import forward_kinematics, position
from qarm_kinematics.robot_params import (
    BASE_FRAME,
    BETA,
    EE_FRAME,
    ELBOW_STRAIGHT,
    JOINT_LIMITS,
    JOINT_NAMES,
    TIP_FRAME,
)

#: Fully extended reach poses, elbow dead straight (q3 = beta - 90 deg).
#: Horizontal: q2 = 90 deg - beta levels the effective upper arm.
#: Vertical:   q2 = -beta cancels the kink and stands the chain upright.
REACH_HORIZONTAL = [0.0, math.pi / 2.0 - BETA, ELBOW_STRAIGHT, 0.0]
REACH_VERTICAL = [0.0, -BETA, ELBOW_STRAIGHT, 0.0]

#: Agreement we demand. 1e-6 m is a micron - far tighter than the arm's own
#: repeatability, but both sides are doing the same arithmetic in double
#: precision, so anything above rounding noise means a genuine modelling
#: disagreement.
TOLERANCE = 1e-6

#: Seconds to wait for robot_state_publisher to come up and answer the first
#: lookup. Generous because node discovery over DDS is not instant.
STARTUP_TIMEOUT = 30.0

#: Seconds to wait for each subsequent transform.
LOOKUP_TIMEOUT = 5.0


def build_urdf_string():
    """Expand ``qarm.urdf.xacro`` into a URDF string.

    Prefers the installed share directory (what a running system would use) and
    falls back to the source tree, so the script still works before a build.
    """
    try:
        share = get_package_share_directory('qarm_description')
        path = os.path.join(share, 'urdf', 'qarm.urdf.xacro')
    except Exception:       # pragma: no cover - only hit on a broken install
        path = None

    if path is None or not os.path.exists(path):
        here = os.path.dirname(os.path.abspath(__file__))
        path = os.path.normpath(os.path.join(
            here, '..', '..', 'qarm_description', 'urdf', 'qarm.urdf.xacro'))

    return xacro.process_file(path).toxml()


def test_poses():
    """Return the list of ``(label, q)`` cases to check.

    Coverage rationale:
      * all zeros - catches a wrong constant offset, which a moving-joint test
        can mask.
      * one joint at a time - isolates which row of the DH table is wrong.
        If only the "joint_3 alone" row fails, row 3 is the suspect.
      * the three ``q4=...`` rows share q1..q3 and differ only in the wrist
        roll; they feed the explicit q4-invariance check.
      * fully extended vertical / horizontal - the reach poses quoted in the
        manual, so a failure here is directly comparable to the datasheet.
      * random in-limit vectors from a FIXED seed - broad coverage that is still
        reproducible, so a failure can be re-run verbatim.
      * at least one pose with all four joints nonzero - cross terms between
        rows only show up when everything moves at once.
    """
    cases = [
        ('all zeros', [0.0, 0.0, 0.0, 0.0]),          # the manual's home pose
        ('joint_1 alone', [0.6, 0.0, 0.0, 0.0]),
        ('joint_2 alone', [0.0, 0.6, 0.0, 0.0]),
        ('joint_3 alone', [0.0, 0.0, -0.8, 0.0]),
        ('joint_4 alone', [0.0, 0.0, 0.0, 1.2]),
        ('all four nonzero', [0.5, -0.4, 0.6, -1.0]),
        ('extended vertical', REACH_VERTICAL),
        ('extended horizontal', REACH_HORIZONTAL),
        ('q4=-1.5', [0.4, -0.3, 0.5, -1.5]),
        ('q4=0.0', [0.4, -0.3, 0.5, 0.0]),
        ('q4=+1.5', [0.4, -0.3, 0.5, 1.5]),
    ]

    # Fixed seed: same five "random" vectors every run, so results are
    # reproducible and a regression can be re-created exactly.
    rng = np.random.default_rng(20250101)
    for n in range(5):
        q = [float(rng.uniform(lo, hi)) for lo, hi in JOINT_LIMITS]
        cases.append(('random #{}'.format(n + 1), q))

    return cases


def quaternion_from_matrix(transform):
    """Convert the rotation block of a 4x4 transform to ``[x, y, z, w]``.

    Shepperd's method: the four candidate formulas each divide by a different
    quaternion component, so we pick the branch whose divisor is largest. A
    single formula would lose precision (or divide by zero) whenever its
    component happens to be near zero.
    """
    m = np.asarray(transform, dtype=float)[:3, :3]
    trace = m[0, 0] + m[1, 1] + m[2, 2]

    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (m[2, 1] - m[1, 2]) / s
        y = (m[0, 2] - m[2, 0]) / s
        z = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s

    return np.array([x, y, z, w])


def quaternion_error(a, b):
    """Return the distance between two quaternions, immune to the q/-q ambiguity.

    Rotating by q and by -q leave a body in an identical orientation, so a
    naive component-wise difference can report a huge error for two identical
    rotations. Taking the smaller of ``|a - b|`` and ``|a + b|`` removes that.
    """
    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float)
    return float(min(np.linalg.norm(a - b), np.linalg.norm(a + b)))


class _TfProbe:
    """rclpy node plus a robot_state_publisher subprocess, as one object."""

    def __init__(self):
        """Start robot_state_publisher and set up the node, publisher and buffer."""
        # Isolate this run on its own DDS domain so a stray node elsewhere on
        # the machine (or a parallel colcon test) cannot feed us someone
        # else's /tf.
        os.environ.setdefault('ROS_LOCALHOST_ONLY', '1')
        os.environ['ROS_DOMAIN_ID'] = str(random.randint(80, 120))

        urdf = build_urdf_string()
        exe = os.path.join(
            get_package_prefix('robot_state_publisher'),
            'lib', 'robot_state_publisher', 'robot_state_publisher')

        # The URDF goes in via a params file rather than "-p name:=value".
        # A command-line parameter is parsed as inline YAML, and a multi-line
        # XML document is not valid inline YAML, so that route silently fails
        # to start the node.
        self._params = tempfile.NamedTemporaryFile(
            mode='w', suffix='.yaml', prefix='qarm_rsp_', delete=False)
        yaml.safe_dump(
            {'/**': {'ros__parameters': {'robot_description': urdf}}},
            self._params, default_flow_style=False)
        self._params.close()

        self._proc = subprocess.Popen(
            [exe, '--ros-args', '--params-file', self._params.name],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            env=os.environ.copy())

        rclpy.init()
        self.node = rclpy.create_node('qarm_fk_validator')
        self._pub = self.node.create_publisher(JointState, '/joint_states', 10)
        self._buffer = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._buffer, self.node)

    def lookup(self, q, child_frame=EE_FRAME, timeout=LOOKUP_TIMEOUT):
        """Publish ``q`` and return the TF transform of ``child_frame`` for it.

        Avoiding a stale read is the whole difficulty here. The guard is a
        timestamp fence: note the time BEFORE the first publish, then keep
        publishing this q (with a fresh stamp each time, because
        robot_state_publisher discards joint states that are not newer than the
        last one it saw) and only accept a transform whose own stamp is at or
        after the fence. Every publish inside the loop carries the same q, so
        any transform past the fence must be the one for this q and cannot be a
        leftover from the previous row.
        """
        msg = JointState()
        msg.name = list(JOINT_NAMES)
        msg.position = [float(v) for v in q]

        fence = self.node.get_clock().now()
        deadline = time.time() + timeout
        while time.time() < deadline:
            msg.header.stamp = self.node.get_clock().now().to_msg()
            self._pub.publish(msg)
            rclpy.spin_once(self.node, timeout_sec=0.05)
            try:
                # Time() means "latest available", not "time zero".
                found = self._buffer.lookup_transform(BASE_FRAME, child_frame, Time())
            except tf2_ros.TransformException:
                continue
            if Time.from_msg(found.header.stamp) >= fence:
                return found

        raise RuntimeError(
            'timed out after {:.1f}s waiting for {} -> {}. '
            'robot_state_publisher exit code: {} (None means still running, so '
            'the problem is discovery or frame names, not startup).'.format(
                timeout, BASE_FRAME, child_frame, self._proc.poll()))

    def close(self):
        """Tear down the node and stop robot_state_publisher."""
        try:
            self.node.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()
            self._proc.terminate()
            try:
                self._proc.wait(timeout=5.0)
            except subprocess.TimeoutExpired:      # pragma: no cover
                self._proc.kill()
            try:
                os.unlink(self._params.name)
            except OSError:                        # pragma: no cover
                pass


def run_validation(verbose=True, tolerance=TOLERANCE):
    """Compare FK against TF for every pose in :func:`test_poses`.

    :returns: ``(rows, extras)``. ``rows`` is a list of dicts with keys
        ``label``, ``q``, ``fk``, ``tf``, ``pos_error``, ``quat_error``, ``ok``.
        ``extras`` holds the derived checks: q4 invariance and the two reach
        numbers.
    """
    probe = _TfProbe()
    rows = []
    extras = {}
    try:
        # First lookup gets the long timeout: it also covers node discovery.
        first_timeout = STARTUP_TIMEOUT
        for label, q in test_poses():
            tf_msg = probe.lookup(q, timeout=first_timeout)
            first_timeout = LOOKUP_TIMEOUT

            t = tf_msg.transform.translation
            r = tf_msg.transform.rotation
            tf_pos = np.array([t.x, t.y, t.z])
            tf_quat = np.array([r.x, r.y, r.z, r.w])

            # check_limits=False: a few deliberate cases (the extended-reach
            # poses) sit right on a limit, and floating point can push them a
            # hair outside. The URDF clamps nothing, so TF will still answer.
            fk = forward_kinematics(q, check_limits=False)
            fk_pos = position(fk)
            fk_quat = quaternion_from_matrix(fk)

            pos_error = float(np.linalg.norm(fk_pos - tf_pos))
            quat_err = quaternion_error(fk_quat, tf_quat)
            rows.append({
                'label': label,
                'q': [float(v) for v in q],
                'fk': fk_pos,
                'tf': tf_pos,
                'pos_error': pos_error,
                'quat_error': quat_err,
                'ok': pos_error <= tolerance and quat_err <= tolerance,
            })

        # ------------------------------------------------------------------
        # EXPLICIT q4 INVARIANCE CHECK.
        # joint_4 rolls about an axis that runs straight down the forearm, and
        # the end effector sits ON that axis. Spinning about a line you are
        # standing on cannot move you. So all three q4=... rows must share one
        # position, in FK and in TF alike. This is the invariant the future IK
        # relies on to split "position from q1..q3" off "orientation from q4";
        # if it ever breaks, the IK decomposition breaks with it.
        # ------------------------------------------------------------------
        q4_rows = [row for row in rows if row['label'].startswith('q4=')]
        fk_spread = max(float(np.linalg.norm(a['fk'] - b['fk']))
                        for a in q4_rows for b in q4_rows)
        tf_spread = max(float(np.linalg.norm(a['tf'] - b['tf']))
                        for a in q4_rows for b in q4_rows)
        extras['q4_fk_spread'] = fk_spread
        extras['q4_tf_spread'] = tf_spread
        extras['q4_invariant'] = fk_spread <= tolerance and tf_spread <= tolerance

        # ------------------------------------------------------------------
        # REACH CHECK against the manual's quoted net reach.
        # Horizontal: shoulder pitched so the effective upper arm is level,
        # i.e. q2 = 90 deg - beta, elbow straight. Vertical: q2 = -beta, which
        # cancels the structural kink and stands the whole chain upright.
        # "Elbow straight" is q3 = ELBOW_STRAIGHT = beta - 90 deg, because the
        # elbow's zero is the manual's home pose (forearm horizontal).
        # ------------------------------------------------------------------
        horiz_q = REACH_HORIZONTAL
        vert_q = REACH_VERTICAL
        ee_h = probe.lookup(horiz_q, EE_FRAME)
        tip_h = probe.lookup(horiz_q, TIP_FRAME)
        ee_v = probe.lookup(vert_q, EE_FRAME)
        tip_v = probe.lookup(vert_q, TIP_FRAME)
        extras['reach_h_ee'] = math.hypot(ee_h.transform.translation.x,
                                          ee_h.transform.translation.y)
        extras['reach_h_tip'] = math.hypot(tip_h.transform.translation.x,
                                           tip_h.transform.translation.y)
        extras['reach_v_ee'] = ee_v.transform.translation.z
        extras['reach_v_tip'] = tip_v.transform.translation.z
    finally:
        probe.close()

    if verbose:
        print(format_table(rows, extras, tolerance))

    return rows, extras


def format_table(rows, extras, tolerance=TOLERANCE):
    """Render the comparison as a readable fixed-width table."""
    out = []
    out.append('FK (python, DH)  vs  TF (robot_state_publisher, URDF)')
    out.append('tolerance: {:g} m / {:g} quaternion units'.format(tolerance, tolerance))
    out.append('')
    header = ('{:<20s} {:<34s} {:<26s} {:<26s} {:>10s} {:>10s}  {}'.format(
        'case', 'q [deg]', 'FK xyz [m]', 'TF xyz [m]', 'pos err', 'quat err', 'result'))
    out.append(header)
    out.append('-' * len(header))
    for row in rows:
        out.append('{:<20s} {:<34s} {:<26s} {:<26s} {:>10.2e} {:>10.2e}  {}'.format(
            row['label'],
            ' '.join('{:>7.2f}'.format(math.degrees(v)) for v in row['q']),
            ' '.join('{:>8.5f}'.format(v) for v in row['fk']),
            ' '.join('{:>8.5f}'.format(v) for v in row['tf']),
            row['pos_error'], row['quat_error'],
            'PASS' if row['ok'] else 'FAIL'))
    out.append('')
    out.append('q4 invariance (position spread across q4 = -1.5, 0, +1.5 rad)')
    out.append('  FK spread {:.2e} m   TF spread {:.2e} m   {}'.format(
        extras['q4_fk_spread'], extras['q4_tf_spread'],
        'PASS' if extras['q4_invariant'] else 'FAIL'))
    out.append('')
    out.append('reach, measured from TF')
    out.append('  horizontal  end_effector {:.6f} m   gripper_tip {:.6f} m'.format(
        extras['reach_h_ee'], extras['reach_h_tip']))
    out.append('  vertical    end_effector {:.6f} m   gripper_tip {:.6f} m'.format(
        extras['reach_v_ee'], extras['reach_v_tip']))
    out.append('  manual Section D: 0.7536 m / 0.8936 m (end_effector)')
    out.append('  manual Section B: 0.800 m / 0.940 m (read as fingertip, gripper_tip)')
    out.append('')
    failures = [row['label'] for row in rows if not row['ok']]
    if failures or not extras['q4_invariant']:
        out.append('RESULT: FAIL ({})'.format(', '.join(failures) or 'q4 invariance'))
    else:
        out.append('RESULT: all {} rows PASS'.format(len(rows)))
    return '\n'.join(out)


def main(argv=None):
    """Console entry point: run the validation and exit non-zero on any failure."""
    del argv
    rows, extras = run_validation(verbose=True)
    ok = all(row['ok'] for row in rows) and extras['q4_invariant']
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
