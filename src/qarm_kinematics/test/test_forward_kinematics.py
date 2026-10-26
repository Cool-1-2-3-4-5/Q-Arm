"""Unit tests for the FK maths. No ROS needed - pure numpy."""

import math

import numpy as np

import pytest

from qarm_kinematics.forward_kinematics import (
    dh_table,
    dh_transform,
    forward_kinematics,
    gripper_tip_pose,
    link_transforms,
    position,
    rpy_from_transform,
)
from qarm_kinematics.robot_params import (
    BETA,
    JOINT_LIMITS,
    L1,
    L2,
    L3,
    L4,
    L5,
    LAMBDA_2,
    L_GRIPPER,
)

TOL = 1e-12


def _rz(t):
    c, s = math.cos(t), math.sin(t)
    return np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])


def _ry(t):
    c, s = math.cos(t), math.sin(t)
    return np.array([[c, 0, s, 0], [0, 1, 0, 0], [-s, 0, c, 0], [0, 0, 0, 1]])


def _trans(x, y, z):
    t = np.eye(4)
    t[:3, 3] = (x, y, z)
    return t


def _urdf_chain(q):
    """Multiply out the URDF joint origins directly, with no DH involved.

    This is the independent second opinion. It is a literal transcription of
    the <origin> and <axis> tags in qarm.urdf.xacro:

        base_link -> link_1       Rz(q1)
        link_1    -> link_2       Tz(L1) Ry(q2)
        link_2    -> link_3       T(L3,0,L2) Ry(beta) Ry(q3)
        link_3    -> link_4       Tz(L4) Rz(q4)
        link_4    -> end_effector Tz(L5)

    If this and the DH table agree for arbitrary q, the DH derivation in
    forward_kinematics.py is correct as algebra - independently of whether the
    URDF itself matches the real hardware, which is what the TF test covers.
    """
    q1, q2, q3, q4 = q
    return (_rz(q1) @ _trans(0, 0, L1) @ _ry(q2)
            @ _trans(L3, 0, L2) @ _ry(BETA) @ _ry(q3)
            @ _trans(0, 0, L4) @ _rz(q4) @ _trans(0, 0, L5))


def test_dh_transform_is_identity_for_zero_row():
    """A DH row of all zeros must be the identity: no rotation, no translation."""
    assert np.allclose(dh_transform(0.0, 0.0, 0.0, 0.0), np.eye(4), atol=TOL)


def test_dh_transform_matches_explicit_product():
    """dh_transform's closed form must equal Rz(theta) Tz(d) Tx(a) Rx(alpha)."""
    theta, d, a, alpha = 0.31, 0.07, 0.22, -math.pi / 3.0
    rx = np.eye(4)
    rx[1, 1] = rx[2, 2] = math.cos(alpha)
    rx[1, 2] = -math.sin(alpha)
    rx[2, 1] = math.sin(alpha)
    expected = _rz(theta) @ _trans(0, 0, d) @ _trans(a, 0, 0) @ rx
    assert np.allclose(dh_transform(theta, d, a, alpha), expected, atol=TOL)


def test_dh_matches_urdf_chain():
    """The DH chain and the raw URDF origin chain must agree for every pose."""
    rng = np.random.default_rng(7)
    for _ in range(200):
        q = [float(rng.uniform(lo, hi)) for lo, hi in JOINT_LIMITS]
        assert np.allclose(forward_kinematics(q), _urdf_chain(q), atol=1e-12)


def test_kink_applied_exactly_once():
    """lambda_2 must be the hypotenuse, so the elbow is 0.353553 m from the shoulder.

    Double-applying beta (the original bug) shortened this to 0.3465 m, and
    omitting the kink entirely would give 0.35 m. Both are caught here.
    """
    elbow = position(link_transforms([0.0, 0.0, 0.0, 0.0])[1])
    shoulder = np.array([0.0, 0.0, L1])
    assert np.linalg.norm(elbow - shoulder) == pytest.approx(LAMBDA_2, abs=1e-12)
    assert LAMBDA_2 == pytest.approx(0.3535533905932738, abs=1e-12)
    assert math.degrees(BETA) == pytest.approx(8.13, abs=0.005)


def test_zero_pose():
    """The corrected zero pose. Supersedes the old [0.106, 0.000, 0.879]."""
    p = position(forward_kinematics([0.0, 0.0, 0.0, 0.0]))
    assert p[0] == pytest.approx(0.10656854, abs=1e-7)
    assert p[1] == pytest.approx(0.0, abs=1e-12)
    assert p[2] == pytest.approx(0.88597980, abs=1e-7)

    # At zero the only rotation left in the chain is the structural kink, so
    # the tool is pitched by exactly beta.
    rpy = rpy_from_transform(forward_kinematics([0.0, 0.0, 0.0, 0.0]))
    assert rpy[0] == pytest.approx(0.0, abs=1e-12)
    assert math.degrees(rpy[1]) == pytest.approx(8.1301, abs=1e-3)
    assert rpy[2] == pytest.approx(0.0, abs=1e-12)


def test_q4_does_not_move_the_end_effector():
    """joint_4 rolls along the tool axis, so it can only change orientation.

    This invariant is what lets the future IK solve position from q1..q3 alone
    and orientation from q4 afterwards.
    """
    base = [0.4, -0.3, 0.5]
    reference = position(forward_kinematics(base + [0.0]))
    for q4 in np.linspace(JOINT_LIMITS[3][0], JOINT_LIMITS[3][1], 25):
        moved = forward_kinematics(base + [float(q4)])
        assert np.allclose(position(moved), reference, atol=1e-12)
        # ...and it must genuinely be doing something, else the test is vacuous.
        if abs(q4) > 0.1:
            assert not np.allclose(moved[:3, :3],
                                   forward_kinematics(base + [0.0])[:3, :3],
                                   atol=1e-6)


def test_reach_horizontal_and_vertical():
    """Reproduce the manual's net reach, once the 46.4 mm gripper is included.

    Horizontal: q2 = 90 deg - beta levels the effective upper arm.
    Vertical:   q2 = -beta cancels the kink and stands the chain upright.
    """
    horiz = position(forward_kinematics([0.0, math.pi / 2.0 - BETA, 0.0, 0.0]))
    horiz_tip = position(gripper_tip_pose([0.0, math.pi / 2.0 - BETA, 0.0, 0.0]))
    assert math.hypot(horiz[0], horiz[1]) == pytest.approx(
        LAMBDA_2 + L4 + L5, abs=1e-12)
    assert math.hypot(horiz[0], horiz[1]) == pytest.approx(0.753553, abs=1e-6)
    assert math.hypot(horiz_tip[0], horiz_tip[1]) == pytest.approx(0.800, abs=5e-6)
    assert horiz[2] == pytest.approx(L1, abs=1e-12)

    vert = position(forward_kinematics([0.0, -BETA, 0.0, 0.0]))
    vert_tip = position(gripper_tip_pose([0.0, -BETA, 0.0, 0.0]))
    assert vert[2] == pytest.approx(L1 + LAMBDA_2 + L4 + L5, abs=1e-12)
    assert vert[2] == pytest.approx(0.893553, abs=1e-6)
    assert vert_tip[2] == pytest.approx(0.940, abs=5e-6)

    # The shortfall is the SAME in both directions. That identity is the whole
    # argument for "it is a constant tip extension, not a link-length error".
    assert (0.800 - math.hypot(horiz[0], horiz[1])) == pytest.approx(
        0.940 - vert[2], abs=1e-9)
    assert L_GRIPPER == pytest.approx(0.940 - vert[2], abs=1e-6)


def test_link_transforms_ends_at_the_end_effector():
    """link_transforms' last entry is the same pose forward_kinematics returns."""
    q = [0.2, -0.1, 0.3, 0.4]
    assert np.allclose(link_transforms(q)[-1], forward_kinematics(q), atol=TOL)
    assert len(link_transforms(q)) == 4


def test_wrong_length_raises():
    """A joint vector of the wrong size is a programming error, not a pose."""
    with pytest.raises(ValueError, match='expected 4 joint angles'):
        forward_kinematics([0.0, 0.0, 0.0])


def test_non_finite_raises_naming_the_joint():
    """NaN/inf must be rejected, and the message must say which joint."""
    with pytest.raises(ValueError, match='joint_3'):
        forward_kinematics([0.0, 0.0, float('nan'), 0.0])
    with pytest.raises(ValueError, match='joint_1'):
        forward_kinematics([float('inf'), 0.0, 0.0, 0.0])


def test_out_of_limit_raises_naming_the_joint():
    """Out-of-range angles are rejected by default..."""
    with pytest.raises(ValueError, match='joint_2'):
        forward_kinematics([0.0, math.radians(120.0), 0.0, 0.0])


def test_limit_check_is_opt_out():
    """...but can be waived deliberately to explore outside the joint range."""
    p = position(forward_kinematics([0.0, math.radians(120.0), 0.0, 0.0],
                                    check_limits=False))
    assert np.all(np.isfinite(p))


def test_gripper_tip_is_l_gripper_beyond_the_end_effector():
    """gripper_tip must sit exactly L_GRIPPER along the tool axis."""
    q = [0.3, 0.2, -0.4, 0.9]
    gap = np.linalg.norm(position(gripper_tip_pose(q)) - position(forward_kinematics(q)))
    assert gap == pytest.approx(L_GRIPPER, abs=1e-12)


def test_dh_table_has_no_beta_in_the_elbow_row():
    """Row 3 must not carry beta: the kink belongs to row 2 and only row 2."""
    rows = dh_table([0.0, 0.0, 0.0, 0.0])
    assert rows[1][0] == pytest.approx(BETA - math.pi / 2.0, abs=1e-15)
    assert rows[2][0] == pytest.approx(math.pi / 2.0, abs=1e-15)
    # a_2 is the hypotenuse; nothing else in the table carries a length that
    # could re-introduce L3.
    assert rows[1][2] == pytest.approx(LAMBDA_2, abs=1e-15)
    assert rows[2][2] == pytest.approx(0.0, abs=1e-15)
