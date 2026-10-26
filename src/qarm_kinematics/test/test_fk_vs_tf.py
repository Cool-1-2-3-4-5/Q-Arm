"""FK vs TF regression test.

This is the guard that keeps ``robot_params.py`` and ``qarm.urdf.xacro`` honest.
They hold the geometry twice, independently; if anyone edits one and forgets the
other, this test fails. It runs fully headless - robot_state_publisher and a
tf2 listener only, no RViz and no joint_state_publisher_gui.
"""

import pytest

from qarm_kinematics.validate_fk_vs_tf import TOLERANCE, format_table, run_validation


@pytest.fixture(scope='module')
def validation():
    """Run the whole FK-vs-TF sweep once and share it across the assertions.

    Module scope on purpose: starting robot_state_publisher and waiting for DDS
    discovery costs seconds, and nothing here mutates the result.
    """
    rows, extras = run_validation(verbose=False)
    return rows, extras


def test_every_pose_matches_tf(validation):
    """Translation and rotation must agree with TF to within the tolerance."""
    rows, extras = validation
    failures = [row for row in rows if not row['ok']]
    assert not failures, 'FK disagrees with TF:\n' + format_table(rows, extras)


def test_at_least_the_required_cases_are_covered(validation):
    """Guard against someone quietly deleting coverage from test_poses()."""
    rows, _ = validation
    labels = {row['label'] for row in rows}
    assert 'all zeros' in labels
    assert {'joint_1 alone', 'joint_2 alone', 'joint_3 alone', 'joint_4 alone'} <= labels
    assert 'all four nonzero' in labels
    assert len([lbl for lbl in labels if lbl.startswith('random #')]) >= 5


def test_q4_does_not_change_position_in_tf(validation):
    """The q4 invariance must hold in the TF tree too, not just in our maths."""
    _, extras = validation
    assert extras['q4_fk_spread'] <= TOLERANCE
    assert extras['q4_tf_spread'] <= TOLERANCE


def test_reach_matches_the_manual(validation):
    """TF must reproduce 0.753553/0.893553 to end_effector and 0.800/0.940 to the tip."""
    _, extras = validation
    assert extras['reach_h_ee'] == pytest.approx(0.753553, abs=1e-6)
    assert extras['reach_v_ee'] == pytest.approx(0.893553, abs=1e-6)
    assert extras['reach_h_tip'] == pytest.approx(0.800, abs=5e-6)
    assert extras['reach_v_tip'] == pytest.approx(0.940, abs=5e-6)
