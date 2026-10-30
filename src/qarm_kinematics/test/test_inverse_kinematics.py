"""IK tests. Pure numpy, no ROS needed.

Every check in ``validate_ik.CHECKS`` runs as its own test, so
``colcon test`` and ``ros2 run qarm_kinematics qarm_validate_ik`` report the
same thing.
"""

import pytest

from qarm_kinematics.validate_ik import CHECKS


@pytest.mark.parametrize('name, check', CHECKS, ids=[name for name, _ in CHECKS])
def test_ik_check(name, check):
    """Each IK validation check must pass."""
    ok, detail = check()
    assert ok, '{}: {}'.format(name, detail)
