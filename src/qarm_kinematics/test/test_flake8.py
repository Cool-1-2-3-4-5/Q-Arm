"""Style check (ament's flake8 configuration)."""

from ament_flake8.main import main_with_errors

import pytest


@pytest.mark.flake8
@pytest.mark.linter
def test_flake8():
    """Fail with the offending lines listed if any file breaks style."""
    rc, errors = main_with_errors(argv=[])
    assert rc == 0, \
        'Found %d code style errors / warnings:\n' % len(errors) + \
        '\n'.join(errors)
