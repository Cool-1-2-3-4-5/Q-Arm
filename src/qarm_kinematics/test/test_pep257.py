"""Docstring check (ament's pep257 configuration)."""

from ament_pep257.main import main

import pytest


@pytest.mark.linter
@pytest.mark.pep257
def test_pep257():
    """Fail if a module, class or public function is missing a docstring.

    D213 is waived. The "ament" preset already ignores D212 ("summary should
    start at the first line"), which leaves D213 ("...should start at the
    second line") in force, so every multi-line docstring in a package that
    explains itself at any length gets flagged. The two rules are mutually
    exclusive by design; we keep the one-line summary style used throughout
    this package and the wider ROS 2 codebase.
    """
    rc = main(argv=['--add-ignore', 'D213'])
    assert rc == 0, 'Found code style errors / warnings'
