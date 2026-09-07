"""Test Python source code for flake8 compliance."""

from pathlib import Path

from ament_flake8.main import main_with_errors
import pytest


@pytest.mark.flake8
@pytest.mark.linter
def test_flake8():
    """Check navigation supervisor source files with flake8."""
    package_root = Path(__file__).resolve().parent.parent

    paths = [
        str(package_root / "navigation_supervisor"),
        str(package_root / "test"),
        str(package_root / "setup.py"),
    ]

    rc, errors = main_with_errors(argv=paths)

    assert (
        rc == 0
    ), f"Found {len(errors)} code style errors / warnings:\n" + "\n".join(
        errors
    )
