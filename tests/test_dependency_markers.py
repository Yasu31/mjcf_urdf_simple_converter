"""Tests for Python-version-specific dependency markers."""

from pathlib import Path
import tomllib

from packaging.requirements import Requirement


def _selected_mujoco_specs(python_version: str) -> list[str]:
    pyproject_path = Path(__file__).resolve().parents[1] / "pyproject.toml"
    pyproject = tomllib.loads(pyproject_path.read_text(encoding="utf-8"))
    requirements = [Requirement(dep) for dep in pyproject["project"]["dependencies"]]

    return [
        str(req.specifier)
        for req in requirements
        if req.name == "mujoco"
        and (req.marker is None or req.marker.evaluate({"python_version": python_version}))
    ]


def test_mujoco_version_is_capped_for_python39():
    """Python 3.9 should use a MuJoCo version with prebuilt wheels."""
    assert _selected_mujoco_specs("3.9") == ["==3.3.4"]


def test_mujoco_version_is_uncapped_for_python310_and_newer():
    """Python 3.10+ should continue using the default latest MuJoCo."""
    assert _selected_mujoco_specs("3.10") == [""]
