"""Tests for converting basic MJCF models to URDF files."""

from pathlib import Path

from mjcf_urdf_simple_converter import convert
import mujoco


def test_simple_mjcf_conversion(tmp_path):
    """Convert a minimal MJCF model and verify a URDF file is created."""
    mjcf_path = Path(__file__).with_name("data") / "articulated_mjcf.xml"
    urdf_path = tmp_path / "articulated.urdf"
    convert(str(mjcf_path), str(urdf_path))
    assert urdf_path.exists()

    # check that exported URDF can at least be loaded in MuJoCo and has same DoF as original MJCF
    mjcf_model = mujoco.MjModel.from_xml_path(str(mjcf_path))
    urdf_model = mujoco.MjModel.from_xml_path(str(urdf_path))
    assert mjcf_model.nv == urdf_model.nv