"""Tests for converting basic MJCF models to URDF files."""

from pathlib import Path

from mjcf_urdf_simple_converter import convert
import mujoco


def test_simple_mjcf_conversion(tmp_path):
    """Convert a minimal MJCF model and verify a URDF file is created."""
    mjcf_path = Path(__file__).with_name("sample_mjcf") / "my_robot.xml"
    urdf_path = tmp_path / "articulated.urdf"
    convert(str(mjcf_path), str(urdf_path))
    assert urdf_path.exists()

    # check that exported URDF can at least be loaded in MuJoCo and has same DoF as original MJCF
    mjcf_model = mujoco.MjModel.from_xml_path(str(mjcf_path))
    urdf_model = mujoco.MjModel.from_xml_path(str(urdf_path))
    assert mjcf_model.nv == urdf_model.nv

    # check that the mesh files are created
    red_code = "".join(f"{int(round(c * 255)):02x}" for c in (1, 0, 0, 1))
    green_code = "".join(f"{int(round(c * 255)):02x}" for c in (0, 1, 0, 1))
    blue_code = "".join(f"{int(round(c * 255)):02x}" for c in (0, 0, 1, 1))
    red_obj = tmp_path / "meshes" / f"converted_tetra_{red_code}.obj"
    green_obj = tmp_path / "meshes" / f"converted_tetra_{green_code}.obj"
    blue_obj = tmp_path / "meshes" / f"converted_tetra_{blue_code}.obj"
    assert red_obj.exists()
    assert green_obj.exists()
    assert blue_obj.exists()
