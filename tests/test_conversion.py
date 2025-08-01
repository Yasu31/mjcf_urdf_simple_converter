from pathlib import Path

from mjcf_urdf_simple_converter import convert


def test_simple_mjcf_conversion(tmp_path):
    """Convert a tiny MJCF file and ensure the URDF file is produced."""
    mjcf_path = Path(__file__).with_name("data") / "simple_mjcf.xml"
    urdf_path = tmp_path / "simple.urdf"
    convert(str(mjcf_path), str(urdf_path))
    assert urdf_path.exists()
