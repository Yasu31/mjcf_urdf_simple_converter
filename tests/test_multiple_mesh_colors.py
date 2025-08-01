from pathlib import Path
from mjcf_urdf_simple_converter import convert


def test_multiple_mesh_colors(tmp_path):
    mjcf_path = Path(__file__).with_name("data") / "multi_color_mjcf.xml"
    urdf_path = tmp_path / "mesh.urdf"
    convert(str(mjcf_path), str(urdf_path))
    red_code = "".join(f"{int(round(c * 255)):02x}" for c in (1, 0, 0, 1))
    blue_code = "".join(f"{int(round(c * 255)):02x}" for c in (0, 0, 1, 1))
    red_obj = tmp_path / "meshes" / f"converted_tetra_{red_code}.obj"
    blue_obj = tmp_path / "meshes" / f"converted_tetra_{blue_code}.obj"
    assert red_obj.exists()
    assert blue_obj.exists()
