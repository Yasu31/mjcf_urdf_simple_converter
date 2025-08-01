from pathlib import Path

from mjcf_urdf_simple_converter import convert


def test_mesh_export(tmp_path):
    mjcf_path = Path(__file__).with_name("data") / "mesh_mjcf.xml"
    urdf_path = tmp_path / "mesh.urdf"
    convert(str(mjcf_path), str(urdf_path))
    color_code = "".join(f"{int(round(c * 255)):02x}" for c in (0, 1, 0, 0.5))
    obj_path = tmp_path / "meshes" / f"converted_tetra_{color_code}.obj"
    mtl_path = tmp_path / "meshes" / f"converted_tetra_{color_code}.mtl"
    assert urdf_path.exists()
    assert obj_path.exists()
    assert mtl_path.exists()
    text = mtl_path.read_text()
    assert "Kd 0" in text and "1" in text
