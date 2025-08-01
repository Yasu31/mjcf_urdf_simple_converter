from pathlib import Path

from mjcf_urdf_simple_converter import convert


def test_mesh_export(tmp_path):
    mjcf_path = Path(__file__).with_name("data") / "mesh_mjcf.xml"
    urdf_path = tmp_path / "mesh.urdf"
    convert(str(mjcf_path), str(urdf_path))
    obj_path = tmp_path / "meshes" / "converted_tetra.obj"
    mtl_path = tmp_path / "meshes" / "converted_tetra.mtl"
    assert urdf_path.exists()
    assert obj_path.exists()
    assert mtl_path.exists()
    text = mtl_path.read_text()
    assert "Kd 0" in text and "1" in text
