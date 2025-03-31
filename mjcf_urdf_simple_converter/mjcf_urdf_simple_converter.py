import mujoco
import numpy
from xml.etree import ElementTree as ET
from xml.dom import minidom
from scipy.spatial.transform import Rotation
import numpy as np
from stl import mesh
import os

def array2str(arr):
    return " ".join([str(x) for x in arr])

def create_body(xml_root, name, inertial_pos, inertial_rpy, mass, ixx, iyy, izz):
    """
    create a body with given mass and inertia
    """
    # create XML element for this body
    body = ET.SubElement(xml_root, 'link', {'name': name})

    # add inertial element
    inertial = ET.SubElement(body, 'inertial')
    ET.SubElement(inertial, 'origin', {'xyz': array2str(inertial_pos), 'rpy': array2str(inertial_rpy)})
    ET.SubElement(inertial, 'mass', {'value': str(mass)})
    ET.SubElement(inertial, 'inertia', {'ixx': str(ixx), 'iyy': str(iyy), 'izz': str(izz),
                                        'ixy': "0", 'ixz': "0", 'iyz': "0"})
    return body

def create_dummy_body(xml_root, name):
    """
    create a dummy body with negligible mass and inertia
    """
    mass = 0.001
    mass_moi = mass * (0.001 ** 2)  # mass moment of inertia
    return create_body(xml_root, name, np.zeros(3), np.zeros(3), mass, mass_moi, mass_moi, mass_moi)
    

def create_joint(xml_root, name, parent, child, pos, rpy, axis=None, jnt_range=None):
    """
    if axis and jnt_range is None, create a fixed joint. otherwise, create a revolute joint
    """
    if axis is None:
        assert jnt_range is None
        joint_type = 'fixed'
    else:
        joint_type = 'revolute'
    # create joint element connecting this to parent
    jnt_element = ET.SubElement(xml_root, 'joint', {'type': joint_type, 'name': name})
    ET.SubElement(jnt_element, 'parent', {'link': parent})
    ET.SubElement(jnt_element, 'child', {'link': child})
    ET.SubElement(jnt_element, 'origin', {'xyz': array2str(pos), 'rpy': array2str(rpy)})
    if axis is not None:
        ET.SubElement(jnt_element, 'axis', {'xyz': array2str(axis)})
        ET.SubElement(jnt_element, 'limit', {'lower': str(jnt_range[0]), 'upper': str(jnt_range[1]), 'effort': "100", 'velocity': "100"})
    return jnt_element


def convert(mjcf_file, urdf_file, asset_file_prefix=""):
    """
    load MJCF file, parse it in mujoco and save it as URDF
    replicate just the kinematic structure, ignore most dynamics, actuators, etc.
    only works with mesh geoms
    https://mujoco.readthedocs.io/en/stable/APIreference.html#mjmodel
    http://wiki.ros.org/urdf/XML
    
    :param mjcf_file: path to existing MJCF file which will be loaded
    :param urdf_file: path to URDF file which will be saved
    :param asset_file_prefix: prefix to add to the stl file names (e.g. package://my_package/meshes/)
    """
    assert mjcf_file.endswith(".xml"), f"{mjcf_file=} should end with .xml"
    assert urdf_file.endswith(".urdf"), f"{urdf_file=} should end with .urdf"
    output_dir = os.path.dirname(urdf_file)
    assert os.path.exists(output_dir), f"{output_dir=} does not exist, please create it first"
    model = mujoco.MjModel.from_xml_path(mjcf_file)
    root = ET.Element('robot', {'name': "converted_robot"})
    root.append(ET.Comment('generated with mjcf_urdf_simple_converter (https://github.com/Yasu31/mjcf_urdf_simple_converter)'))

    for id in range(model.nbody):
        child_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, id)
        parent_id = model.body_parentid[id]
        parent_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, parent_id)

        # URDFs assume that the link origin is at the joint position, while in MJCF they can have user-defined values
        # this requires some conversion for the visual, inertial, and joint elements...
        # this is done by creating a dummy body with negligible mass and inertia at the joint position.
        parentbody2childbody_pos = model.body_pos[id]
        parentbody2childbody_quat = model.body_quat[id]  # [w, x, y, z]
        # change to [x, y, z, w]
        parentbody2childbody_quat = [parentbody2childbody_quat[1], parentbody2childbody_quat[2], parentbody2childbody_quat[3], parentbody2childbody_quat[0]]
        parentbody2childbody_Rot = Rotation.from_quat(parentbody2childbody_quat).as_matrix()
        parentbody2childbody_rpy = Rotation.from_matrix(parentbody2childbody_Rot).as_euler('xyz')

        # read inertial info
        mass = model.body_mass[id]
        inertia = model.body_inertia[id]
        childbody2childinertia_pos = model.body_ipos[id]
        childbody2childinertia_quat = model.body_iquat[id]  # [w, x, y, z]
        # change to [x, y, z, w]
        childbody2childinertia_quat = [childbody2childinertia_quat[1], childbody2childinertia_quat[2], childbody2childinertia_quat[3], childbody2childinertia_quat[0]]
        childbody2childinertia_Rot = Rotation.from_quat(childbody2childinertia_quat).as_matrix()
        childbody2childinertia_rpy = Rotation.from_matrix(childbody2childinertia_Rot).as_euler('xyz')

        # create child body
        body_element = create_body(root, child_name, childbody2childinertia_pos, childbody2childinertia_rpy, mass, inertia[0], inertia[1], inertia[2])

        # read geom info and add it child body
        geomnum = model.body_geomnum[id]
        for geomnum_i in range(geomnum):
            geomid = model.body_geomadr[id] + geomnum_i
            if model.geom_type[geomid] != mujoco.mjtGeom.mjGEOM_MESH:
                # only support mesh geoms
                continue
            geom_dataid = model.geom_dataid[geomid]  # id of geom's mesh
            geom_pos = model.geom_pos[geomid]
            geom_quat = model.geom_quat[geomid]  # [w, x, y, z]
            # change to [x, y, z, w]
            geom_quat = [geom_quat[1], geom_quat[2], geom_quat[3], geom_quat[0]]
            geom_rpy = Rotation.from_quat(geom_quat).as_euler('xyz')
            mesh_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_MESH, geom_dataid)

            # create visual element within body element
            visual_element = ET.SubElement(body_element, 'visual', {'name': mesh_name})
            origin_element = ET.SubElement(visual_element, 'origin', {'xyz': array2str(geom_pos), 'rpy': array2str(geom_rpy)})
            geometry_element = ET.SubElement(visual_element, 'geometry')
            mesh_element = ET.SubElement(geometry_element, 'mesh', {'filename': f"{asset_file_prefix}converted_{mesh_name}.stl"})
            material_element = ET.SubElement(visual_element, 'material', {'name': 'white'})

            # create STL
            # the meshes in the MjModel seem to be different (have different pose) from the original STLs
            # so rather than using the original STLs, write them out from the MjModel
            # https://stackoverflow.com/questions/60066405/create-a-stl-file-from-a-collection-of-points
            vertadr = model.mesh_vertadr[geom_dataid]  # first vertex address
            vertnum = model.mesh_vertnum[geom_dataid]
            vert = model.mesh_vert[vertadr:vertadr+vertnum]
            normal = model.mesh_normal[vertadr:vertadr+vertnum]
            faceadr = model.mesh_faceadr[geom_dataid]  # first face address
            facenum = model.mesh_facenum[geom_dataid]
            face = model.mesh_face[faceadr:faceadr+facenum]
            data = np.zeros(facenum, dtype=mesh.Mesh.dtype)
            for i in range(facenum):
                data['vectors'][i] = vert[face[i]]
            m = mesh.Mesh(data, remove_empty_areas=False)
            mesh_save_path = os.path.join(output_dir, f"converted_{mesh_name}.stl")
            m.save(mesh_save_path)


        jntnum = model.body_jntnum[id]

        if child_name == "world":
            # there is no joint connecting the world to anything, since it is the root
            assert parent_name == "world"
            assert jntnum == 0
            continue  # skip adding joint element or parent body

        
        if jntnum == 0:
            # No joints, create a fixed joint directly to parent
            jnt_name = f"{parent_name}2{child_name}_fixed"
            parentbody2jnt_pos = parentbody2childbody_pos
            parentbody2jnt_rpy = parentbody2childbody_rpy
            create_joint(root, jnt_name, parent_name, child_name, parentbody2jnt_pos, parentbody2jnt_rpy)
        else:
            # For bodies with joints, create a chain of dummy bodies for each joint
            current_parent = parent_name
            cumulative_pos = np.zeros(3)  # position of the current joint in the child body frame
            cumulative_rot = np.eye(3)
            
            # Process all joints for this body
            for j in range(jntnum):
                jntid = model.body_jntadr[id] + j
                jnt_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, jntid)
                if jnt_name is None:
                    # Generate a random name for the joint
                    jnt_name = f"joint_{jntid}"
                    print(f"WARNING: joint name for {jntid} is None (could happen for ball joints with >1DoF), using automatically generated name {jnt_name}")
                jnt_body_name = f"{jnt_name}_jointbody"
                
                # Create dummy body for this joint
                create_dummy_body(root, jnt_body_name)
                
                if model.jnt_type[jntid] == mujoco.mjtJoint.mjJNT_HINGE:
                    # Revolute joint
                    jnt_range = model.jnt_range[jntid]  # [min, max]
                    jnt_axis_childbody = model.jnt_axis[jntid]  # [x, y, z]
                    childbody2jnt_pos = model.jnt_pos[jntid]  # [x, y, z]
                    
                    # Calculate joint position in parent body frame
                    if j == 0:
                        # First joint connects to original parent
                        parentbody2jnt_pos = parentbody2childbody_pos + parentbody2childbody_Rot @ childbody2jnt_pos
                        parentbody2jnt_rpy = parentbody2childbody_rpy
                        parentbody2jnt_axis = jnt_axis_childbody  # In child body frame
                    else:
                        # Subsequent joints connect to previous dummy body
                        # Position is relative to original child body position
                        parentbody2jnt_pos = childbody2jnt_pos - cumulative_pos
                        parentbody2jnt_rpy = np.zeros(3)
                        parentbody2jnt_axis = jnt_axis_childbody
                    
                    # Connect current parent to this joint body
                    create_joint(root, jnt_name, current_parent, jnt_body_name, 
                                parentbody2jnt_pos, parentbody2jnt_rpy, 
                                parentbody2jnt_axis, jnt_range)
                    
                else:
                    # Handle other joint types (as fixed joints for now)
                    print(f"doesn't support joint type {model.jnt_type[jntid]}, treating as fixed joint...")
                    childbody2jnt_pos = model.jnt_pos[jntid]  # [x, y, z]
                    if j == 0:
                        parentbody2jnt_pos = parentbody2childbody_pos
                        parentbody2jnt_rpy = parentbody2childbody_rpy
                    else:
                        parentbody2jnt_pos = np.zeros(3)
                        parentbody2jnt_rpy = np.zeros(3)
                    
                    create_joint(root, jnt_name, current_parent, jnt_body_name,
                                parentbody2jnt_pos, parentbody2jnt_rpy)
                
                # Update parent for next joint in chain
                current_parent = jnt_body_name
                cumulative_pos += childbody2jnt_pos
            
            # Connect last dummy body to child body with fixed joint
            # "bring back" the body coordinates to the child body frame
            jnt2childbody_pos = - childbody2jnt_pos if jntnum > 0 else np.zeros(3)
            jnt2childbody_rpy = np.zeros(3)
            create_joint(root, f"{jnt_name}_offset", current_parent, child_name,
                         jnt2childbody_pos, jnt2childbody_rpy)
    
    # define white material
    material_element = ET.SubElement(root, 'material', {'name': 'white'})
    color_element = ET.SubElement(material_element, 'color', {'rgba': '1 1 1 1'})

    # write to file with pretty printing
    xmlstr = minidom.parseString(ET.tostring(root)).toprettyxml(indent="   ")
    with open(urdf_file, "w") as f:
        f.write(xmlstr)

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('mjcf_file', type=str)
    parser.add_argument('urdf_file', type=str)
    args = parser.parse_args()
    convert(args.mjcf_file, args.urdf_file)