# mjcf_urdf_simple_converter
A minimal and simple script to convert **limited elements** of MJCF (MuJoCo modeling format) robot model files to URDF. Developed from a need to visualize MJCF robots in ROS environments like Rviz, it only converts a limited subset of the robot model elements to URDF.
The model file is loaded in the Python `mujoco` package, and its model elements are parsed using the library and then output according to the URDF XML format.

I do **not** intend to expand this into a full-fledged transclation script from MJCF to URDF.

# installation
```
pip install mjcf-urdf-simple-converter
```

## usage
```python
from mjcf_urdf_simple_converter import convert
convert("model.xml", "model.urdf")
# or, if you are using it in your ROS package and would like for the mesh directories to be resolved correctly, set meshfile_prefix, for example:
convert("model.xml", "model.urdf", asset_file_prefix="package://your_package_name/model/")
```
This converts the `model.xml` (and any associated MJCF files loaded from within `model.xml`) to `model.urdf`. Mesh geoms are also converted to STL files, and saved to `converted_*.stl` files within the same directory. The converted files can be checked in Rviz, or online tools like https://gkjohnson.github.io/urdf-loaders/javascript/example/bundle/index.html (just drag & drop the URDF file and all the mesh STL files into the page).

### what are converted
* links
  * mesh geoms
  * inertial information
* joints
  * joint position, axis and limits (arbitrary values are set for effort and velocity limits)
  * only hinge (revolute) joints are supported
  * ~~only up to one joint per link is supported~~ -> now supports multiple joints per body!

### what are NOT converted
* actuators
* sensors
* tendons
* etc.

## comments
* A similar tool exists to convert from MJCF to URDF ([mjcf2urdf](https://github.com/iory/mjcf2urdf)), but the pybullet import that it uses did not work for our case so this tool was created.
* for converting from URDF to MJCF, the `./compile` program bundled with standard MuJoCo installations can be used.


## how it works
in URDF, when a joint connects a parent and child body, the child body's origin is fixed to the position of the joint (https://wiki.ros.org/urdf/XML/joint). There is always only a single joint connecting the parent and child. On the other hand, for MJCFs there can be multiple joints defined in a body, and joints are defined within the child body element. Also, the body position is defined relative to the parent frame and it does not depend on the joint positions. Thus some conversions are necessary to translate the MJCF's kinematic structure to URDF.

The approach that I took in this code is to use just add many intermediate bodies whose relative poses are defined with `fixed` joints, to express each transformation. The `add_dummy_body()` function creates a body with negligible mass and inertia at the joint position, which creates an empty body, to which joints can be attached. This way, each actual body defined in the MJCF can keep using the same coordinate system 

For example, if a body in the MJCF has 2 joints:

- a `revolute` joint connects `parent_body` to `joint1_jointbody`
- a `revolute` joint connects `joint1_jointbody` to `joint2_jointbody`
- a `fixed` joint connects `joint2_jointbody` to `child_body` (this "brings back" the coordinate frame back to the MJCF child body's frame)

and the process is continued onwards to the next MJCF body.
![](kinematic_chain.drawio.svg)
