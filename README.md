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
This converts the `model.xml` (and any associated MJCF files loaded from within `model.xml`) to `model.urdf`. Mesh geoms are also converted to STL files, and saved to `converted_*.stl` files within the same directory. The converted files can be checked in Rviz, or online tools like https://gkjohnson.github.io/urdf-loaders/javascript/example/bundle/index.html .

### what are converted
* links
  * mesh geoms
  * inertial information
* joints
  * joint position, axis and limits (arbitrary values are set for effort and velocity limits)
  * only hinge (revolute) joints are supported
  * only up to one joint per link is supported

### what are NOT converted
* actuators
* sensors
* tendons
* etc.

## comments
* A similar tool exists to convert from MJCF to URDF ([mjcf2urdf](https://github.com/iory/mjcf2urdf)), but the pybullet import that it uses did not work for our case so this tool was created.
* for converting from URDF to MJCF, the `./compile` program bundled with standard MuJoCo installations can be used.