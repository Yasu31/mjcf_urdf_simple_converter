# mjcf_urdf_simple_converter
A minimal and simple script to convert **limited elements** of MJCF (MuJoCo modeling format) robot model files to URDF. Developed from a need to visualize MJCF robots in ROS environments like Rviz, it only converts a limited subset of the robot model elements to URDF. I do **not** intend to expand this into a full-fledged transclation script from MJCF to URDF.

## usage
```python
from mjcf_urdf_simple_converter import convert
convert("model.xml", "model.urdf")
```
This converts the `model.xml` (and any associated MJCF files loaded from within `model.xml`) to `model.urdf`. Mesh geoms are also converted to STL files, and saved to `converted_*.stl` files within the same directory. The converted files can be checked in Rviz, or online tools like https://gkjohnson.github.io/urdf-loaders/javascript/example/bundle/index.html .

### what are converted

### what are NOT converted

## TODOs
* support multiple geoms in one body

## comments
* A similar tool exists to convert from MJCF to URDF ([mjcf2urdf](https://github.com/iory/mjcf2urdf)), but the pybullet import that it uses did not work for our case so this tool was created.
* for converting from URDF to MJCF, the `./compile` program bundled with standard MuJoCo installations can be used.