## [0.7.2] - 2025-10-23
- Merge PR https://github.com/Yasu31/mjcf_urdf_simple_converter/pull/6 to respect material RGBA if specified
- change example MJCF used in tests which lets me check both articulation & RGB at same time

## [0.7] - 2025-08-24

- Add OBJ mesh + MTL color export
    - instead of STLs, use a combination of OBJ + MTL to bake the color directly into the meshes
    - now different colors for each mesh in one link can be achieved 
    - Rviz cannot render different colors for each `<visual>` element in a single `<link>` when using `<material>` to set colors (known issue, still there in ROS 2- see [here](https://robotics.stackexchange.com/questions/58836/color-issue-with-urdf-and-multiple-visual-tags) or [here](https://github.com/ros-visualization/rviz/issues/843)) so this approach was taken

## [0.6] - 2025-08-01

- fix the output directory assertion error when user just puts in the name of the file (instead of e.g. "./mjcf_file.xml")

## [0.5] — 2025-03-31

- Support multiple joints in a single body
- Update README.md with how the conversion for bodies works 

## [0.4] - 2024-07-22

- fix: avoid cases when joint name has same name as body name

## [0.3] – 2023‑07‑28

- fix: joint axis calculation

## [0.2] - 2023‑03‑13

- support multiple geoms per body
- add Github actions