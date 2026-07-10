# NDE Church Window

This project combines a KUKA iiwa 14 robotic manipulator, an eddy current sensor array, and a Zivid M70 3D camera to dynamically scan a complex geometry referred to as the church window. The goal is to use the Zivid M70 to localize the window, then plan and execute scan paths around it.

The workspace targets ROS 2 Humble on Ubuntu 22.04.

## What is in the workspace

- `src/church_window`: custom ROS 2 package with launch files, meshes, URDF, and the flat scan script
- `src/config`: generated MoveIt configuration and RViz demo launch files
- `src/lbr-stack`: KUKA LBR ROS 2 stack and bringup support
- `src/zivid-ros`: Zivid ROS 2 integration

## Dependencies

The project currently depends on the ROS 2 Humble ecosystem plus the packages used by the `church_window` and `config` packages, including MoveIt 2, the LBR stack, and the Zivid ROS 2 stack.

Recommended base tools:

- `ros-humble-desktop`
- `python3-colcon-common-extensions`
- `python3-rosdep`
- `python3-vcstool`
- `python3-pip`

If you are setting up a fresh machine, install the ROS tools first, then let `rosdep` resolve the remaining package dependencies from the workspace.

## Setup

```bash
mkdir -p ~/ndt_church_window/src
cd ~/ndt_church_window/src

# Clone this repository and any external dependencies you keep outside the repo
# such as the LBR stack or Zivid ROS 2 packages.

cd ~/ndt_church_window
source /opt/ros/humble/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Run the RViz demo

The MoveIt demo lives in the `config` package.

```bash
cd ~/ndt_church_window
source install/setup.bash
ros2 launch config demo.launch.py
```

If you want the robot bringup and control-side launch files from the custom package, use:

```bash
ros2 launch church_window mock.launch.py
ros2 launch church_window move_group.launch.py
```

## Run the scan script

The scan script currently uses the CAD model at:

`src/church_window/meshes/CHURCH_WINDOW_NDT_TEST.stl`

It also assumes the CAD model orientation in `src/church_window/scripts/flat_scan.py` matches the mesh orientation in the workspace.

```bash
cd ~/ndt_church_window
source install/setup.bash
python3 src/church_window/scripts/flat_scan.py
```

## Notes on the scan model

- The URDF includes a virtual joint and links at the sensor tip and optical center of the Zivid M70 so the camera pose can be represented in planning.
- The flat scan script speaks directly to MoveIt 2 services and actions, so the relevant MoveIt interfaces must be running before you start it.
- The current implementation is CAD-driven for path generation; Zivid camera triangulation is part of the intended workflow but is not yet used by the scan script.

## Development

Common development loop:

1. Make your changes in `src/church_window` or `src/config`.
2. Rebuild the workspace with `colcon build --symlink-install`.
3. Source `install/setup.bash` again.
4. Relaunch the MoveIt demo or scan script.

## Project status

This is an active development workspace. The current focus is on reliable localization of the church window, path planning around complex geometry, and integrating the Zivid M70 into the full scanning pipeline.