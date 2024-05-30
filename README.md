# Final project Robotics
Authors: Ian Abuaf Pelo, Eleonora Bardelli, Simone Tarenzi

## How to run

### Coppelia Sim
1. Open CoppeliaSim
2. load the scene
3. Start the simulation with the real-time mode toggle enabled

### Dependencies and compilation
This project requires the `opencv-contrib-python` library, but the `opencv-python` library might already be installed.
To fix this, run:
```
pip3 uninstall opencv-python
pip3 install opencv-contrib-python
```
Compile the code with these commands: 
```
cd ~/dev_ws

colcon build --packages-select project

source install/setup.bash
```
### Optional: Calibrate camera
Calibrate the camera matrix, pose and distorsion with the calibration node, which must run inside the provided calibration scene:
```
ros2 launch project calibrate.launch.xml
```
Wait until the confirmation message and terminate with Ctrl+C.

### Launch
Launch the main node:

```
ros2 launch project vs.launch.xml
```
#### Arguments
The main node can receive the following arguments:
- `target_id`: Set the id of the ArUco marker to target during alignment phase.

    Default is 55. The provided scene has objects with marker id 55, 56 and 57.
- `thymio_id`: Set the id of the ArUco marker to target during Thymio follow phase.

    The provided scene has a thymio with ArUco marker id 74, so this is the default.
- `use_calibration`: Set to True to use the calibration data from the calibration step.

    By default the node will use the optimal camera matrix (False), which we find to work better.
- `offset`: Change to the offset in the x direction used when aligning to a marker.

    0 by default. Recommended to be set to a small negative number like $-0.2$ when using the calibration camera. 
- `plot_alignment`: Generate a plot of the gripper's position during alignment with the
    marker.

    Defaults to False. See the report for some sample plots.

For example, 
```
ros2 launch project vs.launch.xml target_id:=56 use_calibration:=True offset:=-0.2
```