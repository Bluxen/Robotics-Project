# Final project robotics
## Instructions
### Coppelia Sim
1. Open CoppeliaSim
2. load the scene
3. Start the simulation with the real-time mode toggle enabled

### How to run
Compile the code with these commands: 
```
cd ~/dev_ws

colcon build --packages-select project

source install/setup.bash
```
Calibrate the camera matrix, pose and distorsion with the calibration node, which must run inside the provided calibration scene:
```
ros2 launch project calibrate.launch.xml
```
Wait until the confirmation message and terminate with Ctrl+C.

Launch the main node:
```
ros2 launch project test.launch.xml
```