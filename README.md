# Final project robotics
## Instructions
### Coppelia Sim
1. Open CoppeliaSim
2. load the scene
3. Start the simulation with the real-time mode toggle enabled




### Compulsory run
1. In folder ~/dev_ws
This first terminal is used to compile the code with the command: 
```
cd ~/dev_ws
colcon build --packages-select project
```
2. In the second terminal in ~/dev_ws/ it must be pasted:
```
cd ~/dev_ws
source install/setup.bash
ros2 launch project test.launch
```