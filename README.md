# Final project robotics
## Instructions
### Coppelia Sim
1. Open CoppeliaSim
2. load the scene
3. Start the simulation with the real-time mode toggle enabled

### How to run
Compile and run the code with these commands: 
```
cd ~/dev_ws

colcon build --packages-select project

source install/setup.bash

ros2 launch project test.launch
```