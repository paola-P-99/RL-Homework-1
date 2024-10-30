# RL-Homework-1

## 4-DOFs Arm Simulation in ROS2

### Project Overview

The goal of this project is to bring up a 4-degrees-of-freedom robot manipulator, and to simulate it within the Gazebo environment. 

### Installation
1. **Clone the repository** within the source folder of your ROS2 workspace.
  ```
  https://github.com/paola-P-99/RL-Homework-1.git
  ```
2. **Build the packages** in your ROS2 workspace.
  ```
  colcon build --packages-select arm_control arm_description arm_gazebo
  ```

3. **Source the workspace**.
  ```
  source install/setup.bash
  ```
### Project Structure
```
ros2_ws/
└── src/
    ├── arm_description/
    |   ├── config/
    │   ├── launch/
    |   ├── meshes/
    │   ├── urdf/
    │   └── CMakeLists.txt
    │   └── LICENSE
    │   └── package.xml
    ├── arm_gazebo/
    │   ├── launch/
    │   └── CMakeLists.txt
    │   └── LICENSE
    │   └── package.xml
    ├── arm_control/
    │   ├── config/
    │   ├── launch/
    │   └── src/
    │   └── CMakeLists.txt
    │   └── LICENSE
    │   └── package.xml
```
### Launch
#### To visualize the manipulator in Rviz.
```
ros2 launch arm_description display.launch.py
```
#### To launch the simulation in Gazebo.
```
ros2 launch arm_gazebo arm_world.launch.py
```
#### To spawn the controllers, while running the simulation.
```
ros2 launch arm_control arm_control.launch.py
```
#### To launch the simulation in Gazebo and spawn the controllers.
```
ros2 launch arm_gazebo arm_gazebo.launch.py
```
#### To give a position command to the manipulator, while running the simulation.
```
ros2 run arm_control arm_controller_node
```




   
