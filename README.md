# Prerequisiti
- CMake
- ROS 2 Humble

# Setup iniziale
## 1. Setup Workspace
```
mkdir ros2_pathplanning
cd ros2_pathplanning
colcon build
```

## 2. Setup Package
Da dentro la cartella 'ros2_pathplanning' fai:
```
mkdir src
cd src
git clone https://github.com/PathPlanningGroupAAC/PathPlanning.git -b ros2-humble-node
cd .. (torno dentro in 'ros2_pathplanning')
```

# Compilation
## Linux
Da dentro la cartella 'ros2_pathplanning' fai:
```
source /opt/ros/humble/setup.bash
colcon build
```

# Execution
## Linux
Da dentro la cartella 'ros2_pathplanning' fai:
```
source install/setup.bash
ros2 run path_planner path_planner
```
