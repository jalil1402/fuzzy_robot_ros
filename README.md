# Fuzzy & Kinematic Control in Gazebo
A ROS Noetic workspace featuring a differential drive robot.

## Controllers included:
- **Mamdani Fuzzy Logic:** Stable waypoint and circle tracking.
- **Kinematic Law (Eq 7):** Time-independent path following for y = sin(0.5x) + 0.5x.

## How to run:
1. `roslaunch fuzzy_control gazebo.launch`
2. `rosrun fuzzy_control kinematic_path_node.py`
