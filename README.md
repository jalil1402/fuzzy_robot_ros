# Fuzzy & Kinematic Control in Gazebo
A ROS Noetic workspace featuring a differential drive robot.

## Controllers included:
- **Mamdani Fuzzy Logic:** zigzag and circle tracking.
- **Kinematic Law (Eq 7):** Time-independent path following for y = sin(0.5x) + 0.5x.

## How to run:
1. `roslaunch fuzzy_control gazebo.launch`
2. `rosrun fuzzy_control kinematic_path_node.py`
<img width="1366" height="768" alt="Screenshot from fuzzy_mamdani_zigzag mp4" src="https://github.com/user-attachments/assets/ca590f4c-deca-4342-86e6-ad2cdec6e866" />
<img width="1366" height="768" alt="Screenshot from kinmetic_controler_bad_ending" src="https://github.com/user-attachments/assets/0ab9a638-e039-4f2a-8efb-c27753a892d4" />
