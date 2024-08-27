# ROS2 Robot Controller GUI

This repository features a custom-built GUI designed for controlling and managing robots using ROS2. The interface simplifies the setup for complex robots and enables easy experimentation by publishing specific ROS2 topics that the robot's control code can interpret for various functions.

![main_window](https://github.com/user-attachments/assets/22c95f41-e9d6-4f9d-8821-e1625805cb1b)


## Features
1. The GUI is fully integrated with ROS2 (Humble & Foxy), allowing you to directly publish topics to manage your robot's functions.
2. While this repository doesn't detail every UI function, the code is structured for easy modification. You can adapt the controls to meet your specific needs.
3. The GUI offers a user-friendly interface for conducting experiments with your robot, whether you're focused on research, testing, or development.

## Components
**UI_window.py:** This script creates and manages the graphical user interface (GUI) panel.

**UI_controller.py:** This script accesses each button within the GUI and publishes the corresponding topic messages using ROS2.

## Installation
To install and set up the ROS2 Robot Controller GUI, follow these steps

```bash
mkdir -p ~/ui_ws/src
cd ~/ui_ws/src
git clone https://github.com/KimJiHong190/ros2_controller_gui.git
cd ..
colcon build --symlink-install
source install/setup.bash
```
Run the GUI
```bash
ros2 run robot_ui run_ui
```

## Usage
When you run the UI, you can confirm that the following topics are being published. You can click each button to see the format of the topics, and you can easily modify these in **UI_controller.py**. The GUI is designed to be intuitive, but feel free to modify the source code to fit your specific needs.


```bash
/calibration_topic
/log_publisher
/parameter_events
/robot1_control_command
/robot2_control_command
/rosout
/ui_setup_is_joint_activation
/ui_setup_is_robot2_activation
/ui_setup_is_robot_activation
/ui_setup_is_wheel_FL_activation
/ui_setup_is_wheel_FR_activation
/ui_setup_is_wheel_RL_activation
/ui_setup_is_wheel_RR_activation
/ui_setup_is_wheel_activation
/wheel_controller_command

```

## Contribution
Feel free to fork this repository and contribute by submitting pull requests. Any suggestions or improvements are welcome!
