## About

This package provides a simple differential drive robot model designed for use in Gazebo Harmonic simulation with ROS 2 Jazzy Jalisco. 

![differential_drive_robot](https://github.com/user-attachments/assets/85a0b166-be33-4844-9ed3-8ffa028069d8)

## Requirements

To run this package, you'll need the following:

- [Linux Ubuntu 24.04](https://ubuntu.com/blog/tag/ubuntu-24-04-lts)
- [ROS2 Jazzy Jalisco](https://docs.ros.org/en/rolling/Releases/Release-Jazzy-Jalisco.html)
- [Gazebo Harmonic](https://gazebosim.org/docs/harmonic/getstarted/) 


#### Install Required ROS 2 Packages

Make sure to install the following ROS 2 Jazzy Jalisco packages:

```bash
sudo apt install -y                         \
    ros-jazzy-joint-state-publisher         \
    ros-jazzy-joint-state-publisher-gui     \
    ros-jazzy-xacro                         \
    ros-jazzy-teleop-twist-keyboard         \
    ros-jazzy-teleop-twist-joy 
```

## Usage


#### Clone the Repository

Clone this repository into your ``workspace/src`` folder. If you don't have a workspace set up, you can learn more about creating one in the [ROS 2 workspace tutorial](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html).


```bash
cd <path_to_your_workspace>/src
git clone git@github.com:lucasmazz/differential_drive_robot.git
cd ..
```

#### Build the Package

Source the ROS 2 environment and build the package:

```bash
source /opt/ros/jazzy/setup.bash
colcon build
```

#### Launch the Robot

After building the package, launch the ```robot.launch.py``` file from the ```differential_drive_robot``` package:

```bash
source install/setup.bash
ros2 launch differential_drive_robot robot.launch.py
```

#### Control the Robot

**Using a Joystick**

In a new terminal, source the environment and launch the ```teleop-launch.py``` file from the ```teleop_twist_joy``` package. Adjust the joy_config parameter to match your joystick controller (e.g., xbox).

```bash
source /opt/ros/jazzy/setup.bash
ros2 launch teleop_twist_joy teleop-launch.py joy_config:='xbox'
```

**Using a Keyboard**

If you don't have a joystick, you can control the robot using the ```teleop_twist_keyboard``` package. Run the following command:

```bash
source /opt/ros/jazzy/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
