# ROS2 Programming Assignment #4
 - Assignment 1 - Publisher / Subscriber
 - Assignment 2 - Logging and Services
 - Assignment 3 - Tf2, unit testing, and bag files
 - Assignment 4 - Working with webots

 ## Overview
 This ROS 2 package provides a webots demonstration of an obstacle avoiding robot. The robot controller was implemented using a Finite State Machine design pattern in C++. Demonstration spawns the robot in a circular arena with blocks spawned around it. The robot will move forward until one of it's distance sensors detects an object is too close, at which point it will turn to avoid the obstacle. This process repeats until the simulation is terminated.

 ## Dependencies
 - This package assumes that you have ROS2 Humble properly installed and setup on your machine. If this is not the case, the steps to install ROS2 can be found [here](https://docs.ros.org/en/humble/Installation.html).
 - This package depends on webots simulator. To install webots on your machine follow the steps [here](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Webots/Installation-Ubuntu.html).
 - This package is structured to allow for unit testing with catch2. To install catch2 on your machine follow the steps below.
    ```bash
    source /opt/ros/humble/setup.bash  # if needed
    apt install ros-${ROS_DISTRO}-catch-ros2
    ```

 ## How to Build/Run
 ### Create ROS2 Workspace
```bash
# Source ROS2 underlay
source /opt/ros/humble/setup.bash

# Navigate to the driectory location where you wish to create a ROS2 workspace
mkdir ros2_ws
cd ros2_ws/
mkdir src
cd src/
cd ..

colcon build
```
### Clone and Build Repository
```bash
cd src/
git clone https://github.com/GraysonGilbert/my_webots_tutorials.git
cd ..
colcon build --packages-select walker

```
### Launch walker robot simulation
```bash
# Source ROS2 underlay
source /opt/ros/humble/setup.bash
# Source overlay
. install/setup.bash

# Launches walker robot
ros2 launch walker robot_launch.launch.py 
```

ROS 2 bag recording is enabled by default, to launch the simulation without recording a bag, run the following:
```bash
# Launches walker robot without bag recording
ros2 launch walker robot_launch.launch.py enable_bag_recording:=false 
```

### Inspecting and playing bag back
```bash
# Inspect the bag's content with 
ros2 bag info <bag name>


# Make sure webots is NOT running in the background as the walker robot will get confused
# about which /cmd_vel it should listen to 

# Play back the bag's contents with
ros2 bag play <bag name>
```