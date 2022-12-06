# Universal Robots ROS Driver
This is a summary documentation for use the Universal Robots ROS Driver with the UR10 robot arm. the original documentation can be found [here](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver), check it out if you want to use another type of robot.

## Introduction:
Universal Robots have become a dominant supplier of lightweight, robotic manipulators for industry, as well as for scientific research and education. The Robot Operating System (ROS) has developed from a community-centered movement to a mature framework and quasi standard, providing a rich set of powerful tools for robot engineers and researchers, working in many different domains.

## Requirements

### Software Requirements

To use this driver you will need to have this software installed on your machine.

1.- Ubuntu 20.04.

2.- ROS Noetic full-desktop.

3.- Moveit!.

### Hardware Requeriment

1.- A computer with ethernet input.

## Building the UR ROS driver

Run this command in your terminal to download and compile the driver
```
source /opt/ros/<your_ros_version>/setup.bash
mkdir -p ur_ws/src && cd ur_ws
git clone -b boost https://github.com/UniversalRobots/Universal_Robots_Client_Library.git src/Universal_Robots_Client_Library
git clone https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git src/Universal_Robots_ROS_Driver
git clone -b melodic-devel https://github.com/ros-industrial/universal_robot.git src/universal_robot
sudo apt update -qq
rosdep update
rosdep install --from-paths src --ignore-src -y
catkin_make_isolated
source devel_isolated/setup.bash
```

## Prepare the robot

To enable external control of the UR robot from a remote PC you need to install the externalcontrol-1.0.5.urcap which can be downloaded from [Universal_Robots_ExternalControl_URCap](https://github.com/UniversalRobots/Universal_Robots_ExternalControl_URCap/releases).

For installing the necessary URCap and creating a program, please see the individual tutorial on [how to setup a CB3 robot](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/master/ur_robot_driver/doc/install_urcap_cb3.md).

## Network Setup

This section describes a good example using static IP addresses and a direct connection from the PC to the Robot to minimize latency introduced by network hardware.

1. Connect the UR control box directly to the remote PC with an ethernet cable.
2. Open the network settings from the UR teach pendant (Setup Robot -> Network) and enter these settings:
```
IP address: 192.168.1.102
Subnet mask: 255.255.255.0
Default gateway: 192.168.1.1
Preferred DNS server: 192.168.1.1
Alternative DNS server: 0.0.0.0
```
3. On the remote PC, turn off all network devices except the "wired connection", e.g. turn off wifi.

4. Open Network Settings and create a new Wired connection with these settings. You may want to name this new connection UR or something similar:

```
IPv4
Manual
Address: 192.168.1.101
Netmask: 255.255.255.0
Gateway: 192.168.1.1
```
5. Verify the connection from the PC with e.g. ping.
```
ping 192.168.1.102
```


## Usage the driver

1. From the Program Robot tab of the PolyScope, load "external_contro.urp". Click on the "Control by..." section of the program to check the Host IP of the external PC. If it needs to be modified, make the modification under the Installation tab (as prompted on screen). You do not need to modify the Custom Port. Also disable the EtherNet/IP and the PORFINET IO.

2. On your computer, use this command to grant access to the Host port. Is have to be the same as robot's Custom Port.
```
sudo ufw allow <port>
```

3. Extract calibration information.
```
roslaunch ur_calibration calibration_correction.launch robot_ip:=192.168.1.102 target_filename:="${HOME}/my_robot_calibration.yaml"
```
(Though this step is not necessary to control the robot using this driver, it is highly recommended to do so, as otherwise endeffector positions might be off in the magnitude of centimeters.)

4. Start the robot driver by running this launch file in your terminal:
```
roslaunch ur_robot_driver ur10_bringup.launch robot_ip:=192.168.1.102 kinematics_config:="${HOME}/my_robot_calibration.yaml"
```

5. From the Program Robot tab of the PolyScope, click the play button to connect with the external PC. You should see this next message in your driver's terminal:
```
Robot requested program
Sent program to robot
Robot connected to reverse interface. Ready to receive control commands.
```

6. (Optional) Open up an RViz window showing the robot. In another terminal, run:
```
roslaunch ur_robot_driver example_rviz.launch
```
Try moving around the robot's joints using the teach panel and watch how it also moves inside ROS.

7. (Optional) Control the robot using the test_move script. In a third terminal run:
```
rosrun ur_robot_driver test_move
```

## Control the robot using Moveit!
```
roslaunch ur_calibration calibration_correction.launch robot_ip:=192.168.1.102 target_filename:="${HOME}/my_robot_calibration.yaml"
```

```
roslaunch ur_robot_driver ur10_bringup.launch robot_ip:=192.168.1.102 kinematics_config:="${HOME}/my_robot_calibration.yaml"

```

```
roslaunch ur10_moveit_config moveit_planning_execution.launch
```

```
roslaunch ur10_moveit_config moveit_rviz.launch rviz_config:=$(rospack find ur10_moveit_config)/launch/moveit.rviz
```
You can check out everything you can do with this driver [here](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/master/ur_robot_driver/doc/ROS_INTERFACE.md).

