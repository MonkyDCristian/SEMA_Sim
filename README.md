# SEMA Simulator
Simulator of the Automated Mobile Packaging System developed at the "Universidad de Los Andes de Colombia".

![Alt text](/imgs/gzsim_rviz_view.png)
## Introduction
This simulator replicates a physical environment of autonomous palletizing. It counts with a recursive conveyor belt, a UR10 cobot arm, a VGC10 (Vacuum Gipper), a Kinect camera, different box models and a pallet.

The objective of this simulator is to serve as a safe and malleable space where vision, trajectory and paking algorithms can be integrated and tested before being taken to the field-testing phase. In addition, it is expected to be used to test simulated hardware parts, such as grippers, cameras, even other robot arm models, in order to evaluate their value for the project.

## Dependencies

* Operating system: [Ubuntu 20.04](https://releases.ubuntu.com/focal/)
* Communication Software: [ROS Noetic (full-desktop)](http://wiki.ros.org/noetic/Installation/Ubuntu)
* Robot arm control software: [Moveit](https://ros-planning.github.io/moveit_tutorials/doc/getting_started/getting_started.html)

## Install and Compile
```
git clone https://github.com/MonkyDCristian/SEMA_Sim.git
cd /SEMA_sim/sema_ws
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
cd
echo "source {PATH_TO}/SEMA_sim/sem_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Getting Started
* The Instructions assume familiarity with Python3, Linux environment, Robotic Operating System (ROS) framework, Gazebo11, and Moveit! in Rviz. If you're new to these tools, you can get started with these great tutorials:
  * [Python3 Tutorial](https://app.theconstructsim.com/Course/58)
  * [Linux Tutorial](https://app.theconstructsim.com/Course/40)
  * [ROS Tutorials](http://wiki.ros.org/ROS/Tutorials)
  * [Gazebo simulator Tutorials](https://classic.gazebosim.org/tutorials?cat=get_started)
  * [Moveit! with Rviz Tutorial](https://ros-planning.github.io/moveit_tutorials/doc/quickstart_in_rviz/quickstart_in_rviz_tutorial.html)

## Demo 
```
roslaunch sema_gzsim sema_gzsim_moveit.launch demo:=true
```
## Tutorials
* [Lear how  to change simulator mode.]()

* [Lear how to spawn a platform in gazebo.]()

* [Lear how to spawn a box, or a sequence of boxes, in the simulation.]()

* [Lear how to control the conveyor belt.]()

* [Lear how to create an ideal or  a realistic attach between the vacuum gripper and a box.]()

* [Lear how to save reference points and robot joint states for a motion sequence project.]()

* [Lear how control the robot by Moveit! with Python3.]()

## Contact 

 * Cristian Nova Santoya <cristian.nova@uc.cl>
