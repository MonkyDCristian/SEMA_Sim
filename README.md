# SEMA_Sim
Simulator of the Automated Mobile Packaging System developed at the "Universidad de Los Andes de Colombia".

![Alt text](/imgs/gzsim_rviz_view.png)

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
## Demo 
```
roslaunch sema_gzsim sema_gzsim_moveit.launch demo:=true
```
