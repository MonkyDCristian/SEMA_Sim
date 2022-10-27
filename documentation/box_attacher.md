# Box Attacher
The BoxAttacher class simulates the suction effect of the vacuum gripper, in reality the algorithm does not apply any force to the box, what happens is that we use [Gazebo ROS services](https://classic.gazebosim.org/tutorials?tut=ros_comm&cat=connect_ros) to teleport the box to a relative position of the end effector robot arm link (eef_link).

There are two types of attach you can use, an **ideal attach** and a **realistic attach**. The ideal attach places the box in a standard position relative to the eef_link, depending on the type of the box that you want to attach, while the realistic attachment positions the box in the box's position relative to the eef_link at the time that the attach was generated.

**Note:** part of the algorithm was inspired by this [blog](https://erdalpekel.de/?p=178).

## Create an ideal attach by code
The BoxSpawner class is the combination of two classes, VGSimExtensionCtrl and ObjAttacherActSrv. The VGSimExtensionCtrl control the position of a little box relative to the eef_link, this box doesn't have collisions and it is the point where the box is teleport when we create an ideal attach. ObjAttacherActSrv is a [ROS action](http://wiki.ros.org/actionlib) that creates and maintains a connection between eef_link and a free object model, its goal message is given by the name of the free model's parent link and a boolean value, this last one indicate whether the connection is ideal or not.



### Understanding VGSimExtensionCtrl

![Alt text](/imgs/eef_extension.png)

The VGSimExtensionCtrl class apply a [Position Joint trajectory contorller](http://wiki.ros.org/joint_trajectory_controller), so it is easy to operate by the rqt_joint_trajectory_controller GUI. 

### Test it
**T1:**
```
roslaunch sema_gzsim sema_gzsim.launch conveyor_belt_enabled:=false oak_d_enabled:=false          
```
**T2:**
```
rosrun rqt_joint_trajectory_controller rqt_joint_trajectory_controller
```
**Note:** Run this command in case you don't have rqt_joint_trajectory_controller pkg installed
```
sudo apt install ros-noetic-rqt-joint-trajectory-controller
```

![Alt text](/imgs/ideal_attach.png)


![Alt text](/imgs/not_ideal_attach.png)
