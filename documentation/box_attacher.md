# Box Attacher
The BoxAttacher class simulates the suction effect of the vacuum gripper. The algorithm does not apply any force to the box, what happens is that we use [Gazebo ROS services](https://classic.gazebosim.org/tutorials?tut=ros_comm&cat=connect_ros) to teleport the box to a relative position to the end effector robot arm link (eef_link).

There are two types of attach you can use, an **ideal attach** and a **relative attach**. The ideal attach places the box in a standard position relative to the eef_link, depending on the type of the box that you want to attach, while the relative attach places the box in the box's position relative to the eef_link at the time that the attach was generated.

The BoxSpawner class is the combination of two classes, VGSimExtensionCtrl and ObjAttacherActClt. The VGSimExtensionCtrl controls the position of a little box relative to the eef_link, this box doesn't have collisions and it is the point where the box is teleported when we create an ideal attach. The ObjAttacherActClt works to call ObjAttacherActSrv, that is a [ROS action server](http://wiki.ros.org/actionlib) that creates and maintains a connection between eef_link and a free box object model, its goal message is given by the name of the free model's parent link and a boolean value, this last one indicate whether the connection is ideal or not.

[~/SEMA_Sim/sema_ws/src/sema_gzsim/src/sema_gzsim/box_attacher.py](https://github.com/MonkyDCristian/SEMA_Sim/blob/main/sema_ws/src/sema_gzsim/src/sema_gzsim/box_attacher.py)

**Note:** part of the algorithm was inspired by this [blog](https://erdalpekel.de/?p=178).

### VGSimExtensionCtrl

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

## Create an attach by code

To create an attachment with the BoxAttacher class, you just need to set whether the attachment is ideal or not by setting the parameter **box_attacher.attach_params["ideal_attach"]**. Then use **box_attacher.run(box_type)**, with box_type the arbitration name of the box type that you want to attach. It will automatically create the attach between eef_link and the closest box with the given type. To stop the attach use **box_attacher.stop()**.

```
#!/usr/bin/env python3

import rospy
from sema_gzsim.box_spawner_act_clt import BoxAttacher

rospy.init_node('box_attacher')
box_attacher = BoxAttacher()
box_attacher.attach_params["ideal_attach"] =  True

rospy.sleep(5)
box_attach_sequence = ["l", "ml", "m", "bm", "b"]

for box_type in box_attach_sequence:
  box_attacher.run(box_type)
  rospy.sleep(2)
  box_attacher.stop()
```
### Test it
**T1:**
```
roslaunch sema_gzsim sema_gzsim.launch oak_d_enabled:=false demo:=true          
```
**T2:**
```
rosrun sema_gzsim box_attacher.py
```

### Ideal Attach Example

![Alt text](/imgs/ideal_attach.png)

### Ideal Attach Example
To test a relative attach, simply switch box_attacher.attach_params["ideal_attach"] value to False.

![Alt text](/imgs/not_ideal_attach.png)

## Next Tutorial

[Save reference points and robot joint states for a motion sequence project.](https://github.com/MonkyDCristian/SEMA_Sim/blob/main/documentation/save_robot_pose.md)
