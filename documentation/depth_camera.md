# Setup and use a simulated depth camera.
Congratulations, you've reached the last of the basic level tutorials. Recalling what you have learned so far, we can say that you already know how to move the robot using a GUI, how to include boxes and pallets in the simulation environment, how to grab boxes, and how to record the robot's poses by measuring value. of the joints

In this tutorial we will learn how to use a [depth camera](https://thesweetcamera.com/what-is-depth-camera/) through a [Gazebo plugin](https://classic.gazebosim.org/tutorials?tut=ros_gzplugins), the missing piece in the system that will allow us to detect incoming boxes and perform a palletizing mission using manual (non-autonomous) human control.

First of all let's think about why we really need a depth camera, obviously it can be used to detect a dummy box and get its position through some kind of complicated algorithm, but actually we can access to each position and orientation object, in real time, by subscribing to the [Gazebo](http://wiki.ros.org/gazebo) topic ["gazebo/model_states"](http://docs.ros.org/en/api/gazebo_msgs/html/msg/ModelStates.html) or request a specific object information via the [gazebo/get_model_state](http://docs.ros.org/en/api/gazebo_msgs/html/srv/GetModelState.html) service.

**Note:** The "gazebo/model_states" topic is used to display the box frames in RVIZ via the [ROS Markers](http://wiki.ros.org/rviz/DisplayTypes/Marker).

![Alt text](/imgs/.png)

[~/SEMA_Sim/sema_ws/src/](https://github.com/MonkyDCristian/SEMA_Sim/blob/main/sema_ws/src/sema_gzsim/node/publish_markers.py)

Knowing that simulating the camera requires more processing power, why would we insist on including it? Why not just use the Gazebo interface?
Well, this is explained by the objective of the simulator, which is to serve as a complete development environment for a real palletizing system. We won't have the poses of the boxes given by gazebo in the field, so we won't be able to use them to develop the full palletizing flow.

**Note:** It is good for you to know that there are simple and low consumption means to obtain the poses of the boxes. Think of the case where you are only interested in developing algorithms to transport boxes from one place to another, for this you don't have to spend computing power detecting the box by vision, you could access it through the "gazebo/get_model_state" service.

Let's start with this tutorial. The first thing is to consider what type of camera we are going to simulate, as mentioned before, the idea is that the system corresponds to reality. So if we are going to simulate a camera, it is most appropriate for us to have the same properties as the camera we plan to use in the field. [Luxonis](https://shop.luxonis.com/) brand have the OpenCV AI kits depth (OAK-D), models of cameras that support ROS and have a very good framework for developing computers vision algorithms, this is perfect for this project. Of these types of camera we will simulate the most recent model, the [OAK-D-PRO](https://shop.luxonis.com/collections/usb/products/oak-d-pro), we will integrate it and evaluate it in our virtual gazebo environment to know if it is suitable or not for this palletizing system.

## Use example

```
#!/usr/bin/env python3
```

## Test it
**T1:**
```

```
**T2**
```

```

## Next Tutorial 
[Control the robot by Moveit! with Python3.](https://github.com/MonkyDCristian/SEMA_Sim/blob/ROS-focus-develop/documentation/moveit.md)
