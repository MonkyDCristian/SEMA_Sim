# Setup and use a simulated depth camera.
Congratulations, you've reached the last of the basic level tutorials. Recalling what you have learned so far, we can say that you already know how to move the robot using a GUI, how to include boxes and pallets in the simulation environment, how to grab boxes, and how to record the robot's poses by measuring value. of the joints

In this tutorial we will learn how to use a [depth camera](https://thesweetcamera.com/what-is-depth-camera/) through a [Gazebo plugin](https://classic.gazebosim.org/tutorials?tut=ros_gzplugins), the missing piece in the system that will allow us to detect incoming boxes and perform a palletizing mission using manual (non-autonomous) human control.

First of all let's think about why we really need a depth camera, obviously it can be used to detect a dummy box and get its position through some kind of complicated algorithm, but actually we can access to each positions and orientations object in real time by subscribing to the [Gazebo](http://wiki.ros.org/gazebo) topic ["gazebo/model_states"](http://docs.ros.org/en/api/gazebo_msgs/html/msg/ModelStates.html) or request a specific object information via the [gazebo/get_model_state](http://docs.ros.org/en/api/gazebo_msgs/html/srv/GetModelState.html) service.

Note: The "gazebo/model_states" topic is used to display the box frames in RVIZ via the [Markers](http://wiki.ros.org/rviz/DisplayTypes/Marker).

image of markers


[~/SEMA_Sim/sema_ws/src/]()

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
