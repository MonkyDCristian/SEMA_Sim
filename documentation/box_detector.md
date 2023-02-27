#  Use a very simple image processing algorithm to detect box with the camera.

We have a powerful RGB-D camera that we could use to detect the position of the box on the conveyor, but creating that kind of algorithm is very time consuming. Let's start with something easy that allows us to develop a palletizing workflow in no time.

Let's say we put the robot in a certain pose on the conveyor belt, in that pose the camera is able to see the boxes that are passing, which have a 90° angle just as they pass through the center of the camera's field of view.

If we can stop the conveyor belt just at the moment the box reaches the center of the image, then the box will be in an ideal position to be picked up.

So, what we need is a ROS node that processes the image given by the RGB-D camera, that is able to detect the scepter of the box in the image, stop the conveyor belt when the center of the box passes through the center of the image and post a message reporting that there is a box in position to be picked up.

![Alt text](/imgs/box_detector_system.png)


[~/SEMA_Sim/sema_ws/src/]()

## Use example
```
#!/usr/bin/env python3

```

## Test it
**T1:**
```
roslaunch sema_gzsim setup_sim_vision.launch
```
**T2:**
```
rostopic pub /box_detector/active std_msgs/Empty "{}" 
```

![Alt text](/imgs/box_detector.png)

## Next Tutorial 
[Develop a full palletizing system (with a demo).](https://github.com/MonkyDCristian/SEMA_Sim/blob/ROS-focus-develop/documentation/palletizing_develop.md)
