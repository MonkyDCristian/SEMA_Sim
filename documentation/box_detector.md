#  Use a very simple image processing algorithm to detect boxes with the camera.

We have a powerful RGB-D camera that we could use to detect the position of the box on the conveyor, but creating that kind of algorithm is very time-consuming. Let's start with something easy that allows us to develop a palletizing workflow in no time.

Let's say we put the robot in a certain pose on the conveyor belt. In that pose the camera is able to see the boxes that are passing, which have a 90° angle just as they pass through the center of the camera's field of view.

If we can stop the conveyor belt just at the moment the box reaches the center of the image, then the box will be in an ideal position to be picked up.

So, what we need is a ROS node that processes the image given by the RGB-D camera, that is able to detect the scepter of the box in the image, stop the conveyor belt when the center of the box passes through the center of the image and publish a message reporting that there is a box in position to be picked up. It must also have a subscription topic to wake up the system.

![Alt text](/imgs/box_detector_system.png)

Detection can be achieved by applying a simple contour object detection algorithm with OpenCV. To do this, the codes present in this [tutorial](https://docs.opencv.org/4.x/da/d0c/tutorial_bounding_rects_circles.html) were adapted to the box_detector node.

[~/SEMA_Sim/sema_ws/src/sema_gzsim/node/vision/box_detector.py](https://github.com/MonkyDCristian/SEMA_Sim/blob/main/sema_ws/src/shttps://github.com/MonkyDCristian/SEMA_Sim/blob/main/sema_ws/src/sema_gzsim/node/vision/box_detector.pyema_gzsim/node/vision/box_detector.py)

## Test it
**T1:** launch the setup simulation for vision testing.
```
roslaunch sema_gzsim setup_sim_vision.launch
```
**T2:** visualize the box_detector output image.
```
rqt
```
go to plugin -> visualization -> image view, then check for /box_detector/color/img topic.

**T3:** reactivate the conveyor belt **after** detecting a box.
```
rostopic pub /box_detector/active std_msgs/Empty "{}" 
```

![Alt text](/imgs/box_detector.png)

## Next Tutorial 
[Develop a full palletizing system.](https://github.com/MonkyDCristian/SEMA_Sim/blob/main/documentation/palletizing_develop.md)
