#  Use a very simple image processing algorithm to detect box with the camera.

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
