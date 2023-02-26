# Adding objects to Moveit! planning scene.

In the previous tutorial, you learned how to plan and execute movement paths through the [MoveGroupPythonInterface](https://github.com/MonkyDCristian/SEMA_Sim/blob/ROS-focus-develop/sema_ws/src/sema_moveit/src/sema_moveit) class.

Now, suppose you want to run a palletizing mission. Consider that you have two boxes, one red and one blue, the mission will be to move the red box to the right side of the blue box, as shown in the image:

![Alt text](/imgs/simple_mision.png)

To accomplish this mission we have to create a movement trajectory for the red box that avoids collisions with the blue box, rather Moveit! has to create it.

For this we have to consider the collision space of the moving box, the red one, and the static box, the blue one. We can do this by adding the position and size of these objects to the virtual space of Moveit!, known as [PlanningScene](http://docs.ros.org/en/noetic/api/moveit_core/html/classplanning__scene_1_1PlanningScene.html), by the [PlanningSceneInterface](http://docs.ros.org/en/noetic/api/moveit_commander/html/classmoveit__commander_1_1planning__scene__interface_1_1PlanningSceneInterface.html).

You will see that it is possible to perform this task using the MoveGroupPythonInterface functions but to facilitate this task the Obj2Scene class was created, this allows you to add objects and figures intuitively to the PlanningScene.

[~/SEMA_Sim/sema_ws/src/sema_moveit/src/sema_moveit/obj2scene.py]()

## Use example
```
#!/usr/bin/env python3

from sema_moveit.move_group_python_interface import MoveGroupPythonInterface
from sema_moveit.obj2scene import Obj2Scene

rospy.init_node("obj2scene")

mgpi = MoveGroupPythonInterface()
mgpi.show_variable()

obj2scene = Obj2Scene(mgpi)

dict_box = {"model":"m", "id":0, "x":0.0, "y":-0.3, "z":0.8, "yaw":0.0}
dict_palet = {"x":0.1, "y":0.6, "z":0.5, "yaw":0.0, "size":{"x":0.6, "y":0.9, "z":0.14}}

obj2scene.add_palet(dict_palet)
rospy.sleep(4)

obj2scene.add_box(dict_box)
rospy.sleep(4)

obj2scene.attach_box(dict_box)
rospy.sleep(4)

obj2scene.detach_box(dict_box)
```
***Note:*** Notice that you have to import and instantiate the MoveGroupPythonInterface(mgpi) class to declare your Obj2Scene object. You also have to initialize the mgpi object by the show_variable() funtion before use any funtion of the Obj2Scene object.

## Test it
**T1:**
```
roslaunch sema_gzsim sema_gzsim_moveit.launch
```
**T2**
```
rosrun sema_moveit obj2scene.py
```

## Next Tutorial 
[Create a setup environmental file in python.](https://github.com/MonkyDCristian/SEMA_Sim/blob/ROS-focus-develop/documentation/setup_env.md)
