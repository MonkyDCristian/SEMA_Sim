# Fast palletizing by box teleportation and static positioning.

In this tutorial you will learn to use the BoxTeleport class, with this class you will be able to move any type of box, given its model and id, to a certain position and rotation. In addition, you will be able to establish the final position of the box as static.

This class is intended to facilitate testing of a packing algorithm.

[~/SEMA_Sim/sema_ws/src/sema_gzsim/src/sema_gzsim/box_teleport.py](https://github.com/MonkyDCristian/SEMA_Sim/blob/ROS-focus-develop/sema_ws/src/sema_gzsim/src/sema_gzsim/box_teleport.py)

## Use example
```
#!/usr/bin/env python3

from sema_gzsim.box_teleport import BoxTeleport

rospy.init_node("box_teleport")

box_teleport = BoxTeleport()
dict_box = {"model":"m", "id":0, "x":0.0, "y":-0.3, "z":0.8, "yaw":0.0, "static":True}
box_teleport.teleport(dict_box)
```

## Test it
**T1**
```
roslaunch sema_gzsim setup_sim.launch
```
**T2**
```
rosrun sema_gzsim box_teleport.py
```
## Testing a packing algorithm

There are already (4 different packing algorithms)[https://github.com/MonkyDCristian/SEMA_Sim/blob/ROS-focus-develop/sema_ws/src/sema_gzsim/src/sema_gzsim/packing_examples.py] for palletizing 9 medium model boxes. Below is an example of how to test a sample packing algorithm using the BoxTeleport class.

```
#!/usr/bin/env python3

import rospy

from sema_gzsim.box_spawner import boxes_prms
from sema_gzsim.box_teleport import BoxTeleport
from sema_gzsim.packing_examples import PackingExample1 # PackingExample2, PackingExample3, PackingExample4

rospy.init_node("packing_demo")

packing = PackingExample1() # PackingExample{1/2/3/4}()

packing.palet_prms = {"x":0.1, "y":0.75, "z":0.65, "yaw":0.0, "size":{"x":0.6, "y":0.9, "z":0.14}}
packing.target = {"model":"m", "id":0, "x":0.0, "y":0.0, "z":0.0, "yaw":0.0, "static":True}
packing.box_prms = boxes_prms[packing.target["model"]]
packing.offset = 0.001

packing.set_first_box_pose()
box_teleport = BoxTeleport()

for count in range(9):
	packing.target["id"] = count
	box_teleport.teleport(packing.target)
	packing.get_next_pose()
```

## Test it
**T1:** setup sumulation for packing
```
roslaunch sema_gzsim setup_sim_packing.launch
```
**T2**
```
rosrun sema_gzsim packing_demo.py
```

## Next Tutorial 
[Develop and test a trajectory algorithm.](https://github.com/MonkyDCristian/SEMA_Sim/blob/ROS-focus-develop/documentation/moveit_attacher.md)
