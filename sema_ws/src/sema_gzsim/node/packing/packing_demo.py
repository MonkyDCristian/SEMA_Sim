#!/usr/bin/env python3

import rospy

from sema_gzsim.box_spawner import boxes_prms
from sema_gzsim.box_teleport import BoxTeleport
from sema_gzsim.packing_examples import PackingExample1, PackingExample2, PackingExample3, PackingExample4

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
