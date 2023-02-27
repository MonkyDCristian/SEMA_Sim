# Create a setup simulation file in python

If you want to develop or test code for this simulator, it is always best to have a predefined simulation environment according to what you want to develop.

A very clear example of this is when you want to replicate the conditions of a real environment (like the one in the following image) to ensure that you can apply the same codes that you tried in the simulated one:

![Alt text](/imgs/real_scene.jpeg)

In this space we have 4 objects, not considered in the Moveit! Scene, that we want to make sure that the robot doesn't crash: the floor, the pallet, the support of the base of the robot and of course the work area (the table on the left side with the computers and monitors)

***Note:*** all the robot parts that are added in the xacro description have already been added to the Moveit! scene.

It's time to use the tools from previous tutorials to build our environment. To do this you will need two files, a setup_sim.py and a setup_sim.launch. With setup_sim.py we'll use the classes we've learned to add an object to Moveit! and Gazaebo. The setup_sim.launch will launch the simulator and call setup_sim.py.

## Example for setup_sim.py
```
#!/usr/bin/env python3

import rospy

from sema_gzsim.box_spawner_act_clt import BoxSpawnerActClt
from sema_gzsim.conveyor_belt_vel_ctrl import ConveyorBeltVelocityCtrl
from sema_gzsim.pallet_spawner import PalletSpawner

from sema_moveit.move_group_python_interface import MoveGroupPythonInterface
from sema_moveit.obj2scene import Obj2Scene


class SetupSim(object):
	
	def __init__(self):
		rospy.init_node("setup_sim")
		self.variables_init()
		self.connections_init()
		self.run()
		

	def variables_init(self):
		self.cb_vel = 0.1
		self.palet_spawn_prms = {"name":"palet", "x":0.1, "y":0.75, "z":0.65, "yaw":0.0, "size":{"x":0.6, "y":0.9, "z":0.14}}
		self.box_spawn_prms = {"sequence":"m,m,m,m,m,m,m,m,m", "hz":0.15, "x":-2.8, "y":2.2, "z":0.8, "yaw":1.57}
		self.plane_prms = {"name":"floor", "z":0.0}
		
		self.robot_support_prms = {"name":"robot_support", "x":0.0, "y":0.0, "z":0.45, "yaw":0.0, "height":0.7, "radius":0.15}
		
		self.robot_box_prms = {"name": "robot_box", "x":0.0, "y":0.2, "z":0.45, "yaw":0.0, "size":{"x":0.3, "y":0.2, "z":0.25}}
		self.robot_base_prms = {"name": "robot_base", "x":0.0, "y":0.0, "z":0.05, "yaw":0.0, "size":{"x":0.8, "y":0.8, "z":0.1}}
		self.table_prms = {"name": "table", "x":0.0, "y":-2.0, "z":0.8, "yaw":0.0, "size":{"x":1.0, "y":2.0, "z":1.6}}



	def connections_init(self): 
		self.pallet_spawner = PalletSpawner()
		self.box_spawner_clt = BoxSpawnerActClt()
		self.cb_vel_ctrl = ConveyorBeltVelocityCtrl()
		
		self.mgpi = MoveGroupPythonInterface()
		self.mgpi.show_variable()

		self.obj2scene = Obj2Scene(self.mgpi)
	

	def run(self):
		# add colision objects to Moveit! PlanningScene
		self.obj2scene.add_palet(self.palet_spawn_prms)
		self.obj2scene.add_palet(self.robot_box_prms)
		self.obj2scene.add_palet(self.robot_base_prms)
		self.obj2scene.add_palet(self.table_prms)
		self.obj2scene.add_cylinder(self.robot_support_prms)
		self.obj2scene.add_plane(self.plane_prms)

		# start the conveyor belt 
		self.cb_vel_ctrl.run(self.cb_vel)

		# add palet to gazebo
		self.pallet_spawner.set_params(self.palet_spawn_prms)
		self.pallet_spawner.run()

		# add boxes to gazebo
		self.box_spawner_clt.set_params(self.box_spawn_prms)
		self.box_spawner_clt.run()


if __name__ == "__main__":
	sema_demo = SetupSim()
```
## Example for setup_sim.launch
```
<?xml version="1.0"?>
<launch>
  
  <arg name="vgc10_enabled"         default="true" />
  <arg name="conveyor_belt_enabled" default="true" />
  <arg name="oak_d_enabled"         default="true" />
  <arg name="gazebo_gui"            default="true" />

  
  <!-- SEMA sim Configuration --> 
  <include file="$(find sema_gzsim)/launch/sema_gzsim_moveit.launch">
	<arg name="vgc10_enabled"         value="$(arg vgc10_enabled)" />
	<arg name="conveyor_belt_enabled" value="$(arg conveyor_belt_enabled)" />
	<arg name="oak_d_enabled"         value="$(arg oak_d_enabled)" />  
	<arg name="gazebo_gui"            value="$(arg gazebo_gui)" />
  </include>
  
  <node name="setup_sim" pkg="sema_vision" type="setup_sim.py"/>
  
</launch>
```

## Test it
**T1:**
```
roslaunch sema_gzsim setup_sim.launch
```

![Alt text](/imgs/setup_sim.png)

At this point you have everything you need to start building a complete palletizing flow. Naturally, there will be three main areas of development: Perception, Packing, and Trajectory Planning. Perception will allow you to know the position and type of box that enters the system, packing will determine the target position on the pallet of the box given its type and trajectory planning will create and execute a safe route to move the box from its initial position towards the target.

A good practice that you are expected to apply is to develop these three areas separately. The next tutorials will give you very useful tools to quickly develop each of those areas.

## Next Tutorial 
[Fast palletizing by box teleportation and static positioning.](https://github.com/MonkyDCristian/SEMA_Sim/blob/main/documentation/box_teleport.md)
