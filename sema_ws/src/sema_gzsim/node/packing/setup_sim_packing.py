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
		self.setup()
		

	def variables_init(self):
		self.cb_vel = 0.0
		self.pallet_spawn_prms = {"name":"pallet1","x":0.1, "y":0.75, "z":0.65, "yaw":0.0, "size":{"x":0.6, "y":0.9, "z":0.14}}
		self.box_spawn_prms = {"sequence":"m,m,m,m,m,m,m,m,m", "hz":5.0, "x":-2.8, "y":-2.2, "z":0.8, "yaw":1.57}
		
		# object in the scene
		self.plane_prms = {"name":"floor", "z":0.0}
		self.robot_support_prms = {"name":"robot_support", "x":0.0, "y":0.0, "z":0.45, "yaw":0.0, "height":0.7, "radius":0.15}
		self.robot_box_prms = {"name": "robot_box", "x":0.0, "y":0.2, "z":0.45, "yaw":0.0, "size":{"x":0.3, "y":0.2, "z":0.25}}
		self.robot_base_prms = {"name": "robot_base", "x":0.0, "y":0.0, "z":0.05, "yaw":0.0, "size":{"x":0.8, "y":0.8, "z":0.1}}
		self.workspace_prms = {"name": "workspace", "x":0.0, "y":-2.0, "z":0.8, "yaw":0.0, "size":{"x":1.0, "y":2.0, "z":1.6}}


	def connections_init(self): 
		self.pallet_spawner = PalletSpawner()
		self.box_spawner = BoxSpawnerActClt()
		self.cb_vel_ctrl = ConveyorBeltVelocityCtrl()

		self.obj2scene = Obj2Scene()
		self.obj2scene.mgpi = MoveGroupPythonInterface()
		self.obj2scene.mgpi.show_variable()


	def setup(self):
		# add colision objects to Moveit! PlanningScene
		self.obj2scene.add_cube(self.pallet_spawn_prms)
		self.obj2scene.add_plane(self.plane_prms)
		self.obj2scene.add_cylinder(self.robot_support_prms)
		self.obj2scene.add_cube(self.robot_box_prms)
		self.obj2scene.add_cube(self.robot_base_prms)
		self.obj2scene.add_cube(self.workspace_prms)

		# start the conveyor belt 
		self.cb_vel_ctrl.run(self.cb_vel)

		# add palet to gazebo
		self.pallet_spawner.set_params(self.pallet_spawn_prms)
		self.pallet_spawner.run()

		# add boxes to gazebo
		for n, box_type in enumerate(self.box_spawn_prms["sequence"].split(",")):
			prms = self.box_spawn_prms.copy()
			prms["sequence"] = box_type 
			prms["y"] += n/2
			self.box_spawner.set_params(prms)
			self.box_spawner.run()


if __name__ == "__main__":
	setup_sim = SetupSim()
