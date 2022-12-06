#!/usr/bin/env python3

import rospy

from sema_gzsim.box_spawner_act_clt import BoxSpawnerActClt
from sema_gzsim.conveyor_belt_vel_ctrl import ConveyorBeltVelocityCtrl
from sema_gzsim.pallet_spawner import PalletSpawner


class SemaDemo(object):
	
	def __init__(self):
		rospy.init_node("sema_demo")
		self.variables_init()
		self.connections_init()
		self.run()
		

	def variables_init(self):
		self.cb_vel = 0.1 
		self.pallet_spawn_prms = {"x":0.1, "y":0.6, "z":0.5, "yaw":0.0, "size_x":0.6, "size_y":0.9}
		self.box_spawn_prms = {"sequence":"l,ml,m,bm,b", "hz":0.2, "x":-2.8, "y":-2.2, "z":0.8, "yaw":0.0}


	def connections_init(self): 
		self.pallet_spawner = PalletSpawner()
		self.box_spawner = BoxSpawnerActClt()
		self.cb_vel_ctrl = ConveyorBeltVelocityCtrl()


	def run(self):
		self.cb_vel_ctrl.run(self.cb_vel)

		self.pallet_spawner.set_params(self.pallet_spawn_prms)
		self.pallet_spawner.run()

		self.box_spawner.set_params(self.box_spawn_prms)
		self.box_spawner.run()


if __name__ == "__main__":
	sema_demo = SemaDemo()
