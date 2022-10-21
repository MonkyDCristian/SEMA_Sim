#!/usr/bin/env python3

import rospy

from box_spawner_act_clt import BoxSpawnerAct
from conveyor_belt_vel_ctrl import ConveyorBeltVelocityCtrl
from pallet_spawner import PalletSpawner


class SemaDemo(object):
	
	def __init__(self):
		rospy.init_node("sema_demo")
		self.variables_init()
		self.connections_init()
		self.run()
		

	def variables_init(self):
		self.cb_vel = 0.1 
		self.pallet_spawn_prms = {"x":0.3, "y":0.8, "z":0.1, "yaw":0.0}
		self.box_spawn_prms = {"sequence":"l,ml,m,bm,b", "hz":0.2, "x":-2.8, "y":-2.2, "z":0.8, "yaw":0.0}


	def connections_init(self): 
		self.pallet_spawner = PalletSpawner()
		self.box_spawner = BoxSpawnerAct()
		self.cb_vel_ctrl = ConveyorBeltVelocityCtrl()


	def run(self):
		self.cb_vel_ctrl.run(self.cb_vel)

		self.pallet_spawner.set_spawn_params(self.pallet_spawn_prms)
		self.pallet_spawner.run()

		self.box_spawner.set_spawn_params(self.box_spawn_prms)
		self.box_spawner.run()


if __name__ == "__main__":
	sema_demo = SemaDemo()
