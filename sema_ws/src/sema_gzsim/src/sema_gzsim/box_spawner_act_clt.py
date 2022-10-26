#!/usr/bin/env python3

import rospy

from actionlib import SimpleActionClient
from sema_gzsim.msg import BoxSpawnerAction ,BoxSpawnerGoal

class BoxSpawnerActClt(object):

	def __init__(self):
		self.variables_init()
		self.connections_init()
		

	def variables_init(self):
		self.as_goal = BoxSpawnerGoal()
		
		self.as_goal.sequence = None
		self.as_goal.hz = None
		self.as_goal.x = None
		self.as_goal.y = None
		self.as_goal.z = None
		self.as_goal.yaw = None


	def connections_init(self): 
		self.client = SimpleActionClient("box_spawner_act_srv", BoxSpawnerAction)
		self.client.wait_for_server()
	

	def run(self):
		self.client.send_goal(self.as_goal)
		self.client.wait_for_result()
		print(self.client.get_result())
		
	
	def set_params(self, params):
		self.as_goal.sequence = params["sequence"]
		self.as_goal.hz = params["hz"]
		self.as_goal.x = params["x"]
		self.as_goal.y = params["y"]
		self.as_goal.z = params["z"]
		self.as_goal.yaw = params["yaw"]


if __name__ == "__main__":
	rospy.init_node("box_spawner_act_clt")
	box_spawner = BoxSpawnerActClt()
	
	box_sequence = ["ml", "ml", "ml"]
	spawn_params = {"sequence":"", "hz":1.0, "x":-0.6, "y":-0.3, "z":0.8, "yaw":0.0}

	for box_type in box_sequence:
		spawn_params["sequence"] = box_type
		
		box_spawner.set_params(spawn_params)
		box_spawner.run()

		spawn_params["y"] += 0.5
