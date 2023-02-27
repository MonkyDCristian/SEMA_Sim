#!/usr/bin/env python3

import rospy

from gazebo_msgs.srv import DeleteModel
from sema_gzsim.box_spawner import BoxSpawner, get_box_model_name

class BoxTeleport(object):
	
	def __init__(self):
		self.variables_init()
		self.connections_init()


	def variables_init(self):
		self.box_spawner = BoxSpawner()
	

	def connections_init(self): 
		rospy.wait_for_service("/gazebo/delete_model")
		self.del_model_srv = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
	

	def teleport(self, dict_box):

		box_name = get_box_model_name(dict_box["model"], dict_box["id"])
		res = self.del_model_srv(box_name)
		
		print(res.status_message)
		rospy.sleep(0.25) # wait for the box to be deleted
		
		if res.success:
			self.box_spawner.set_pose_target(dict_box["x"], dict_box["y"], dict_box["z"], dict_box["yaw"], static = dict_box["static"])
			self.box_spawner.spawn(dict_box["model"], dict_box["id"])


if __name__ == "__main__":
	rospy.init_node("box_teleport")
	box_teleport = BoxTeleport()
	dict_box = {"model":"m", "id":0, "x":0.0, "y":-0.3, "z":0.8, "yaw":0.0, "static":True}
	box_teleport.teleport(dict_box)
