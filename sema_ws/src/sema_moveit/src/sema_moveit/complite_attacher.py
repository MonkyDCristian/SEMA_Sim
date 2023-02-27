#!/usr/bin/env python3

"""
This class allows you to create an attachment between the end effector and the box. 
The attached file is generated in Moveit! and Gazebo.
When the box is releasing, the final position of the box can be set statically.
"""

import rospy

from sema_gzsim.box_attacher import BoxAttacher
from sema_gzsim.box_spawner import get_box_model_name
from sema_gzsim.box_teleport import BoxTeleport

from sema_moveit.move_group_python_interface import MoveGroupPythonInterface
from sema_moveit.obj2scene import Obj2Scene

from tf.transformations import euler_from_quaternion
from gazebo_msgs.msg import ModelStates

class CompliteAttacher(object):
	
	def __init__(self, mgpi=None):
		self.box_attacher = BoxAttacher()
		self.box_attacher.attach_params["ideal_attach"] =  True

		self.box_teleport = BoxTeleport()
		self.obj2scene = Obj2Scene(mgpi)

		self.update_time = 0.25


	def attach_box(self, dict_box):
		# create simulation attach
		self.box_attacher.run(dict_box["model"], dict_box["id"])
		
		if not dict_box["id"]: # get the id of the box if the attachment is not ideal
			dict_box["id"] = int(self.box_attacher.attach_params["obj_link"][-1])
		
		rospy.sleep(self.update_time) # wait for the simulation to update
		
		self.get_current_pose(dict_box) # update de dict_box position
		
		# create moveit! attach
		self.obj2scene.add_box(dict_box)
		self.obj2scene.attach_box(dict_box)
	

	def detach_box(self, dict_box):
		# detach in simulation 
		self.box_attacher.stop()
		
		rospy.sleep(self.update_time) # wait for the simulation to update
		
		self.get_current_pose(dict_box) # update de dict_box position

		# keep box position
		if "static" in dict_box:
			if dict_box["static"]:
				self.box_teleport.teleport(dict_box)
		
		# detach in moveit!
		self.obj2scene.detach_box(dict_box)


	def get_current_pose(self, dict_box):
		box_model, box_id = dict_box["model"], dict_box["id"]
		my_box_name = get_box_model_name(box_model, box_id)
		
		model_states_msg = rospy.wait_for_message("/box_poses", ModelStates, timeout=3)

		for n, box_name in enumerate(model_states_msg.name):
			if box_name == my_box_name:
				box_pose = model_states_msg.pose[n]
				dict_box["x"] = box_pose.position.x
				dict_box["y"] = box_pose.position.y
				dict_box["z"] = box_pose.position.z
				quat = (box_pose.orientation.x, box_pose.orientation.y, box_pose.orientation.z, box_pose.orientation.w)
				dict_box["roll"], dict_box["pitch"], dict_box["yaw"] = euler_from_quaternion(quat)
				break


if __name__ == "__main__":
	rospy.init_node("complite_attacher")

	mgpi = MoveGroupPythonInterface()
	mgpi.show_variable()
	
	complite_attacher = CompliteAttacher(mgpi)
	
	dict_box = {"model":"m", "id": 0, "x":0.0, "y":0.0, "z":0.0, "yaw":0.0, "static":True}
	complite_attacher.attach_box(dict_box)
	
	rospy.sleep(10)
	
	complite_attacher.detach_box(dict_box)



	





