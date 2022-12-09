#!/usr/bin/env python3

import rospy
import numpy as np

from std_msgs.msg      import String

# https://classic.gazebosim.org/tutorials?tut=ros_comm&cat=connect_ros
from gazebo_msgs.msg   import ModelStates
from gazebo_msgs.srv import GetLinkState, GetLinkStateRequest

from sema_gzsim.vg_sim_extension_ctrl import VGSimExtensionCtrl
from sema_gzsim.obj_attacher_act_clt import ObjAttacherActClt
from sema_gzsim.box_spawner import get_box_link_name


class BoxAttacher():
	def __init__(self):
		self.variables_init()
		self.connections_init()
		

	def variables_init(self):
		self.reference_frame  = "sema/vgc10/extension_link"

		self.dict_box = {"l":{"name":"sema_little_box", "extension": 0.075},
					     "ml":{"name": "sema_middle_little_box", "extension": 0.075},
						 "m":{"name": "sema_middle_box", "extension":0.09},
					 	 "bm":{"name": "sema_big_middle_box", "extension":0.14},
					  	 "b":{"name": "sema_big_box", "extension":0.215}}

		self.hz = 20
		self.rate = rospy.Rate(self.hz) #Hz
		
		self.box_model = ""
		self.dist_to_closer = 0.0

		self.extension_ctrl = VGSimExtensionCtrl()
		self.obj_attacher = ObjAttacherActClt()

		self.attach_params = {"obj_link":"", "ideal_attach": True}


	def connections_init(self): 
		self.get_link_state_srv = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
		rospy.Subscriber("attach/box/type", String, self.callback)
		

	def callback(self, cmd):
		if cmd.data == "stop":
			self.stop()	
		
		else:
			self.run(cmd.data)
	

	def run(self, box_model, box_id = False):
		
		self.box_model =  box_model
		self.extension_ctrl.run(self.dict_box[self.box_model]["extension"])
		rospy.sleep(0.5)

		print(box_id)
		
		if type(box_id) != bool:
			self.attach_params["obj_link"] = get_box_link_name(box_model, box_id)
		
		else:
			self.attach_params["obj_link"] = self.get_clouser_box_link()
		
		print(self.attach_params["obj_link"])

		self.obj_attacher.set_params(self.attach_params)
		self.obj_attacher.run()

	
	def stop(self):
		self.obj_attacher.stop()


	def get_clouser_box_link(self):
		model_states_msg = rospy.wait_for_message("/box_poses", ModelStates, timeout = 3)
		extension_state_msg = self.get_link_state_srv(GetLinkStateRequest(self.reference_frame, "world"))
		self.ex_st_pos = extension_state_msg.link_state.pose
		
		model_poses = []
		model_names = []

		for n, model_name in enumerate(model_states_msg.name):
			if self.dict_box[self.box_model]["name"] in model_name:
				model_poses.append(model_states_msg.pose[n])
				model_names.append(model_name)
		
		list_dist = np.array(list(map(self.get_dist, model_poses)))
		self.dist_to_closer = np.min(list_dist)

		closer_box_index = np.where(list_dist == self.dist_to_closer)[0][0]
		
		box_model = model_names[closer_box_index]
		return box_model[:-1] + "_link" + box_model[-1]
		

	def get_dist(self, model_pose):
		pose1 = self.ex_st_pos.position
		pose2 = model_pose.position
		return np.linalg.norm(np.array([pose2.x - pose1.x, pose2.y - pose1.y, pose2.z - pose1.z]))


if __name__ == '__main__':

	rospy.init_node('box_attacher')
	box_attacher = BoxAttacher()
	box_attacher.attach_params["ideal_attach"] =  True

	rospy.sleep(5)
	box_attach_sequence = ["l", "ml", "m", "bm", "b"]
	for box_model in box_attach_sequence:
		box_attacher.run(box_model)
		rospy.sleep(2)
		box_attacher.stop()