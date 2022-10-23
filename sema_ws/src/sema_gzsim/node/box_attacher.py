#!/usr/bin/env python3

import rospy
import numpy as np

from geometry_msgs.msg import Pose, Twist, Vector3
from std_msgs.msg      import String
from gazebo_msgs.msg   import ModelStates

from gazebo_msgs.srv import GetLinkState, GetLinkStateRequest
from gazebo_msgs.srv import GetLinkProperties, GetLinkPropertiesRequest
from gazebo_msgs.srv import SetLinkState, SetLinkStateRequest
from gazebo_msgs.srv import SetLinkProperties, SetLinkPropertiesRequest


class BoxAttacher():
	def __init__(self):
		rospy.init_node('box_attacher')
		self.variables_init()
		self.connections_init()
		rospy.spin()
		

	def variables_init(self):
		self.model_states_tc = "/gazebo/model_states" 

		self.reference_frame  = "sema/vgc10/extension_link"

		self.get_link_properties_req = GetLinkPropertiesRequest()
		self.get_link_properties_req.link_name = ""

		self.get_link_state_req = GetLinkStateRequest()
		self.get_link_state_req.link_name = ""
		self.get_link_state_req.reference_frame = self.reference_frame 

		self.set_link_state_req = SetLinkStateRequest()
		self.set_link_state_req.link_state.link_name = ""
		self.set_link_state_req.link_state.pose = Pose()
		self.set_link_state_req.link_state.twist = Twist()
		self.set_link_state_req.link_state.reference_frame = self.reference_frame 

		self.set_link_properties_req = SetLinkPropertiesRequest()
		self.set_link_properties_req.link_name = ""
		self.set_link_properties_req.com = Pose()
		self.set_link_properties_req.gravity_mode = True
		self.set_link_properties_req.mass = 0
		self.set_link_properties_req.ixx = 0 
		self.set_link_properties_req.ixy = 0
		self.set_link_properties_req.ixz = 0
		self.set_link_properties_req.iyy = 0
		self.set_link_properties_req.iyz = 0
		self.set_link_properties_req.izz = 0
		
		self.dict_box = {"l":{"name":"sema_little_box", "dist": 0.075},
					     "ml":{"name": "sema_middle_little_box", "dist":0.075},
						 "m":{"name": "sema_middle_box", "dist":0.09},
					 	 "bm":{"name": "sema_big_middle_box", "dist":0.14},
					  	 "b":{"name": "sema_big_box", "dist":0.215}}

		self.hz = 20
		self.rate = rospy.Rate(self.hz) #Hz
		self.box_name = ""
		self.box_type = ""
		self.dist_to_closer = 0.0
		self.ideal_attach = False
		self.box_attached = False
		self.vg_box_relative_pose = None


	def connections_init(self): 
		self.get_link_properties_srv = rospy.ServiceProxy('/gazebo/get_link_properties', GetLinkProperties)
		self.get_link_state_srv = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
		self.set_link_properties_srv = rospy.ServiceProxy('/gazebo/set_link_properties', SetLinkProperties)
		self.set_link_state_srv = rospy.ServiceProxy('/gazebo/set_link_state', SetLinkState)
		
		rospy.Subscriber("attach/box/type", String, self.callback)
		

	def callback(self, cmd):
		if cmd.data == "stop":
			self.box_attached = False
		
		else:
			self.box_type = cmd.data
			self.get_box_name()

			self.create_attach()
	

	def get_box_name(self):
		model_states_msg = rospy.wait_for_message(self.model_states_tc, ModelStates, timeout = 3)
		
		extension_state_msg = self.get_link_state_srv(GetLinkStateRequest(self.reference_frame, "world"))
		self.ex_st_pos = extension_state_msg.link_state.pose
		
		model_poses = []
		model_names = []

		for n, model_name in enumerate(model_states_msg.name):
			if self.dict_box[self.box_type]["name"] in model_name:
				model_poses.append(model_states_msg.pose[n])
				model_names.append(model_name)
		
		list_dist = np.array(list(map(self.get_dist, model_poses)))
		self.dist_to_closer = np.min(list_dist)

		closer_box_index = np.where(list_dist == self.dist_to_closer)[0][0]
		self.box_name = model_names[closer_box_index]
		

	def get_dist(self, model_pose):
		pose1 = self.ex_st_pos.position
		pose2 = model_pose.position
		return np.linalg.norm(np.array([pose2.x - pose1.x, pose2.y - pose1.y, pose2.z - pose1.z]))
		
	
	def run(self, extension, enable):
		pass


	def create_attach(self):

		res = self.costum_and_send_set_properties_req(gravity=False)

		if res.success:
			self.box_attached = True
			self.costum_set_state_req()

			#keep attach
			while not rospy.is_shutdown() and self.box_attached:
				res = self.set_link_state_srv(self.set_link_state_req) 
				
				if not res.success:
					print(res)
				
				self.rate.sleep()
			
			self.costum_and_send_set_properties_req(gravity=True)
		
		else: 
			print(res)

	
	def costum_set_state_req(self):
		self.set_link_state_req.link_state.link_name = self.box_name[:-1] + "_link" + self.box_name[-1]
		self.get_vg_box_relative_pose()

		if self.ideal_attach:
			self.set_link_state_req.link_state.pose = self.vg_box_relative_pose
		
		else:
			self.set_link_state_req.link_state.pose = Pose()
		
		twist_msg = Twist()
		twist_msg.linear  = Vector3(0,0,0)
		twist_msg.angular = Vector3(0,0,0)

		self.set_link_state_req.link_state.twist = twist_msg


	def get_vg_box_relative_pose(self):	
		res = self.costum_and_send_get_state_req()
		
		if res.success:
			self.vg_box_relative_pose = res.link_state.pose

		else: 
			print(res)


	def costum_and_send_get_state_req(self):
		self.get_link_state_req.link_name = self.box_name[:-1] + "_link" + self.box_name[-1]
		return self.get_link_state_srv(self.get_link_state_req) 
	
	
	def costum_and_send_set_properties_req(self, gravity=False):
		self.get_link_properties_req.link_name = self.box_name[:-1] + "_link" + self.box_name[-1]
		link_properties_msg = self.get_link_properties_srv(self.get_link_properties_req)

		self.set_link_properties_req.link_name = self.box_name[:-1] + "_link" + self.box_name[-1]
		self.set_link_properties_req.gravity_mode = gravity
		self.set_link_properties_req.mass = link_properties_msg.mass
		self.set_link_properties_req.ixx = link_properties_msg.ixx
		self.set_link_properties_req.iyy = link_properties_msg.iyy
		self.set_link_properties_req.izz = link_properties_msg.izz

		return self.set_link_properties_srv(self.set_link_properties_req)


if __name__ == '__main__':
	box_attacher = BoxAttacher()