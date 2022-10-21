#!/usr/bin/env python3

import rospy
import numpy as np

from geometry_msgs.msg import Pose, Twist, Vector3
from std_msgs.msg      import String
from gazebo_msgs.msg   import ModelStates


class BoxAttacher():
	def __init__(self):
		rospy.init_node('box_attacher')
		self.variables_init()
		self.connections_init()
		rospy.spin()
		

	def variables_init(self):
		
		self.posible_boxes = ["l", "ml", "m", "bm", "b"]
		self.dict_box = {"l":{"name":"sema_little_box"},
					     "ml":{"name": "sema_middle_little_box"},
						 "m":{"name": "sema_middle_box"},
					 	 "bm":{"name": "sema_big_middle_box"},
					  	 "b":{"name": "sema_big_box"}}
		
		self.box_type = ""
		self.box_name = ""
		self.box_seted = False


	def connections_init(self): 
		rospy.Subscriber("/gazebo/model_states", String, self.pub_move_data)
		rospy.Subscriber("move/box/type", String, self.set_box)
		

	def set_box(self, cmd):
		if cmd.data not in self.posible_boxes:
			self.box_seted = False
		
		else:
			self.box_seted = True
			self.box_type = cmd.data[:-1]
			self.box_name = self.dict_box[self.box_type] + cmd.data[-1]
	

	def pub_move_data(self, models_data):

		for n, model_name in enumerate(models_data.name):
			if model_name == self.box_name:
				box_pose = models_data.pose[n]
				box_twist = models_data.twist[n]
				break
		
		
				
	
	
			

if __name__ == '__main__':
	box_attacher = BoxAttacher()