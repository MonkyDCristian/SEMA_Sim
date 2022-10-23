#!/usr/bin/env python3

import rospy

from nav_msgs.msg      import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg      import String
from gazebo_msgs.msg   import ModelStates


class ShowMoveData():
	def __init__(self):
		rospy.init_node('show_move_data')
		self.variables_init()
		self.connections_init()
		rospy.spin()
		

	def variables_init(self):
		
		self.model_name = ""
		self.model_seted = False


	def connections_init(self): 
		rospy.Subscriber("/show_move_data/model_name", String, self.set_box)
		rospy.Subscriber("/gazebo/model_states", ModelStates, self.publish_move_data)
		self.pub_move_data = rospy.Publisher("/move/data/twist", Twist, queue_size=2)
		

	def set_box(self, cmd):
		
		self.model_seted = True
		self.model_name = cmd.data

	def publish_move_data(self, models_data):

		if self.model_seted:
			
			for n, model_name in enumerate(models_data.name):
				if model_name == self.model_name:
					move_data_twist = models_data.twist[n]
					self.pub_move_data.publish(move_data_twist)
					break


if __name__ == '__main__':
	show_move_data = ShowMoveData()