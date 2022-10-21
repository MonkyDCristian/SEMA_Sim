#!/usr/bin/env python3

import rospy

#from std_msgs.msg import Float64
from actionlib import SimpleActionClient
from sema_gzsim.msg import BoxSpawnerAction ,BoxSpawnerGoal

class BoxSpawnerAct(object):

	def __init__(self) -> None:
		rospy.init_node("box_spawner_act")
		self.variables_init()
		self.connections_init()
		self.run()
		

	def variables_init(self):
		
		self.box_sequence = ["l","ml","m"]

		self.as_goal = BoxSpawnerGoal()
		
		self.as_goal.hz = 1.0
		self.as_goal.x = -0.6
		self.as_goal.y = -0.3
		self.as_goal.z = 0.8
		self.as_goal.yaw = 0.0


	def connections_init(self): 

		#self.pub_cb_cmd = rospy.Publisher("/cb_joint_vel_controller/command", Float64, queue_size=1)
		self.client = SimpleActionClient(rospy.get_name() + "_srv", BoxSpawnerAction)
		self.client.wait_for_server()
	

	def run(self):
		
		for box in self.box_sequence:
			self.as_goal.sequence = box
			
			self.client.send_goal(self.as_goal)
			self.client.wait_for_result()
			print(self.client.get_result())
			self.as_goal.y += 0.5
			self.as_goal.yaw += 0.3


if __name__ == "__main__":
	box_spawner = BoxSpawnerAct()
		