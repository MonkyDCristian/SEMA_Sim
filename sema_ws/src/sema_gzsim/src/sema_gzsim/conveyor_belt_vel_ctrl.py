#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64


class ConveyorBeltVelocityCtrl(object):

	def __init__(self):
		self.variables_init()
		self.connections_init()
		

	def variables_init(self):
		self.msg = Float64() #rad/s


	def connections_init(self): 
		self.pub_cb_cmd = rospy.Publisher("/cb_joint_vel_controller/command", Float64, queue_size=5)
		
		while self.pub_cb_cmd.get_num_connections() == 0 and not rospy.is_shutdown():
			rospy.sleep(0.2)
		                                 

	def run(self, velocity):
		self.msg.data = velocity
		self.pub_cb_cmd.publish(self.msg)
	

if __name__ == "__main__":
	rospy.init_node("conveyor_belt_velocity_ctrl")
	cb_vel_ctrl = ConveyorBeltVelocityCtrl()
	
	vel = 0.1 #rad/s
	cb_vel_ctrl.run(vel)
		