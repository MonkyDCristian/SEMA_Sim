#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64


class ConveyorBeltVelocityCtrl(object):

	def __init__(self) -> None:
		rospy.init_node("conveyor_belt_velocity_ctrl")
		self.variables_init()
		self.connections_init()
		self.run()
		

	def variables_init(self):
		
		self.vel = Float64(0.1) #rad/s


	def connections_init(self): 

		self.pub_cb_cmd = rospy.Publisher("/cb_joint_vel_controller/command", Float64, queue_size=5)
		
		while self.pub_cb_cmd.get_num_connections() == 0 and not rospy.is_shutdown():
			rospy.sleep(0.2)
		                                 

	def run(self):
		#while not rospy.is_shutdown():
		self.pub_cb_cmd.publish(self.vel)
	

if __name__ == "__main__":
	box_spawner = ConveyorBeltVelocityCtrl()
		