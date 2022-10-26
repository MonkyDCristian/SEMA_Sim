#!/usr/bin/env python3

import rospy

from std_msgs.msg        import Float32
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class VGSimExtensionCtrl():
	def __init__(self):
		self.variables_init()
		self.connections_init()
		

	def variables_init(self):
		self.extension = 0
		self.create_jt_msg()


	def connections_init(self): 
		rospy.Subscriber("vg_sim_extension/cmd", Float32, self.callback)
		self.pub_vg_cmd =rospy.Publisher('/vgc_joint_traj_controller/command', JointTrajectory, queue_size=3)	
		
		while self.pub_vg_cmd.get_num_connections() == 0 and not rospy.is_shutdown():
			rospy.sleep(0.2)
		

	def create_jt_msg(self):
		self.jt_msg = JointTrajectory()
		self.jt_msg.header.frame_id = ""
		self.jt_msg.joint_names = ["sema/vgc10/extension_joint"]

		self.jtp_msg = JointTrajectoryPoint()
		self.jtp_msg.positions = [self.extension]
		self.jtp_msg.velocities = []
		self.jtp_msg.accelerations = []
		self.jtp_msg.effort = []
		self.jtp_msg.time_from_start = rospy.Duration(0.5)

		self.jt_msg.points.append(self.jtp_msg)
	

	def callback(self,extension_msg):
		self.run(extension_msg.data)


	def run(self, extension):
		self.extension = extension
		self.jt_msg.points[0].positions = [self.extension]
		self.jt_msg.header.stamp = rospy.Time.now()
		self.pub_vg_cmd.publish(self.jt_msg)


if __name__ == '__main__':
	rospy.init_node('vg_sim_extension_controller')
	vg_sim_extension_controller = VGSimExtensionCtrl()
	extension = 0.15 # mt
	vg_sim_extension_controller.run(extension)
	rospy.spin()