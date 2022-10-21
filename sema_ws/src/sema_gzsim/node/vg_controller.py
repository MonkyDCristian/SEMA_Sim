#!/usr/bin/env python3

import rospy
import time

from std_msgs.msg import Header, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from sema_gzsim.cfg import vgc10_controlConfig 
from dynamic_reconfigure.server import Server as DRServer

class VGSimController():
	def __init__(self):
		rospy.init_node('vg_controller')
		self.variables_init()
		self.connections_init()
		rospy.spin()
		

	def variables_init(self):
		self.header = Header()
		self.extension = 0
		self.rate = rospy.Rate(10) #Hz
		self.create_jt_msg()


	def connections_init(self): 

		self.pub_vgc_cmd =rospy.Publisher('/vgc_joint_traj_controller/command', JointTrajectory, queue_size=3)	
		
		rospy.Subscriber("sim_vg/cmd", String, self.callback)
		srv = DRServer(vgc10_controlConfig, self.dynamic_config_callback)
	

	def create_jt_msg(self):

		self.jt_msg = JointTrajectory()
		self.jt_msg.header.frame_id = ""
		self.jt_msg.joint_names = ["sema_vgc10/extension_joint"]

		self.jtp_msg = JointTrajectoryPoint()
		self.jtp_msg.positions = [self.extension]
		self.jtp_msg.velocities = []
		self.jtp_msg.accelerations = []
		self.jtp_msg.effort = []
		self.jtp_msg.time_from_start = rospy.Duration(1)

		self.jt_msg.points.append(self.jtp_msg)


	def dynamic_config_callback(self, cfg, level):
		self.run(cfg["extension"], cfg["enable"])
		return cfg


	def callback(self, cmd):
		extension, enable = cmd.data.split(',')
		self.run(float(extension), bool(enable))
		
	
	def run(self, extension, enable):
		
		self.extension = extension
		self.jt_msg.points[0].positions = [self.extension]
		self.jt_msg.header.stamp = rospy.Time.now()
		self.pub_vgc_cmd.publish(self.jt_msg)


if __name__ == '__main__':
	time.sleep(1)
	vg_sim_controller = VGSimController()