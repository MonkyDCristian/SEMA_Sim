#!/usr/bin/env python3

import rospy

from sensor_msgs.msg  import JointState


class URJointsRegiter():
	def __init__(self):
		self.variables_init()
		self.connections_init()
		

	def variables_init(self):
		self.ur_joints_name = ["sema/elbow_joint", "sema/shoulder_lift_joint", "sema/shoulder_pan_joint",
		                       "sema/wrist_1_joint", "sema/wrist_2_join", "sema/wrist_3_joint"]

		self.joints_regiter = {}
		self.save_file = "pick_box_poses.py"


	def connections_init(self): 
		pass
	

	def callback(self, joint_msg):
		pass


	def run(self, pose_name):
		joint_state_msg = rospy.wait_for_message("/joint_states", JointState, timeout=3)
		
		dict_joints = {}
		for n, joint_name in enumerate(joint_state_msg.name):
			if joint_name in self.ur_joints_name:
				dict_joints[joint_name] = joint_state_msg.position[n]
		
		self.joints_regiter[pose_name] = dict_joints

		self.save_joint_regiter()
	

	def save_joint_regiter(self):
		with open(self.save_file, 'w') as f:
			f.write(f"{self.save_file[:-3]}={str(self.joints_regiter)}")
		

if __name__ == '__main__':
	rospy.init_node('ur_joint_register')
	ur_joint_register = URJointsRegiter()
	pose_name = "pose0"
	ur_joint_register.run(pose_name)