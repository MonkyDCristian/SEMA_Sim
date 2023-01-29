#!/usr/bin/env python3

import rospy 

from rospkg  import RosPack

from gazebo_msgs.srv import GetLinkState, GetLinkStateRequest
from sensor_msgs.msg  import JointState

from sema_gzsim.cfg import save_pose_registerConfig 
from dynamic_reconfigure.server import Server as DRServer


def tf2joint_state_msg(lib_poses, pose_name):
	goal_joints = JointState()
	
	goal_joints.name = list(lib_poses[pose_name]["ur_joints"].keys())
	goal_joints.position = list(lib_poses[pose_name]["ur_joints"].values())

	return goal_joints


class URJointsRegiter():
	def __init__(self):
		self.variables_init()
		self.connections_init()
		

	def variables_init(self):
		self.rospack = RosPack()
		self.path = self.rospack.get_path('sema_gzsim')+"/node/pose_compilation/"
		self.file_name = ""

		self.in_simulation = True

		self.ur_joints_name = ["sema/elbow_joint", "sema/shoulder_lift_joint", "sema/shoulder_pan_joint",
		                       "sema/wrist_1_joint", "sema/wrist_2_joint", "sema/wrist_3_joint"]
		
		self.real_ur_joints_name = ["elbow_joint", "shoulder_lift_joint", "shoulder_pan_joint",
		                            "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
		
		#self.ur_eef_name = "sema/wrist_3_link"
		
		self.ur_pose_data = {}


	def connections_init(self): 
		self.get_link_state_srv = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
		srv = DRServer(save_pose_registerConfig, self.dynamic_config_callback)


	def dynamic_config_callback(self, cfg, level):
		if cfg["enable"]:
			self.in_simulation = cfg["in_simulation"]
			self.run(cfg["file_name"], cfg["pose_name"])

		return cfg


	def run(self, file_name, pose_name):
		
		if file_name != self.file_name:
			self.file_name = file_name
			self.ur_pose_data = {}
			
		ur_joints = self.get_ur_joints()

		self.ur_pose_data[pose_name] = {"ur_joints":ur_joints} 
		
		self.save_ur_pose_data()

	
	def get_ur_joints(self):
		joint_state_msg = rospy.wait_for_message("/joint_states", JointState, timeout=3)

		if self.in_simulation:
			ur_joints_name = self.ur_joints_name
		
		else:
			ur_joints_name = self.real_ur_joints_name
		
		ur_joints = {}
	
		for n, joint_name in enumerate(joint_state_msg.name):
			if joint_name in ur_joints_name:
				ur_joints[joint_name] = joint_state_msg.position[n]
		
		return ur_joints

	"""
	def get_eef_pose(self):
		link_state_msg = self.get_link_state_srv(GetLinkStateRequest(self.ur_eef_name, "world"))
		pose_msg = link_state_msg.link_state.pose
		
		eef_pose = {"position":{"x":pose_msg.position.x, "y":pose_msg.position.y, "z":pose_msg.position.z}, 
		            "orientation": {"x":pose_msg.orientation.x,"y":pose_msg.orientation.y,
					                "z":pose_msg.orientation.z,"w":pose_msg.orientation.w}}

		return eef_pose
	"""

	def save_ur_pose_data(self):
		with open(self.path + self.file_name, 'w') as f:
			f.write(f"{self.file_name[:-3]}={str(self.ur_pose_data)}")


def main():
	rospy.init_node('ur_joint_register')
	ur_joint_register = URJointsRegiter()
	rospy.spin()


def manual_register():
	rospy.init_node('ur_joint_register')
	ur_joint_register = URJointsRegiter()
	ur_joint_register.path = ur_joint_register.rospack.get_path('sema_gzsim')+"/node/pose_compilation/"
	ur_joint_register.in_simulation = True
	file_name, pose_name = "example.py", "pose0"
	ur_joint_register.run(file_name, pose_name)


if __name__ == '__main__':
	main()
