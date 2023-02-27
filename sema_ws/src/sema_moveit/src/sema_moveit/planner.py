#!/usr/bin/env python3

"""
This class allows you to get data from Moveit! Planner and configure it. 
It is possible to load a dictionary of already registered poses for direct execution.
It also has a simple parameter configuration syntax for planning and executing paths using dictionaries. 
Reference: http://docs.ros.org/en/noetic/api/moveit_commander/html/classmoveit__commander_1_1move__group_1_1MoveGroupCommander.html
"""

import rospy, numpy as np

from sema_moveit.move_group_python_interface import MoveGroupPythonInterface
from sema_gzsim.ur_pose_register import tf2joint_state_msg

from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler


class Planner(object):
	
	def __init__(self, mgpi=None):
		self.mgpi = mgpi
		self.gripper_offset = 0.115 # default gripper offset
		self.extension_offset = {"l": 0.075,"ml": 0.075, "m":0.09,"bm":0.14,"b":0.215} # default extension offset by box model
		self.joint_poses = {} # dictionary of registered joint poses


	def setup(self, vel_factor=1.0, acc_factor=1.0, replanning=False): 
		self.mgpi.move_group.set_max_velocity_scaling_factor(vel_factor) # vel_factor between 0.0 and 1.0
		self.mgpi.move_group.set_max_acceleration_scaling_factor(acc_factor) # acc_factor between 0.0 and 1.0
		self.mgpi.move_group.allow_replanning(replanning) # keep replanning until a solution is found
	

	def get_planner_info(self):
		print("Planner ID: ", self.mgpi.move_group.get_planner_id())
		print("Planning Frame: ", self.mgpi.move_group.get_planning_frame())
		print("Planning Pipeline ID: ", self.mgpi.move_group.get_planning_pipeline_id())
		print("Planning Time: ", self.mgpi.move_group.get_planning_time())
	

	def go_to_joint_pose(self, pose_name):
		joints = tf2joint_state_msg(self.joint_poses, pose_name)
		self.mgpi.move_group.go(joints=joints, wait=True)


	def go_to_target(self, dict_target):
		pose = self.create_pose(dict_target)
		self.mgpi.go_to_target(pose)
	

	def plan(self, dict_target):
		pose = self.create_pose(dict_target)
		success, trajectory, planning_time, error_code = self.mgpi.move_group.plan(pose)
		
		print(f"success: {success}, planning_time: {planning_time}, error_code: {error_code}")
		
		self.mgpi.display_trajectory(trajectory)
		
		enter_msg = input("READY TO MOVE: Press enter execute")
		
		self.mgpi.move_group.go(wait=True)

	
	def create_pose(self, dict_target):
		quat = quaternion_from_euler(np.pi, 0, dict_target["yaw"])
		height = dict_target["z"] + self.gripper_offset  + self.extension_offset[dict_target["model"]]
		return Pose(Point(dict_target["x"], dict_target["y"], height), Quaternion(*quat))
	

if __name__ == "__main__":

	from sema_gzsim.pose_compilation.pick_box_poses import pick_box_poses
	
	rospy.init_node("planner")
	
	mgpi = MoveGroupPythonInterface()	
	mgpi.show_variable()

	planner = Planner(mgpi)	

	planner.setup()
	planner.get_planner_info()

	planner.joint_poses = pick_box_poses
	planner.go_to_joint_pose("pick_pose")

	dict_target = {"model":"m", "x":-0.5, "y":0.2, "z":1.0, "yaw":0.0} # position of the end effector

	planner.plan(dict_target) # plan and wait for enter signal to go to the target
	#planner.go_to_target(dict_target) # plan and go to the target











