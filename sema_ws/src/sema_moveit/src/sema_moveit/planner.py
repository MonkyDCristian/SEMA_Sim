#!/usr/bin/env python3

import rospy, numpy as np

from sema_moveit.move_group_python_interface import MoveGroupPythonInterface
from sema_gzsim.ur_pose_register import tf2joint_state_msg

from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler


class Planner(object):
	
	def __init__(self, mgpi=None):
		self.mgpi = mgpi
		self.gripper_offset = 0.205
		self.joint_poses = {}
	

	def setup(self, vel_factor=1.0, acc_factor=1.0, replanning=False):
		self.mgpi.move_group.set_max_velocity_scaling_factor(vel_factor)
		self.mgpi.move_group.set_max_acceleration_scaling_factor(acc_factor)
		self.mgpi.move_group.allow_replanning(replanning)
	

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
		return Pose(Point(dict_target["x"], dict_target["y"], dict_target["z"] + self.gripper_offset), Quaternion(*quat))
	

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

	dict_target = {"x":-0.5, "y":0.2, "z":1.0, "yaw":0.0} # position of the end effector

	planner.plan(dict_target) # plan and wait for enter signal to go to the target
	#planner.go_to_target(dict_target) # plan and go to the target











