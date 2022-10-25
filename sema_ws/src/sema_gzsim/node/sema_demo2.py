#!/usr/bin/env python3

import rospy

from sensor_msgs.msg import JointState

# https://roboticsbackend.com/ros-import-python-module-from-another-package/
from sema_moveit.move_group_python_interface import MoveGroupPythonInterface
from pose_compilation.example import example

class SemaDemo2(object):
	
	def __init__(self):
		rospy.init_node("sema_demo2")
		self.variables_init()
		self.connections_init()
		

	def variables_init(self):
		self.mgpi = MoveGroupPythonInterface()


	def connections_init(self): 
		pass
	

	def run(self):
		pass


def tf2joint_state_msg(lib_poses, pose_name):
	goal_joints = JointState()
	
	goal_joints.name = list(lib_poses[ pose_name]["ur_joints"].keys())
	goal_joints.position = list(lib_poses[ pose_name]["ur_joints"].values())

	return goal_joints
		

if __name__ == "__main__":
	sema_demo2 = SemaDemo2()
	#sema_demo2.mgpi.show_variable()
	
	goal_joints_0 = tf2joint_state_msg(example, "pose0")
	goal_joints_1 = tf2joint_state_msg(example, "pose1")
	
	enter_msg = input("READY TO PLANNING: Press enter to planning a trayectory")
	 
	plan = sema_demo2.mgpi.move_group.plan(joints=goal_joints_0)
	print(plan)
	# or 
	#sema_demo2.mgpi.move_group.set_joint_values_target(goal_joints_0)
	#plan = sema_demo2.mgpi.move_group.plan()

	trajectory = plan[1]
	sema_demo2.mgpi.display_trajectory(trajectory)


	enter_msg = input("READY TO MOVE: Press enter to move the UR10")

	sema_demo2.mgpi.move_group.go(wait=True)

	enter_msg = input("READY TO MOVE: Press enter to plan and move with one comand")

	sema_demo2.mgpi.move_group.go(joints=goal_joints_1 , wait=True)