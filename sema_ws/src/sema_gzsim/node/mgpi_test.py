#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState

# https://roboticsbackend.com/ros-import-python-module-from-another-package/
from sema_moveit.move_group_python_interface import MoveGroupPythonInterface
from sema_gzsim.ur_pose_register import tf2joint_state_msg
from pose_compilation.pick_box_poses import pick_box_poses


def sim_main():
	rospy.init_node("move_group_python_interface")
	mgpi = MoveGroupPythonInterface()
	mgpi.show_variable()
	
	goal_joints_0 = tf2joint_state_msg(pick_box_poses, "pick_pose")
	goal_joints_1 = tf2joint_state_msg(pick_box_poses, "final_place_pose")
	
	enter_msg = input("READY TO PLANNING: Press enter to planning a trayectory")
	 
	plan = mgpi.move_group.plan(joints=goal_joints_0)
	# or use:
	#sema_demo2.mgpi.move_group.set_joint_values_target(goal_joints_0)
	#plan = sema_demo2.mgpi.move_group.plan()

	trajectory = plan[1]
	mgpi.display_trajectory(trajectory)

	enter_msg = input("READY TO MOVE: Press enter to move the UR10")

	mgpi.move_group.go(wait=True)

	enter_msg = input("READY TO MOVE: Press enter to plan and move with one comand")

	mgpi.move_group.go(joints=goal_joints_1 , wait=True)


def main():
	rospy.init_node("move_group_python_interface")
	mgpi = MoveGroupPythonInterface()
	mgpi.show_variable()
	
	goal_joints_0 = tf2joint_state_msg(pick_box_poses, "real_pick_pose")
	goal_joints_1 = tf2joint_state_msg(pick_box_poses, "real_final_pose")
	
	enter_msg = input("READY TO PLANNING: Press enter to planning a trayectory")
	 
	plan = mgpi.move_group.plan(joints=goal_joints_0)
	# or use:
	#sema_demo2.mgpi.move_group.set_joint_values_target(goal_joints_0)
	#plan = sema_demo2.mgpi.move_group.plan()

	trajectory = plan[1]
	mgpi.display_trajectory(trajectory)

	enter_msg = input("READY TO MOVE: Press enter to move the UR10")

	mgpi.move_group.go(wait=True)

	enter_msg = input("READY TO MOVE: Press enter to plan and move with one comand")

	mgpi.move_group.go(joints=goal_joints_1 , wait=True)


if __name__ == "__main__":
	main()