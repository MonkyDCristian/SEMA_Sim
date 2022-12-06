#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState

# https://roboticsbackend.com/ros-import-python-module-from-another-package/
from sema_moveit.move_group_python_interface import MoveGroupPythonInterface
from pose_compilation.pick_box_poses import pick_box_poses



def tf2joint_state_msg(lib_poses, pose_name):
	goal_joints = JointState()
	
	goal_joints.name = list(lib_poses[ pose_name]["ur_joints"].keys())
	goal_joints.position = list(lib_poses[ pose_name]["ur_joints"].values())

	return goal_joints


def samu_main():
	rospy.init_node("move_group_python_interface")
	mgpi = MoveGroupPythonInterface()
	mgpi.show_variable()
	
	list_joints_name = ['sema/shoulder_pan_joint', 'sema/shoulder_lift_joint', 'sema/elbow_joint', 'sema/wrist_1_joint', 'sema/wrist_2_joint', 'sema/wrist_3_joint']
	list_joints = [[0.27979846,-1.42453212,-1.96969155,-1.32503303,1.56688722,-2.85549416],
					[1.84884104,-1.42470104,-1.97016527,-1.3214439,1.57765714,-1.28647845]]
    
	for joints in list_joints:
		goal_joints = JointState()
		goal_joints.name = list_joints_name
		goal_joints.position = joints
		mgpi.move_group.go(joints=goal_joints , wait=True)
	

if __name__ == "__main__":
		samu_main()