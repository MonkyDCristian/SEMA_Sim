#!/usr/bin/env python3

import rospy

#https://roboticsbackend.com/ros-import-python-module-from-another-package/
from sema_moveit.move_group_python_interface import MoveGroupPythonInterface


class SemaDemo(object):
	
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
		

if __name__ == "__main__":
	sema_demo = SemaDemo()
