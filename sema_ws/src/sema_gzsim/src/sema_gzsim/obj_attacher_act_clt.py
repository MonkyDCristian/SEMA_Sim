#!/usr/bin/env python3

import rospy

# http://wiki.ros.org/actionlib
from actionlib import SimpleActionClient
from sema_gzsim.msg import ObjAttacherAction ,ObjAttacherGoal

class ObjAttacherActClt(object):
	def __init__(self):
		self.variables_init()
		self.connections_init()
		

	def variables_init(self):
		self.as_goal = ObjAttacherGoal()
		
		self.as_goal.obj_link = None
		self.as_goal.ideal_attach = None


	def connections_init(self): 
		self.client = SimpleActionClient("obj_attacher_act_srv", ObjAttacherAction)
		self.client.wait_for_server()
	

	def run(self):
		self.client.send_goal(self.as_goal)
	

	def stop(self):
		self.client.cancel_goal()


	def set_params(self, params):
		self.as_goal.obj_link = params["obj_link"]
		self.as_goal.ideal_attach = params["ideal_attach"]


if __name__ == "__main__":
	rospy.init_node("obj_attacher_act_clt")
	obj_attacher = ObjAttacherActClt()

	attach_params = {"obj_link":"sema_middle_little_box_link1", "ideal_attach": True}
	
	obj_attacher.set_params(attach_params)
	obj_attacher.run()

	rospy.sleep(5)

	obj_attacher.stop()

	rospy.sleep(5)

	obj_attacher.run()
