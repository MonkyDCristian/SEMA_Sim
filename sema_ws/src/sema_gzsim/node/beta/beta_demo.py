#!/usr/bin/env python3

import rospy, numpy as np

from sema_gzsim.packing_examples import PackingExample1, PackingExample2, PackingExample3, PackingExample4
from sema_gzsim.box_spawner import boxes_prms

from sema_moveit.move_group_python_interface import MoveGroupPythonInterface
from sema_moveit.complite_attacher import CompliteAttacher
from sema_moveit.planner import Planner

from sema_beta_poses import sema_beta_poses

from std_msgs.msg import Empty


class BetaDemo(object):
	
	def __init__(self):
		self.variables_init()
		self.connections_init()
		

	def variables_init(self):
		self.packing = PackingExample4()
		self.packing.palet_prms = {"x":0.1, "y":0.75, "z":0.65, "yaw":0.0, "size":{"x":0.6, "y":0.9, "z":0.14}}
		self.packing.target = {"model":"m", "id":0, "x":0.0, "y":0.0, "z":0.0, "yaw":np.pi, "static":True}
		self.packing.box_prms = boxes_prms[self.packing.target["model"]]
		self.packing.offset = 0.004

		self.mgpi = MoveGroupPythonInterface()
		self.mgpi.show_variable()

		self.complite_attacher = CompliteAttacher(self.mgpi)
		self.complite_attacher.box_attacher.attach_params["ideal_attach"] =  False

		self.planner = Planner(self.mgpi)
		self.planner.joint_poses = sema_beta_poses
		self.planner.setup(replanning=True)
	

	def connections_init(self):
		self.pub_active_vision = rospy.Publisher("box_detector/active", Empty, queue_size=2)
		rospy.Subscriber("box_in_position" , Empty, self.callback)
	

	def callback(self, msg):
		self.pick_box()
		self.move_box_to_target()
		self.dict_box["id"] = False
		self.pub_active_vision.publish()


	def pick_box(self):
		self.planner.go_to_joint_pose("pick")
		self.complite_attacher.attach_box(self.dict_box)
		self.planner.go_to_joint_pose("vision")


	def move_box_to_target(self):
		
		if self.packing.target["id"] == 0:
			self.packing.set_first_box_pose()
		
		else:
			self.packing.get_next_pose()

		self.planner.go_to_target(self.packing.target)
		self.complite_attacher.detach_box(self.dict_box)

		self.packing.target["id"]+= 1
		self.planner.go_to_joint_pose("vision")
		

if __name__ == "__main__":
	rospy.init_node("beta_demo")
	beta_demo = BetaDemo()
	beta_demo.dict_box = {"model":"m", "id": False, "x":0.0, "y":0.0, "z":0.0, "yaw":0.0, "static":True}
	beta_demo.planner.go_to_joint_pose("vision")
	rospy.spin()