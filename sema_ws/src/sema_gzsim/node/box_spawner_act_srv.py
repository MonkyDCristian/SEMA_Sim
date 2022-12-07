#!/usr/bin/env python3

import rospy

from actionlib import SimpleActionServer
from sema_gzsim.msg import BoxSpawnerAction ,BoxSpawnerFeedback, BoxSpawnerResult

from sema_gzsim.box_spawner import BoxSpawner, dict_boxes

class BoxSpawnerActSrv(object):

	def __init__(self):
		rospy.init_node("box_spawner_act_srv")
		self.variables_init()
		self.connections_init()
		rospy.spin()
		

	def variables_init(self):
		self.posible_boxes = list(dict_boxes.keys())

		self.box_id_register = {}
		for box_model in self.posible_boxes:
			self.box_id_register[box_model] = 0  
		
		# spawn box per *hz* second 
		self.hz = 0.2 # 
		self.rate = rospy.Rate(self.hz)

		self.as_feedback = BoxSpawnerFeedback()
		self.as_feedback.feedback = "box_spawner_start"
		
		self.as_result = BoxSpawnerResult()
		self.as_result.result = "box_spawner_finish"

		self.box_spawner = BoxSpawner()


	def connections_init(self): 
		self.a_srv = SimpleActionServer(rospy.get_name(), BoxSpawnerAction, execute_cb=self.execute_cb, auto_start = False)
		self.a_srv.start()
	

	def execute_cb(self, act_msg):
		
		self.box_spawner.set_pose_target(act_msg.x, act_msg.y, act_msg.z, act_msg.yaw, act_msg.static)

		self.hz = act_msg.hz
		self.rate = rospy.Rate(self.hz)

		self.box_sequence = act_msg.sequence.split(',')

		self.a_srv.publish_feedback(self.as_feedback)

		for box_model in self.box_sequence:
			if box_model not in self.posible_boxes:
				self.as_feedback.feedback = f"{box_model} does not belong to any model of box"
				self.a_srv.publish_feedback(self.as_feedback)
			
			else:
				box_type, box_id = dict_boxes[box_model]["type"], self.box_id_register[box_model]
				self.as_feedback.feedback = f"spawn {box_type}{box_id}"
				self.a_srv.publish_feedback(self.as_feedback)

				self.box_spawner.spawn(box_model, box_id)
				
				self.box_id_register[box_model]+=1
				self.rate.sleep()
			

			if self.a_srv.is_preempt_requested():
				self.a_srv.set_preempted()
		

		self.a_srv.set_succeeded(self.as_result)


if __name__ == "__main__":
	box_spawner = BoxSpawnerActSrv()
		