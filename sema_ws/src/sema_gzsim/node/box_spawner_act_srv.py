#!/usr/bin/env python3

import rospy, rospkg

from actionlib import SimpleActionServer
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Quaternion, Pose, Point
from tf.transformations import quaternion_from_euler

from sema_gzsim.msg import BoxSpawnerAction ,BoxSpawnerFeedback, BoxSpawnerResult

class BoxSpawnerActSrv(object):

	def __init__(self):
		rospy.init_node("box_spawner_act_srv")
		self.variables_init()
		self.connections_init()
		rospy.spin()
		

	def variables_init(self):
		self.rospack = rospkg.RosPack()
		self.box_path = self.rospack.get_path('sema_models')+"/urdf/box.urdf"
		
		# pose parameters 
		self.x, self.y, self.z = -2.8, -2.2, 1.0
		self.yaw = 0

		self.posible_boxes = ["l", "ml", "m", "bm", "b"]

		self.dict_box = {"l":{"type": "little", "mass":0.5, "count":0,
							  "size_x":0.12, "size_y":0.15, "size_z":0.12},
					     "ml":{"type": "middle_little", "mass":0.84, "count":0,
						       "size_x":0.16, "size_y":0.19, "size_z":0.12},
						 "m":{"type": "middle", "mass":2.08, "count":0,
						      "size_x":0.2, "size_y":0.3, "size_z":0.15},
				         "bm":{"type": "big_middle", "mass":6.94, "count":0,
						       "size_x":0.3, "size_y":0.4, "size_z":0.25},
				    	 "b":{"type": "big", "mass":13.89, "count":0,
						      "size_x":0.5, "size_y":0.3, "size_z":0.4}}
		
		self.reference_frame = "world"

		# spawn box per *hz* second 
		self.hz = 0.2 # 
		self.rate = rospy.Rate(self.hz)

		self.as_feedback = BoxSpawnerFeedback()
		self.as_feedback.feedback = "box_spawner_start"
		
		self.as_result = BoxSpawnerResult()
		self.as_result.result = "box_spawner_finish"


	def connections_init(self): 
		
		rospy.wait_for_service("/gazebo/spawn_urdf_model")
		self.spawn_urdf_srv = rospy.ServiceProxy("/gazebo/spawn_urdf_model", SpawnModel)
		
		self.a_srv = SimpleActionServer(rospy.get_name(), BoxSpawnerAction, execute_cb=self.execute_cb, auto_start = False)
		self.a_srv.start()
	

	def execute_cb(self, action_msg):
		
		self.x, self.y, self.z = action_msg.x, action_msg.y, action_msg.z
		
		self.hz = action_msg.hz
		self.rate = rospy.Rate(self.hz)

		self.yaw = action_msg.yaw

		self.box_sequence = action_msg.sequence.split(',')

		self.a_srv.publish_feedback(self.as_feedback)

		for box in self.box_sequence:
			if box not in self.posible_boxes:
				self.as_feedback.feedback = f"{box} does not belong any type of box"
				self.a_srv.publish_feedback(self.as_feedback)
			
			else:
				box_type, box_id = self.dict_box[box]["type"], self.dict_box[box]["count"]
				self.as_feedback.feedback = f"spawn {box_type}{box_id}"
				self.a_srv.publish_feedback(self.as_feedback)

				self.spawn(box)
				
				self.rate.sleep()
			
			if self.a_srv.is_preempt_requested():
				self.a_srv.set_preempted()
		

		self.a_srv.set_succeeded(self.as_result)


	def spawn(self, box):
		name = "sema_" + self.dict_box[box]["type"] + "_box" + str(self.dict_box[box]["count"])
		
		box_urdf = self.costum_box(box)

		quat = quaternion_from_euler(0, 0, self.yaw)
		orient = Quaternion(quat[0], quat[1], quat[2], quat[3])
		pose = Pose(Point(x=self.x, y=self.y, z=self.z), orient)
		
		print(self.spawn_urdf_srv(name, box_urdf, '', pose, self.reference_frame))
	

	def costum_box(self, box):

		with open(self.box_path,"r") as f:
			box_urdf = f.read()
		
		box_size = str(self.dict_box[box]["size_x"]) + " " + str(self.dict_box[box]["size_y"]) + " " + str(self.dict_box[box]["size_z"])
		ixx, iyy, izz = self.get_inertial(box)
	
		box_urdf  = box_urdf.replace("type", self.dict_box[box]["type"])
		box_urdf  = box_urdf.replace("N", str(self.dict_box[box]["count"]))
		box_urdf  = box_urdf.replace("box_size", box_size)
		box_urdf  = box_urdf.replace("mass_", str(self.dict_box[box]["mass"]))
		box_urdf  = box_urdf.replace("ixx_", ixx)
		box_urdf  = box_urdf.replace("iyy_", iyy)
		box_urdf  = box_urdf.replace("izz_", izz)

		print(box_urdf)

		self.dict_box[box]["count"]+=1

		return box_urdf

	
	def get_inertial(self, box):
		ixx = (self.dict_box[box]["mass"]/12)*(self.dict_box[box]["size_y"]**2+self.dict_box[box]["size_z"]**2)
		iyy = (self.dict_box[box]["mass"]/12)*(self.dict_box[box]["size_x"]**2+self.dict_box[box]["size_z"]**2)
		izz = (self.dict_box[box]["mass"]/12)*(self.dict_box[box]["size_x"]**2+self.dict_box[box]["size_y"]**2)
		return str(ixx), str(iyy), str(izz)
	

if __name__ == "__main__":
	box_spawner = BoxSpawnerActSrv()
		