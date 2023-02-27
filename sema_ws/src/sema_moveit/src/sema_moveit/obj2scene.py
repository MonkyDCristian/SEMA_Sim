#!/usr/bin/env python3

import rospy

# https://roboticsbackend.com/ros-import-python-module-from-another-package/
from sema_moveit.move_group_python_interface import MoveGroupPythonInterface
from sema_gzsim.box_spawner import boxes_prms, get_box_model_name

from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler


class Obj2Scene(object):
	
	def __init__(self, mgpi=None):
		self.mgpi = mgpi
		self.grasping_group = "endeffector"
		self.frame_id = "world"


	def add_cube(self, dict_cube):
		cube_obj = {}

		cube_obj["name"] = dict_cube["name"]

		cube_obj["pose"] = PoseStamped()
		cube_obj["pose"].header.frame_id = self.frame_id
		
		quat = quaternion_from_euler(0,0,dict_cube["yaw"])
		cube_obj["pose"].pose = Pose(Point(dict_cube["x"], dict_cube["y"], dict_cube["z"]), Quaternion(*quat))
		
		size = dict_cube["size"]
		cube_obj["size"] = (size["x"], size["y"], size["z"])

		self.mgpi.add_object_to_scene(cube_obj, obj_type="box")
	
	def add_cylinder(self, dict_cylinder):
		cylinder_obj = {}
		cylinder_obj["name"] = dict_cylinder["name"]
		
		cylinder_obj["pose"] = PoseStamped()
		cylinder_obj["pose"].header.frame_id = self.frame_id
		quat = quaternion_from_euler(0, 0, 0)
		cylinder_obj["pose"].pose = Pose(Point(dict_cylinder["x"], dict_cylinder["y"], dict_cylinder["z"]), Quaternion(*quat))
		
		cylinder_obj["height"] = dict_cylinder["height"]
		cylinder_obj["radius"] = dict_cylinder["radius"]

		self.mgpi.add_object_to_scene(cylinder_obj, obj_type="cylinder")
	

	def add_plane(self, dict_plane):
		plane_obj = {}
		plane_obj["name"] = dict_plane["name"]
		
		plane_obj["pose"] = PoseStamped()
		plane_obj["pose"].header.frame_id = self.frame_id
		quat = quaternion_from_euler(0, 0, 0)
		plane_obj["pose"].pose = Pose(Point(0, 0, dict_plane["z"]), Quaternion(*quat))

		self.mgpi.add_object_to_scene(plane_obj, obj_type="plane")


	def add_box(self, dict_box):
		box_obj = {}
		box_obj["name"] = get_box_model_name(dict_box["model"], dict_box["id"])

		box_obj["pose"] = PoseStamped()
		box_obj["pose"].header.frame_id = self.frame_id
		quat = quaternion_from_euler(0,0, dict_box["yaw"])
		box_obj["pose"].pose = Pose(Point(dict_box["x"], dict_box["y"], dict_box["z"]), Quaternion(*quat))
		
		size = boxes_prms[dict_box["model"]]["size"]
		box_obj["size"] = (size["x"], size["y"], size["z"])
		
		self.mgpi.add_object_to_scene(box_obj, obj_type="box")
	

	def attach_box(self, dict_box):
		obj_name = get_box_model_name(dict_box["model"], dict_box["id"])
		self.mgpi.attach_obj(obj_name, self.grasping_group)
	
	
	def detach_box(self, dict_box):
		box_name = get_box_model_name(dict_box["model"], dict_box["id"])
		self.mgpi.detach_obj(box_name)
		self.mgpi.remove_obj(box_name)
		self.add_box(dict_box)


if __name__ == "__main__":
	rospy.init_node("obj2scene")

	mgpi = MoveGroupPythonInterface()
	mgpi.show_variable()
	
	obj2scene = Obj2Scene(mgpi)
	
	dict_box = {"model":"m", "id":0, "x":0.0, "y":-0.3, "z":0.8, "yaw":0.0}
	dict_palet = {"name":"palet1" ,"x":0.1, "y":0.6, "z":0.5, "yaw":0.0, "size":{"x":0.6, "y":0.9, "z":0.14}}
	
	obj2scene.add_cube(dict_palet)
	rospy.sleep(4)
	
	obj2scene.add_box(dict_box)
	rospy.sleep(4)
	
	obj2scene.attach_box(dict_box)
	rospy.sleep(4)
	
	obj2scene.detach_box(dict_box)

