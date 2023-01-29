#!/usr/bin/env python3

import rospy

from rospkg import RosPack

from gazebo_msgs.srv   import SpawnModel
from geometry_msgs.msg import Quaternion, Pose, Point

from tf.transformations import quaternion_from_euler

class PalletSpawner():

	def __init__(self):
		self.variables_init()
		self.connections_init()
		

	def variables_init(self):
		self.rospack = RosPack()
		self.path = self.rospack.get_path('sema_models')+"/urdf/"
		self.pallet = self.path +"sema_pallet.urdf"
		self.x, self.y, self.z, self.yaw = None, None, None, None
	
		self.scale = {"x":1.0, "y":1.0, "z":1.0}
		self.size = {"x":0.8, "y":1.2, "z":0.144}

	
	def connections_init(self): 
		rospy.wait_for_service("/gazebo/spawn_urdf_model")
		self.spawn_urdf_srv = rospy.ServiceProxy("/gazebo/spawn_urdf_model", SpawnModel)


	def run(self):
		with open(self.pallet,"r") as f:
			pallet_urdf = f.read()
		
		str_scale = "scale='" + str(self.scale["x"]) + " " + str(self.scale["y"]) + " " +  str(self.scale["z"]) +"'"
		str_size = "size='" + str(self.size["x"]) + " " + str(self.size["y"]) + " " +  str(self.size["z"]) +"'"
		
		pallet_urdf = pallet_urdf.replace("scale='1.0 1.0 1.0'", str_scale)
		pallet_urdf = pallet_urdf.replace("size='0.8 1.2 0.144'", str_size)

		quat = quaternion_from_euler(0, 0, self.yaw)
		orient = Quaternion(quat[0], quat[1], quat[2], quat[3])
		pose = Pose(Point(x=self.x, y=self.y, z=self.z), orient)
		print(self.spawn_urdf_srv("sema_pallet", pallet_urdf, '', pose, 'world'))
	

	def set_params(self, params):
		self.x = params["x"]
		self.y = params["y"] 
		self.z = params["z"] 
		self.yaw = params["yaw"]

		if "size" in params:
			if "x" in params["size"]:
				self.scale["x"] =(params["size"]["x"]*self.scale["x"])/self.size["x"]
				self.size["x"] = params["size"]["x"]
			
			if "y" in params["size"]:
				self.scale["y"] =(params["size"]["y"]*self.scale["y"])/self.size["y"]
				self.size["y"] = params["size"]["y"]
			
			if "z" in params["size"]:
				self.scale["z"] =(params["size"]["z"]*self.scale["z"])/self.size["z"]
				self.size["z"] = params["size"]["z"]


if __name__ == "__main__":
	rospy.init_node("pallet_spawner")
	spawn_params = {"x":0.1, "y":0.6, "z":0.5, "yaw":0.0, "size_x":0.6, "size_y":0.9}
	
	pallet_spawner = PalletSpawner()
	pallet_spawner.set_params(spawn_params)
	pallet_spawner.run()

	