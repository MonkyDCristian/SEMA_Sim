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
		self.x = None
		self.y = None 
		self.z = None 
		self.yaw = None

	
	def connections_init(self): 
		rospy.wait_for_service("/gazebo/spawn_urdf_model")
		self.spawn_urdf_srv = rospy.ServiceProxy("/gazebo/spawn_urdf_model", SpawnModel)


	def run(self):
		with open(self.pallet,"r") as f:
			pallet_urdf = f.read()
		
		quat = quaternion_from_euler(0, 0, self.yaw)
		orient = Quaternion(quat[0], quat[1], quat[2], quat[3])
		pose = Pose(Point(x=self.x, y=self.y, z=self.z), orient)
		print(self.spawn_urdf_srv("sema_pallet", pallet_urdf, '', pose, 'world'))
	

	def set_params(self, params):
		self.x = params["x"]
		self.y = params["y"] 
		self.z = params["z"] 
		self.yaw = params["yaw"]


if __name__ == "__main__":
	rospy.init_node("pallet_spawner")
	spawn_params = {"x":0.3, "y":0.8, "z":0.1, "yaw":0.0}
	
	pallet_spawner = PalletSpawner()
	pallet_spawner.set_params(spawn_params)
	pallet_spawner.run()

	