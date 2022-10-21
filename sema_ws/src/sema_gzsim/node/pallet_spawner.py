#!/usr/bin/env python3

import rospy, tf, rospkg
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Quaternion, Pose, Point

class PalletSpawner():

	def __init__(self) -> None:
		rospy.init_node("pallet_spawner")
		self.variables_init()
		self.connections_init()
		

	def variables_init(self):
		self.rospack = rospkg.RosPack()
		self.path = self.rospack.get_path('sema_models')+"/urdf/"
		self.pallet = self.path+"sema_pallet.urdf"
		self.x, self.y, self.z = 0.3, 0.8, 0.1
		self.yaw = 0

	
	def connections_init(self): 
		rospy.wait_for_service("/gazebo/spawn_urdf_model")
		self.spawn_urdf_srv = rospy.ServiceProxy("/gazebo/spawn_urdf_model", SpawnModel)


	def spawn(self):
		with open(self.pallet,"r") as f:
			pallet_urdf = f.read()
		
		quat = tf.transformations.quaternion_from_euler(0, 0, self.yaw)
		orient = Quaternion(quat[0], quat[1], quat[2], quat[3])
		pose = Pose(Point(x=self.x, y=self.y, z=self.z), orient)
		print(self.spawn_urdf_srv("palet", pallet_urdf, '', pose, 'world'))
	

if __name__ == "__main__":
	pallet_spawner = PalletSpawner()
	pallet_spawner.spawn()
		