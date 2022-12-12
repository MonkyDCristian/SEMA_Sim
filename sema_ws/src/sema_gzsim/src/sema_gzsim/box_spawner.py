import rospy, rospkg

from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Quaternion, Pose, Point
from tf.transformations import quaternion_from_euler


boxes_prms = {"l":{"type": "little", "mass":0.5, "size":{"x":0.12, "y":0.15, "z":0.12}},
			  "ml":{"type": "middle_little", "mass":0.84, "size":{"x":0.16, "y":0.19, "z":0.12}},
			  "m":{"type": "middle", "mass":2.08, "size":{"x":0.2, "y":0.3, "z":0.15}},
			  "bm":{"type": "big_middle", "mass":6.94, "size":{"x":0.3, "y":0.4, "z":0.25}},
			  "b":{"type": "big", "mass":13.89, "size":{"x":0.5, "y":0.3, "z":0.4}}}


def get_box_model_name(box_model, box_id):
	return "sema_" + boxes_prms[box_model]["type"] + "_box" + str(box_id)

def get_box_link_name(box_model, box_id):
	return "sema_" + boxes_prms[box_model]["type"] + "_box_link" + str(box_id)


class BoxSpawner(object):
	
	def __init__(self):
		self.variables_init()
		self.connections_init()


	def variables_init(self):
		self.rospack = rospkg.RosPack()
		self.box_path = self.rospack.get_path('sema_models')+"/urdf/box.urdf"

		# pose parameters 
		self.x, self.y, self.z = 0, 0, 0
		self.yaw = 0

		self.reference_frame = "world"
		self.static = False
	

	def connections_init(self): 
		rospy.wait_for_service("/gazebo/spawn_urdf_model")
		self.spawn_urdf_srv = rospy.ServiceProxy("/gazebo/spawn_urdf_model", SpawnModel)


	def spawn(self, box_model, box_id):
		name = get_box_model_name(box_model, box_id)
		
		dict_box = boxes_prms[box_model]
		box_urdf = self.costum_box(dict_box, box_id)

		quat = quaternion_from_euler(0, 0, self.yaw)
		orient = Quaternion(quat[0], quat[1], quat[2], quat[3])
		pose = Pose(Point(x=self.x, y=self.y, z=self.z), orient)
		
		print(self.spawn_urdf_srv(name, box_urdf, '', pose, self.reference_frame))
	

	def costum_box(self, dict_box, box_id):

		with open(self.box_path,"r") as f:
			box_urdf = f.read()
		
		box_size = str(dict_box["size"]["x"]) + " " + str(dict_box["size"]["y"]) + " " + str(dict_box["size"]["z"])
		ixx, iyy, izz = self.get_inertial(dict_box)
	
		box_urdf  = box_urdf.replace("type", dict_box["type"])
		box_urdf  = box_urdf.replace("N", str(box_id))
		box_urdf  = box_urdf.replace("box_size", box_size)
		box_urdf  = box_urdf.replace("mass_", str(dict_box["mass"]))
		box_urdf  = box_urdf.replace("ixx_", ixx)
		box_urdf  = box_urdf.replace("iyy_", iyy)
		box_urdf  = box_urdf.replace("izz_", izz)

		if self.static:
			box_urdf  = box_urdf.replace("<static>false</static>", "<static>true</static>")

		return box_urdf
	

	def set_pose_target(self, x , y, z, yaw, static=False):
		self.x, self.y, self.z = x, y, z
		self.yaw = yaw
		self.static = static

	
	def get_inertial(self, dict_box):
		ixx = (dict_box["mass"]/12)*(dict_box["size"]["y"]**2 + dict_box["size"]["z"]**2)
		iyy = (dict_box["mass"]/12)*(dict_box["size"]["x"]**2 + dict_box["size"]["z"]**2)
		izz = (dict_box["mass"]/12)*(dict_box["size"]["x"]**2 + dict_box["size"]["y"]**2)
		return str(ixx), str(iyy), str(izz)

