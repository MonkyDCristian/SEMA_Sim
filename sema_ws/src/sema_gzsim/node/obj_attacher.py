#!/usr/bin/env python3

import rospy

from geometry_msgs.msg import PoseStamped
from std_msgs.msg      import String

from gazebo_msgs.srv import GetLinkState, GetLinkStateRequest
from gazebo_msgs.srv import GetLinkProperties, GetLinkPropertiesRequest
from gazebo_msgs.srv import SetLinkState, SetLinkStateRequest
from gazebo_msgs.srv import SetLinkProperties, SetLinkPropertiesRequest

# http://docs.ros.org/en/jade/api/tf/html/python/transformations.html
from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion
from tf import TransformListener


class ObjAttacher():
	def __init__(self):
		self.variables_init()
		self.connections_init()
		

	def variables_init(self):
		self.get_link_properties_req = GetLinkPropertiesRequest()
		self.get_link_state_req = GetLinkStateRequest()
		
		self.set_link_state_req = SetLinkStateRequest()
		self.set_link_properties_req = SetLinkPropertiesRequest()
		
		self.hz = 18
		self.rate = rospy.Rate(self.hz) #Hz
		
		self.obj_link = ""
		self.ref_link = ""
		self.obj_attached = False
		self.ideal_attach = False
		self.pose_tf = PoseStamped()
		self.pose_tf.header.frame_id = ""


	def connections_init(self): 
		
		self.get_link_properties_srv = rospy.ServiceProxy('/gazebo/get_link_properties', GetLinkProperties)
		self.get_link_state_srv = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
		self.set_link_properties_srv = rospy.ServiceProxy('/gazebo/set_link_properties', SetLinkProperties)
		self.set_link_state_srv = rospy.ServiceProxy('/gazebo/set_link_state', SetLinkState)
		
		self.tf_listener = TransformListener()
		
		rospy.Subscriber("attach/links/obj/ref", String, self.callback)
	

	def callback(self, cmd):
		pass


	def run(self, obj_link, reference_link="sema/vgc10/extension_link", pose_tf_frame_id="sema/wrist_3_link", ideal_attach=False):
		self.obj_link = obj_link
		self.ref_link = reference_link
		self.pose_tf.header.frame_id = pose_tf_frame_id
		self.ideal_attach = ideal_attach

		self.create_attach()


	def create_attach(self):

		res = self.costum_obj_properties(gravity=False)

		if not res.success:
			print(res)

		if not self.ideal_attach:
			self.pose_tf.pose = self.get_link_pose(self.obj_link, self.ref_link)
		
		self.obj_attached= True
		
		while not rospy.is_shutdown() and self.obj_attached:
			self.keep_attach()
			self.rate.sleep()
		
		res = self.costum_obj_properties(gravity=True)

		if not res.success:
			print(res)


	def costum_obj_properties(self, gravity=False):
		self.get_link_properties_req.link_name = self.obj_link
		link_properties_msg = self.get_link_properties_srv(self.get_link_properties_req)

		self.set_link_properties_req.link_name = self.obj_link
		self.set_link_properties_req.gravity_mode = gravity
		self.set_link_properties_req.mass = link_properties_msg.mass
		self.set_link_properties_req.ixx = link_properties_msg.ixx
		self.set_link_properties_req.iyy = link_properties_msg.iyy
		self.set_link_properties_req.izz = link_properties_msg.izz

		return self.set_link_properties_srv(self.set_link_properties_req)
	

	def keep_attach(self):
		
		if self.ideal_attach:
			obj_pose = self.get_link_pose(self.ref_link, "world")
			ref_quat = [obj_pose.orientation.x, obj_pose.orientation.y, obj_pose.orientation.z, obj_pose.orientation.w]
			
			row, pitch, yaw = euler_from_quaternion(ref_quat)
			fix_quat = quaternion_from_euler(-row, pitch, yaw)
			
			obj_pose.orientation.x = fix_quat[0]
			obj_pose.orientation.y = fix_quat[1]
			obj_pose.orientation.z = fix_quat[2]
			obj_pose.orientation.w = fix_quat[3]
			
		else:
			self.tf_listener.waitForTransform("world", self.pose_tf.header.frame_id, rospy.Time(), rospy.Duration(10.0))
			obj_pose = self.tf_listener.transformPose("world", self.pose_tf).pose
	
		res = self.call_set_state_srv(obj_pose)
	

	def get_link_pose(self, link_obj, link_ref):
		res = self.call_get_state_srv(link_obj, link_ref)
		
		if res.success:
			return res.link_state.pose
		
		else:
			print(res)


	def call_get_state_srv(self, link_obj, link_ref):
		self.get_link_state_req.link_name = link_obj
		self.get_link_state_req.reference_frame = link_ref
		return self.get_link_state_srv(self.get_link_state_req) 


	def call_set_state_srv(self, relative_pose): # try set model 
		self.set_link_state_req.link_state.link_name = self.obj_link
		self.set_link_state_req.link_state.pose = relative_pose
		self.set_link_state_req.link_state.reference_frame = "world"

		return self.set_link_state_srv(self.set_link_state_req)

	
if __name__ == '__main__':
	rospy.init_node('obj_attacher')

	obj_link = "sema_middle_little_box_link1"
	ideal_attach = False
	
	obj_attacher = ObjAttacher()
	obj_attacher.run(obj_link, ideal_attach=ideal_attach)
