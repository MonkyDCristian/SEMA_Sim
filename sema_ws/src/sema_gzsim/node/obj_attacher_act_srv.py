#!/usr/bin/env python3

import rospy

from geometry_msgs.msg import PoseStamped

# https://classic.gazebosim.org/tutorials?tut=ros_comm&cat=connect_ros
from gazebo_msgs.srv import GetLinkState, GetLinkStateRequest
from gazebo_msgs.srv import GetLinkProperties, GetLinkPropertiesRequest
from gazebo_msgs.srv import SetLinkState, SetLinkStateRequest
from gazebo_msgs.srv import SetLinkProperties, SetLinkPropertiesRequest

# http://wiki.ros.org/actionlib
from actionlib      import SimpleActionServer
from sema_gzsim.msg import ObjAttacherAction, ObjAttacherFeedback , ObjAttacherResult

# https://wiki.ros.org/tf/Tutorials/tf%20and%20Time%20%28Python%29
# http://docs.ros.org/en/jade/api/tf/html/python/transformations.html
from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion
from tf import TransformListener

class ObjAttacherActSrv():
	def __init__(self):
		rospy.init_node('obj_attacher_act_srv')
		self.variables_init()
		self.connections_init()
		rospy.spin()
		

	def variables_init(self):
		self.get_link_properties_req = GetLinkPropertiesRequest()
		self.get_link_state_req = GetLinkStateRequest()
		
		self.set_link_state_req = SetLinkStateRequest()
		self.set_link_properties_req = SetLinkPropertiesRequest()

		self.as_feedback = ObjAttacherFeedback() 
		self.as_result = ObjAttacherResult()
		
		self.hz = 18
		self.rate = rospy.Rate(self.hz) #Hz
		
		self.obj_link = ""
		self.ref_link = "sema/vgc10/extension_link"
		self.ideal_attach = False
		self.pose_tf = PoseStamped()
		self.pose_tf.header.frame_id = "sema/wrist_3_link"
		self.stop = False


	def connections_init(self): 	
		self.get_link_properties_srv = rospy.ServiceProxy('/gazebo/get_link_properties', GetLinkProperties)
		self.get_link_state_srv = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
		self.set_link_properties_srv = rospy.ServiceProxy('/gazebo/set_link_properties', SetLinkProperties)
		self.set_link_state_srv = rospy.ServiceProxy('/gazebo/set_link_state', SetLinkState)
		
		self.a_srv = SimpleActionServer(rospy.get_name(), ObjAttacherAction, execute_cb=self.execute_cb, auto_start = False)
		self.a_srv.start()

		self.tf_listener = TransformListener()
	

	def execute_cb(self, msg):
		self.run(msg.obj_link, ideal_attach = msg.ideal_attach)


	def run(self, obj_link, ideal_attach=False):
		self.obj_link = obj_link
		self.ideal_attach = ideal_attach

		self.create_attach()


	def create_attach(self): # +++
		self.costum_obj_properties(gravity=False)

		if not self.ideal_attach:
			self.pose_tf.pose = self.get_link_pose(self.obj_link, self.ref_link)
		
		self.stop = False
		while not rospy.is_shutdown() and not self.stop:
			self.keep_attach()
			self.rate.sleep()

			if self.a_srv.is_preempt_requested():
				self.a_srv.set_preempted()
				self.stop = True


		
		self.costum_obj_properties(gravity=True)


	def costum_obj_properties(self, gravity=False):
		self.get_link_properties_req.link_name = self.obj_link
		link_properties_msg = self.get_link_properties_srv(self.get_link_properties_req)

		self.set_link_properties_req.link_name = self.obj_link
		self.set_link_properties_req.gravity_mode = gravity
		self.set_link_properties_req.mass = link_properties_msg.mass
		self.set_link_properties_req.ixx = link_properties_msg.ixx
		self.set_link_properties_req.iyy = link_properties_msg.iyy
		self.set_link_properties_req.izz = link_properties_msg.izz
		
		res = self.set_link_properties_srv(self.set_link_properties_req)
		
		return self.check_response(res)
		

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
		return self.check_response(res).link_state.pose


	def call_get_state_srv(self, link_obj, link_ref):
		self.get_link_state_req.link_name = link_obj
		self.get_link_state_req.reference_frame = link_ref
		
		res = self.get_link_state_srv(self.get_link_state_req) 
		
		return self.check_response(res)


	def call_set_state_srv(self, relative_pose): # try set model 
		self.set_link_state_req.link_state.link_name = self.obj_link
		self.set_link_state_req.link_state.pose = relative_pose
		self.set_link_state_req.link_state.reference_frame = "world"

		res = self.set_link_state_srv(self.set_link_state_req)

		return self.check_response(res)
	

	def check_response(self, response):
		if response.success:
			return response
		
		else:
			self.as_feedback.feedback = response.status_message
			self.a_srv.publish_feedback(self.as_feedback)

	
if __name__ == '__main__':
	obj_attacher_act_srv = ObjAttacherActSrv()
