#!/usr/bin/env python3

import rospy

from gazebo_msgs.msg import ModelStates


class BoxPosePub():

    def __init__(self):
        rospy.init_node('box_pose_pub')
        self.variables_init()
        self.connections_init()
        rospy.spin()

    
    def variables_init(self):
        pass
    

    def connections_init(self):
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.publish_box_poses)
        self.pub_box_poses = rospy.Publisher("/box_poses", ModelStates, queue_size=5)


    def publish_box_poses(self, models_data):
        box_states = ModelStates()
    
        for _id ,model_name in enumerate(models_data.name):

            if "box" in model_name:
                box_states.name.append(model_name)       
                box_states.pose.append(models_data.pose[_id])

        self.pub_box_poses.publish(box_states)


if __name__ == '__main__':
    pub_markers = BoxPosePub()