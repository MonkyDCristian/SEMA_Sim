#!/usr/bin/env python3

import rospy
from std_msgs.msg import Header
from gazebo_msgs.msg import ModelStates
from visualization_msgs.msg import Marker, MarkerArray


class PubMarkers():

    def __init__(self):
        rospy.init_node('publish_markers')
        self.variables_init()
        self.connections_init()
        rospy.spin()

    
    def variables_init(self):
        self.header = Header()
        self.header.frame_id = "world"

        self.colors = {"white":{"r":255,"g":255,"b":255},
                       "black":{"r":0,"g":0,"b":0},
                       "red":{"r":255,"g":0,"b":0},
                       "green":{"r":0,"g":255,"b":0},
                       "blue":{"r":0,"g":0,"b":255},
                       "yellow":{"r":255,"g":255,"b":0},
                       "cyan":{"r":0,"g":255,"b":255},
                       "magenta":{"r":255,"g":0,"b":255},
                       "sylver":{"r":192,"g":192,"b":192},
                       "gray":{"r":128,"g":128,"b":128},
                       "maroon":{"r":128,"g":0,"b":0},
                       "olive":{"r":128,"g":128,"b":0},
                       "purple":{"r":128,"g":0,"b":128},
                       "teal":{"r":0,"g":128,"b":128}}
                       
        self.models = {"sema_little_box": 
                        {"type": Marker().CUBE , "color": self.colors["yellow"], 
                        "size":{"x": 0.12, "y": 0.15, "z":0.12}},
                        "sema_middle_little_box": 
                        {"type": Marker().CUBE , "color": self.colors["purple"], 
                        "size":{"x": 0.16, "y": 0.19, "z":0.12}},
                        "sema_middle_box": 
                        {"type": Marker().CUBE , "color": self.colors["blue"], 
                        "size":{"x": 0.2, "y": 0.3, "z":0.15}},
                        "sema_big_middle_box":
                        {"type": Marker().CUBE , "color": self.colors["green"], 
                        "size":{"x": 0.3, "y": 0.4, "z":0.25}},
                        "sema_big_box":
                        {"type": Marker().CUBE , "color": self.colors["cyan"], 
                        "size":{"x": 0.5, "y": 0.3, "z":0.4}},
                        "sema_pallet":
                        {"type": Marker().CUBE , "color": self.colors["red"], 
                        "size":{"x": 0.8, "y": 1.2, "z":0.144}}}
        
        self.model_names = list(self.models.keys())
        
        self.robot_z = int(rospy.get_param(f"{rospy.get_name()}/robot_z"))

    def connections_init(self):
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.publish_markers)
        self.pub_markers = rospy.Publisher("/marker_array", MarkerArray, queue_size=5)


    def publish_markers(self, models_data):

        markerArray = MarkerArray()

        for _id ,model_name in enumerate(models_data.name):
            if model_name in self.model_names:
                viz_model = self.models[model_name]
                model_data = models_data.pose[_id]
                
                marker = self.create_obj_marker(_id, viz_model, model_data)
                markerArray.markers.append(marker) 


        self.header.stamp = rospy.Time.now()
        self.pub_markers.publish(markerArray)
    

    def create_obj_marker(self, _id, viz_model, model_data):
        
        marker = Marker()    
        marker.header = self.header
        marker.id = _id
        marker.action = marker.ADD
        marker.ns = ""
        marker.lifetime = rospy.Duration(1)
        
        marker.type = viz_model["type"]

        marker.scale.x = viz_model["size"]["x"]
        marker.scale.y = viz_model["size"]["y"]
        marker.scale.z = viz_model["size"]["z"]

        marker.pose.position.x = model_data.position.x
        marker.pose.position.y = model_data.position.y
        marker.pose.position.z = model_data.position.z

        marker.pose.orientation.x = model_data.orientation.x
        marker.pose.orientation.y = model_data.orientation.y
        marker.pose.orientation.z = model_data.orientation.z
        marker.pose.orientation.w = model_data.orientation.w

        marker.color.r = viz_model["color"]["r"]
        marker.color.g = viz_model["color"]["g"]
        marker.color.b = viz_model["color"]["b"]
        marker.color.a = 180

        return marker


if __name__ == '__main__':
    pub_markers = PubMarkers()