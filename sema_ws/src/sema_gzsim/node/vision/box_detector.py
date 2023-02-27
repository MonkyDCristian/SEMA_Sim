#!/usr/bin/env python3

"""
referent: https://docs.opencv.org/4.x/da/d0c/tutorial_bounding_rects_circles.html
"""

import rospy
import cv2
import numpy as np
import random as rng

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Empty

from sema_gzsim.cfg import box_detectorConfig 
from dynamic_reconfigure.server import Server as DRServer

from sema_gzsim.conveyor_belt_vel_ctrl import ConveyorBeltVelocityCtrl


class BoxDetector(object):

    def __init__(self):
        rospy.init_node('box_detector')
        self.variables_init()
        self.connections_init()
        rospy.spin()

    
    def variables_init(self):
        self.w, self.h = None, None
        self.radius = 20
        self.canny_threshold = 40
        self.blur_threshold = 5
        self.min_area, self.max_area = 10, 100
        self.min_distance = 10
        self.active = True
        
        self.bridge = CvBridge()
        self.cb_vel_ctrl = ConveyorBeltVelocityCtrl()

               
    def connections_init(self):
        # publishers
        self.pub_img = rospy.Publisher("box_detector/color/img", Image, queue_size=5)
        self.pub_bip = rospy.Publisher("box_in_position" , Empty, queue_size=2)

        # subscriber
        rospy.Subscriber("/camera/color/image_raw", Image, self.img_callback)
        rospy.Subscriber("box_detector/active", Empty, self.active_callback)

        srv = DRServer(box_detectorConfig, self.dynamic_config_callback)
    

    def dynamic_config_callback(self, cfg, level):
        self.canny_threshold = cfg["canny_threshold"]
        self.blur_threshold = cfg["blur_threshold"]
        self.max_area = cfg["max_area"]
        self.min_area = cfg["min_area"]
        self.min_distance = cfg["min_distance"]
       
        return cfg


    def img_callback(self, img_msg):
        img_cv2 = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
        self.h, self.w = img_cv2.shape[:2]
        self.run(img_cv2)
    

    def active_callback(self, msg):
        self.start_conveyor()
        rospy.sleep(0.5)
        self.active = True

        
    def run(self, img_cv2):
        list_contours, key_points = self.contuor_finding(img_cv2)
        list_contours = self.filter_by_shape(list_contours)
        self.draw_contours(list_contours, img_cv2)

        img_cv2 = cv2.rectangle(img_cv2, (self.w//2, 0), (self.w//2, self.h), (0, 0, 255), 2)

        if self.active:
            self.trigger_pick(list_contours)
        
        self.publish_img(img_cv2)
        

    def contuor_finding(self, img):
        
        if len(img.shape) == 2:
            img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

        img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        img_blur = cv2.blur(img_gray, (self.blur_threshold ,self.blur_threshold))
        img_canny = cv2.Canny(img_blur, self.canny_threshold, self.canny_threshold*2)
        
        contours, _ = cv2.findContours(img_canny, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        return self.get_contuor_features(contours)
    

    def get_contuor_features(self, contours):
        list_contours = []
        list_keypoints = np.array([])
        
        for n, c in enumerate(contours):
            contours_poly = cv2.approxPolyDP(c, 3, True)
            centers, radius = cv2.minEnclosingCircle(contours_poly)
            
            list_contours.append({"contour": c, 
                                  "contours_poly": contours_poly,
                                  "boundRect":cv2.boundingRect(contours_poly),
                                  "area": cv2.contourArea(contours_poly),
                                  "centers":centers, 
                                  "radius":radius})
            
            if n == 0:
                list_keypoints = contours_poly
            
            else:
                list_keypoints = np.concatenate((list_keypoints, contours_poly), axis=0)

        return list_contours, list_keypoints


    def filter_by_shape(self, list_data):
    
        return list(filter(lambda x: self.min_area < x["area"] < self.max_area, list_data))
    

    def draw_contours(self, list_contours, frame):
        
        for i, c_data in enumerate(list_contours):
            color = (rng.randint(0,256), rng.randint(0,256), rng.randint(0,256))
            cv2.drawContours(frame, c_data["contours_poly"], -1, color, 3)
            
            pt1 = (int(c_data["boundRect"][0]), int(c_data["boundRect"][1]))
            pt2 = (int(c_data["boundRect"][0]+c_data["boundRect"][2]), int(c_data["boundRect"][1]+c_data["boundRect"][3]))

            cv2.rectangle(frame, pt1, pt2, color, 2)
            
            center = (int(c_data["centers"][0]), int(c_data["centers"][1]))
            radius = int(c_data["radius"])
            
            cv2.circle(frame, center, radius, color, 2)
            cv2.circle(frame, center, self.radius, color, -1)
        
        return frame


    def trigger_pick(self, list_contours):
        for data in list_contours:
            center_x = data["centers"][0] #x
            dist = np.abs(self.w//2 - center_x)
            print(dist)

            if dist < self.min_distance:
                self.stop_conveyor()
                self.active = False
                self.pub_bip.publish()
            
            break


    def publish_img(self, img_cv2):
        img_msg = self.bridge.cv2_to_imgmsg(img_cv2,"bgr8")
        self.pub_img.publish(img_msg)

    
    def stop_conveyor(self):
        self.cb_vel_ctrl.run(0.0)
        
    
    def start_conveyor(self):
        self.cb_vel_ctrl.run(0.1)
            

if __name__ == '__main__':
    box_detector = BoxDetector()



