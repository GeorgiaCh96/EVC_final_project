#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge
import cv2
import time
import math
import os
import sys
import yaml
import webcolors
from scipy.spatial import KDTree
import numpy as np

class UnifiedMotionCameraNode:
    def __init__(self):
        rospy.init_node("unified_motion_camera_node")
        self.bridge = CvBridge()
        self.firstStraight = True

        self.image_sub = rospy.Subscriber(
            "/camera/processed_image_colour", CompressedImage, self.image_callback, queue_size=1
        )

        rospy.loginfo("Unified node initialized!")

    def image_callback(self, msg):

        try:
        # Convert compressed image to OpenCV format
            np_arr = np.frombuffer(msg.data, np.uint8)
            img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            cv2.imshow("Undistorted", img)
            cv2.waitKey(1)
        except Exception as e:
            rospy.logerr("Image processing error: {}".format(e))

if __name__ == '__main__':
    try:
        UnifiedMotionCameraNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
