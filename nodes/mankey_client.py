#!/usr/bin//env python
from mankey_ros.srv import *

import rospy
import cv2
import os
from sensor_msgs.msg import Image, RegionOfInterest
from cv_bridge import CvBridge, CvBridgeError
from jsk_recognition_msgs.msg import RectArray
from geometry_msgs.msg import PoseArray, Point, Pose
import message_filters

class depth_rgb_to_3d():
    def __init__(self):
        rospy.wait_for_service("detect_keypoints")
        try:
            self.detect_keypoint = rospy.ServiceProxy("detect_keypoints", MankeyKeypointDetection)
            self.subscribe()
            rospy.Rate(100)
            self.pub = rospy.Publisher("~output", PoseArray , queue_size=1)

        except rospy.ServiceException, e:
            print("Service call failed: %s"%e)

    def subscribe(self):
        #print("aa")
        self.sub_rgb = message_filters.Subscriber("camera/rgb/image_rect_color", Image, queue_size=10)
        self.sub_depth = message_filters.Subscriber("camera/depth_registered/sw_registered/image_rect_raw", Image, queue_size=10)
        self.sub_rect = message_filters.Subscriber("~input_rect", RectArray, queue_size=10)
        self.subs = [self.sub_rgb, self.sub_depth, self.sub_rect]
        #self.subs = [self.sub_rgb, self.sub_depth]

        if rospy.get_param("~approximate_sync", True):
            slop = rospy.get_param("~slop", 0.7)
            sync = message_filters.ApproximateTimeSynchronizer(
                fs=self.subs, queue_size=10, slop=slop)
        else:
            sync = message_filters.TimeSynchronizer(
                fs=self.subs, queue_size=10)
        sync.registerCallback(self.callback)

    def callback(self, img_msg, depth_msg, rect_msg):
        #print("bb")
        msg = PoseArray()
        msg.header = img_msg.header
        
        request = MankeyKeypointDetectionRequest()
        bridge = CvBridge()
        request.rgb_image = img_msg
        request.depth_image = depth_msg
        request.bounding_box.x_offset = rect_msg.rects[0].x
        request.bounding_box.y_offset = rect_msg.rects[0].y
        request.bounding_box.height = rect_msg.rects[0].height
        request.bounding_box.width = rect_msg.rects[0].width
        response = self.detect_keypoint(request)
        print(response)

        for i in range(response.num_keypoints):
            point = response.keypoints_camera_frame[i]
            print(point)
            if point:
                msg.poses.append(Pose(position=point))
        self.pub.publish(msg)

if __name__ == "__main__":
    rospy.init_node("depth_rgb_to_3d")
    depth_rgb_to_3d()
    rospy.spin()
