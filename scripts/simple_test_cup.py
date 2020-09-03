#! /usr/bin/env python
from mankey_ros.srv import *

import rospy
import cv2
import os
from sensor_msgs.msg import Image, RegionOfInterest
from cv_bridge import CvBridge, CvBridgeError


def main():
    rospy.wait_for_service('detect_keypoints')
    detect_keypoint = rospy.ServiceProxy('detect_keypoints', MankeyKeypointDetection)

    # Get the test data path
    project_path = os.path.join(os.path.dirname(__file__), os.path.pardir)
    project_path = os.path.abspath(project_path)
    test_data_path = os.path.join(project_path, 'test_cup')
    cv_rbg_path = os.path.join(test_data_path, '001_cup_rgb.png')
    cv_depth_path = os.path.join(test_data_path, '001_cup_depth.png')

    # Read the image
    cv_rgb = cv2.imread(cv_rbg_path, cv2.IMREAD_COLOR)
    cv_depth = cv2.imread(cv_depth_path, cv2.IMREAD_ANYDEPTH)

    # The bounding box
    roi = RegionOfInterest()
    roi.x_offset = 299
    roi.y_offset = 353
    roi.width = 391 - 299
    roi.height = 438 - 353

    # Build the request
    request = MankeyKeypointDetectionRequest()
    bridge = CvBridge()
    request.rgb_image = bridge.cv2_to_imgmsg(cv_rgb, 'bgr8')
    request.depth_image = bridge.cv2_to_imgmsg(cv_depth)
    request.bounding_box = roi
    response = detect_keypoint(request)
    print response

    cv2.circle(cv_rgb, (334,371) , 2, color=(255,0,0), thickness=2)
    cv2.circle(cv_rgb, (376,390) , 2, color=(255,0,0), thickness=2)
    cv2.circle(cv_rgb, (333,409) , 2, color=(255,0,0), thickness=2)
    #cv2.circle(cv_rgb, (291,230) , 2, color=(255,0,0), thickness=2)
    cv2.namedWindow("image", cv2.WINDOW_AUTOSIZE)
    cv2.imshow("image", cv_rgb)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    cv2.waitKey(1)


if __name__ == '__main__':
    main()
