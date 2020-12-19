#!/usr/bin/env python3

import sys
import cv2
import rospy
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage
from duckietown_msgs.msg import Twist2DStamped

class Follower:
    def __init__(self):
        self.bridge = CvBridge()
        rospy.Subscriber("/coptic/camera_node/image/compressed", CompressedImage, self.callback, queue_size=1, buff_size=2**24)
        self.pub1 = rospy.Publisher("/coptic/car_cmd_switch_node/cmd", Twist2DStamped, queue_size=10)
        self.pub2 = rospy.Publisher("/coptic/camera_node/image/circle", Image, queue_size=10)
        self.twist = Twist2DStamped()

    def callback(self, msg):
        image = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_yellow = np.array([ 10, 10, 10])
        upper_yellow = np.array([255, 255, 250])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        h, w, d = image.shape
        search_top = 3*int(h/4)
        search_bot = 3*int(h/4) + 20
        mask[0:search_top, 0:w] = 0
        mask[search_bot:h, 0:w] = 0
        M = cv2.moments(mask)

        if M['m00'] > 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])

            cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
            final = self.bridge.cv2_to_imgmsg(image, "bgr8")
            self.pub2.publish(final)

            delta = cx - w/2
            self.twist.v = 0.2
            self.twist.omega = -float(delta) / 100
            self.pub1.publish(self.twist)
            cv2.waitKey(3)

if __name__=="__main__":
    rospy.init_node('line_following')
    follower = Follower()
    rospy.spin()

