#!/usr/bin/env python

import rospy
import sys

import cv2 as cv
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from message_filters import Subscriber, ApproximateTimeSynchronizer


#  Change here to see other cameras ('Front', 'Back', 'Port', 'Starboard')
cam = 'Port'


class Displayer():
    def __init__(self):
        self.bridge = CvBridge()

    def display_callback(self, front, back, port, starboard):
        if cam == 'Front':
            try:
                frame = self.bridge.imgmsg_to_cv2(front, "bgr8")
            except CvBridgeError as e:
                print(e)
        elif cam == 'Back':
            try:
                frame = self.bridge.imgmsg_to_cv2(back, "bgr8")
            except CvBridgeError as e:
                print(e)
        elif cam == 'Port':
            try:
                frame = self.bridge.imgmsg_to_cv2(port, "bgr8")
            except CvBridgeError as e:
                print(e)
        elif cam == 'Starboard':
            try:
                frame = self.bridge.imgmsg_to_cv2(starboard, "bgr8")
            except CvBridgeError as e:
                print(e)

        cv.imshow('Camera View', frame)
        cv.waitKey(2)

def main(args):
    rospy.init_node('cam_display')
    front_sub = Subscriber('/optical/Front/image_raw', Image)
    back_sub = Subscriber('/optical/Back/image_raw', Image)
    port_sub = Subscriber('/optical/Port/image_raw', Image)
    starboard_sub = Subscriber('/optical/Starboard/image_raw', Image)

    displayer = Displayer()

    ats = ApproximateTimeSynchronizer([front_sub, back_sub, port_sub, starboard_sub], queue_size=5, slop=0.3, allow_headerless=False)
    ats.registerCallback(displayer.display_callback)

    rospy.spin()


if __name__ == "__main__":
    try:
        main(sys.argv)     
    except rospy.ROSInterruptException:
        pass