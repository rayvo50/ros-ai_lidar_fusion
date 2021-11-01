#!/usr/bin/env python

import rospy
import sys
from std_msgs.msg import String
from sensor_msgs.msg import Image
import cv2 as cv
from cv_bridge import CvBridge, CvBridgeError
from math import tan, atan, pi

# camera parametres
#FOV_x = 62.11118
#FOV_y = 37.42529


class Converter:

  def __init__(self):
    #self.angles_pub = rospy.Publisher("image_topic_2",Image)

    self.bridge = CvBridge()
    self.port_sub = rospy.Subscriber('/optical/Starboard/image_raw', Image, self.callback)

  def callback(self,data):
    try:
      frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    pixel = get_pixel()
    h, w, c = frame.shape
    angles = calc_angle(frame, *pixel, Degree=True)
    print(angles)

    cv.imshow('Image window', frame)
    cv.waitKey(2)

    #try:
    #   self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    # except CvBridgeError as e:
    #   print(e)


#  provisory function to basically indicate what pixel is to be considered
def get_pixel():
    x, y = 530, 300
    return (x,y)

def calc_angle(img, ptx, pty, FOVx=62.11118, FOVy=37.42529, Degree=False, Display=True):

    res_h, res_w, _ = img.shape
    print(img.shape)

    #  convert FOV to radians
    fovx = FOVx/180*pi
    fovy = FOVy/180*pi  

    #  Horizontal angle
    dipH = (res_w/2)/tan(fovx/2)
    x = -(res_w/2 - ptx)
    angleH = atan(x/dipH)  
    #  Vertical angle
    dipV = (res_h/2)/tan(fovy/2)
    y = (res_h/2 - pty)
    angleV = atan(y/dipV) 

    #  convert angles to degrees
    if Degree:
        angleH = angleH/pi*180
        angleV = angleV/pi*180
    
    #if Display:
        cv.circle(img, (ptx, pty), 10, (0,255,0), thickness=cv.FILLED)
        info1 = 'x: {x} || AngleH: {a}'.format(x=ptx, a=angleH)
        info2 = 'y: {y} || AngleV: {a}'.format(y=pty, a=angleV)
        cv.putText(img, info1, (ptx+15, pty+15), cv.FONT_HERSHEY_PLAIN, 1, (0,255,0))
        cv.putText(img, info2, (ptx+15, pty+30), cv.FONT_HERSHEY_PLAIN, 1, (0,255,0))

    return (angleH, angleV)


def main(args):
    rospy.init_node('Converter', anonymous=True)
    ic = Converter()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv.destroyAllWindows()


if __name__ == "__main__":
    try:
        main(sys.argv)     
    except rospy.ROSInterruptException:
        pass
