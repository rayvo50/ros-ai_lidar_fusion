#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Header
from ai_lidar_fusion.msg import Information


#  This is a simple publisher node for testing the ai_lidar_fusion. 
#  It publishes a "Information" message that contains the coordinates of a pixel 
#  and the id of the camera. In order to test the ai_lidar_fusion node change
#  lines 27-29 to the desired values.
#  To see the pixel that corresponds to what you want to test use camera.py


def main():
    rospy.init_node('test_publisher', anonymous=True)
    pub = rospy.Publisher('test_info', Information, queue_size=10)
    rate = rospy.Rate(3)
    while not rospy.is_shutdown():
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = ':)'
        header.seq

        msg = Information()
        msg.header = header
        msg.cam_id = 'Starboard'
        msg.ptx = 127  # c
        msg.pty = 169 

        pub.publish(msg)
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

