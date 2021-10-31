#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Header
from ai_lidar_fusion.msg import Information

def main():
    rospy.init_node('test_publisher', anonymous=True)
    pub = rospy.Publisher('test_info', Information, queue_size=10)
    rate = rospy.Rate(2)
    while not rospy.is_shutdown():
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = ':('
        header.seq

        # change the way the information is gathered 
        msg = Information()
        msg.header = header
        msg.cam_id = 'Starboard'
        msg.ptx = 127 #530
        msg.pty = 169 #300

        pub.publish(msg)
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
