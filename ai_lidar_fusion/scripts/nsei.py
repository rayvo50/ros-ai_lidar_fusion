#!/usr/bin/env python

import rospy
import sys
from roslib import message
from std_msgs.msg import String, Header
import cv2 as cv
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
import numpy as np
from math import sqrt, tan, atan, pi


class GetInfo:

    def __init__(self):
        #self.info_sub = rospy.Subscriber('/info', String, self.callback)
        id = 1
        ptx = 530
        pty = 300
        cenas_ = [id, ptx, pty]
        self.cenas = cenas_

    def callback(self,data):
        cenas_ = data.data
        id = 1
        ptx = 530
        pty = 300
        cenas_ = [id, ptx, pty]
        self.cenas = cenas_   

    def get_info(self):
        return self.cenas    


class GetPointCloud:

    def __init__(self):
        self.points = []
        self.pc2_sub = rospy.Subscriber('lidar', PointCloud2, self.callback)

    def callback(self, data):
        #  get points from /lidar pointcloud msg
        assert isinstance(data, PointCloud2)
        gen = pc2.read_points(data, field_names=('x','y','z'), skip_nans=True)
        points_xyz = []
        for p in gen:
            #if p[0] < 0 and (not(p[0]>-2.0 and p[0]<2.0 and p[1]>-2.0 and p[1]<2.0 and p[2]>-2.0 and p[2]<2.0)):
                points_xyz.append(p)
        self.points = np.reshape(points_xyz, (len(points_xyz), 3))
        # print(self.points)
    
    def get_points(self):
        return self.points


class PubPointCloud:
    
    def __init__(self):
        self.pc2_pub = rospy.Publisher('testpc2', PointCloud2, queue_size=10)
    
    def pubpoints(self, points):
        # create test point cloud to visualize on rviz
        fields = [PointField('x', 0, PointField.FLOAT32, 1),
                    PointField('y', 4, PointField.FLOAT32, 1),
                    PointField('z', 8, PointField.FLOAT32, 1),
                    PointField('intensity', 12, PointField.FLOAT32, 1)]
        header = Header()
        header.frame_id = 'velodyne'
        header.stamp = rospy.Time.now()
        points = np.array([points[:,0], points[:,1], points[:,2], points[:,2]])
        points = points.reshape(4,-1).T
        pointcloud = pc2.create_cloud(header, fields, points)
        self.pc2_pub.publish(pointcloud)
        
        
# Function to improve on !!!
def getpointsub(points, angleH, angleV, desvio=0.04):
    # for now assume camera_pos = lidar_pos = origin (0, 0, 0)
    # calculate the angleH and angleV cooresponding to the x axis (x and y, x and z)
    # fist lets calculate angleH, the angle in the horizontal plane, starting on the x axis
    lim_sup_H = angleH + desvio
    lim_inf_H = angleH - desvio
    lim_sup_V = angleV + desvio
    lim_inf_V = angleV - desvio


    # actual calculation of angle and filter pointcloud to only points in the right angles
    pontos = []
    distances = []
    for i, p in enumerate(points):
      angH = atan(p[1]/(-p[0]))  # angle in the horizontal plane in rad
      angV = atan(p[2]/(-p[0]))  # angle in the vertical plane in rad
      dist = sqrt(pow(p[0],2) + pow(p[1],2) + pow(p[2],2) )  # distance to the origin
      if (angH > lim_inf_H and angH < lim_sup_H) and (angV > lim_inf_V and angV < lim_sup_V) :
        pontos.append(p)
        distances.append(dist)
    pontos = np.reshape(pontos, (len(pontos), 3))
    distances = np.reshape(distances, (len(pontos), 1))
    pontos2 = np.append(pontos, distances, axis=1)
    # print(distances)
    # print('\n')
    # print(pontos)
    # print('\n')
    # print(pontos2)
    # print('\n\n Done \n\n')

    return pontos2

def calc_distance(points):    
    if len(points) != 0:
      dist = np.mean(points[:,3])
    else: 
      dist = -1
    return dist


#  Calculates the angles (Vertical and Horizontal) of a given pixel.
#  The angle is measured from the axis wich is the center of the image
# @parametres:
#       res_h, res_w - resolution of given frame
#       ptx, pty - coordinates of the pixel to calculate angle
#       FOVx , FOVy - FOV values of the camera used
# 
# @returns:
#       angleH, angleV - angles calculated  
#
def calc_angle(res_h, res_w, ptx, pty, FOVx=62.11118, FOVy=37.42529, Degree=False):

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
    
    # if Display:
    #     cv.circle(img, (ptx, pty), 10, (0,255,0), thickness=cv.FILLED)
    #     info1 = 'x: {x} || AngleH: {a}'.format(x=ptx, a=angleH)
    #     info2 = 'y: {y} || AngleV: {a}'.format(y=pty, a=angleV)
    #     cv.putText(img, info1, (ptx+15, pty+15), cv.FONT_HERSHEY_PLAIN, 1, (0,255,0))
    #     cv.putText(img, info2, (ptx+15, pty+30), cv.FONT_HERSHEY_PLAIN, 1, (0,255,0))

    return [angleH, angleV]


def main(args):
    res_h, res_w = 360, 640

    # res_h, res_w = 360, 640 
    # print('check1')
    # rospy.init_node('Converter', anonymous=True)
    # print('check2')
    # myGetInfo = GetInfo()
    # print('check3')
    # myGetPointCloud = GetPointCloud()
    # print('check4')

    rospy.init_node('Converter', anonymous=True)
    myGetInfo = GetInfo()  
    myGetPointCloud = GetPointCloud()
    # getting information of required pixel and pointcloud from the respective topics
    info = myGetInfo.get_info()
    #print(info)
    points = myGetPointCloud.get_points()
    print(points)
    rospy.spin()

    # angles = calc_angle(res_h, res_w, info[1], info[2])
    # points_subset = getpointsub(points, angles[0], angles[1])
    # #print(points_subset)
    # distance = calc_distance(points_subset)
    # #print(distance)

    # try:
    #     rospy.spin()
    # except KeyboardInterrupt:
    #     print("Shutting down")


if __name__ == "__main__":
    main(sys.argv)     
