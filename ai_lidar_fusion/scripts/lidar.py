#!/usr/bin/env python

import rospy
import sys
from roslib import message
from std_msgs.msg import String, Header
from sensor_msgs.msg import Image, PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
import numpy as np
from math import sqrt, tan, atan, pi



class point_cloud_converter:

  def __init__(self):
    self.pc2_sub = rospy.Subscriber('lidar', PointCloud2, self.callback)
    self.pc2_pub = rospy.Publisher('testpc2', PointCloud2, queue_size=10)

  def callback(self, data):

    #  get points from /lidar pointcloud msg
    assert isinstance(data, PointCloud2)
    gen = pc2.read_points(data, field_names=('x','y','z'), skip_nans=True)
    points_xyz = []
    for p in gen:
      if p[0] < 0 and (not(p[0]>-2.0 and p[0]<2.0 and p[1]>-2.0 and p[1]<2.0 and p[2]>-2.0 and p[2]<2.0)):
        points_xyz.append(p)
    points_xyz = np.reshape(points_xyz, (len(points_xyz), 3))
    

    # for now assume camera_pos = lidar_pos = origin (0, 0, 0)
    # calculate the angleH and angleV cooresponding to the x axis (x and y, and x and z)
    # fist lets calculate angleH, the angle in the horizontal plane, starting on the x axis

    # FOVx=62.11118, FOVy=37.42529

    # limits of camera FOV, aka, what lidar points the camera can see
    # lim_sup_H = 62.11118/2*pi/180
    # lim_inf_H = -lim_sup_H
    # lim_sup_V = 37.42529/2*pi/180
    # lim_inf_V = -lim_sup_H

    # custom angle value, aka, the zone where the pixel is
    
    
    ang_H = 21.563033535869824*pi/180
    ang_V = -12.724997111365617*pi/180
    desvio = 0.04

    lim_sup_H = ang_H + desvio
    lim_inf_H = ang_H - desvio
    lim_sup_V = ang_V + desvio
    lim_inf_V = ang_V - desvio


    # actual calculation of angle and filter pointcloud to only points in the right angles
    pontos = []
    distances = []
    for i, p in enumerate(points_xyz):
        angH = atan(p[1]/(-p[0]))  # angle in the horizontal plane in rad
        angV = atan(p[2]/(-p[0]))  # angle in the vertical plane in rad
        dist = sqrt(pow(p[0],2) + pow(p[1],2) + pow(p[2],2) )  # distance to the origin
        #if (angH > lim_inf_H and angH < lim_sup_H) and (angV > lim_inf_V and angV < lim_sup_V) :
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
    
    if len(distances) != 0:
      distancia = np.mean(distances)
    else: 
      distancia = -1

    print(distances)
    print(distancia)


    # create test point cloud to visualize on rviz
    fields = [PointField('x', 0, PointField.FLOAT32, 1),
              PointField('y', 4, PointField.FLOAT32, 1),
              PointField('z', 8, PointField.FLOAT32, 1),
              PointField('intensity', 12, PointField.FLOAT32, 1)]
    header = Header()
    header.frame_id = 'velodyne'
    header.stamp = rospy.Time.now()
    points = np.array([pontos[:,0], pontos[:,1], pontos[:,2], pontos[:,2]])
    points = points.reshape(4,-1).T
    pointcloud = pc2.create_cloud(header, fields, points)
    self.pc2_pub.publish(pointcloud)



    # dist = np.abs(points_xyz)
    # for i in range(len(dist)):
    #   if dist[i] != 0:
    #     print(dist)    
    
    # assert isinstance(data, PointCloud2)
    # gen = pc2.read_points(data)
    # print(type(gen))
    # for p in gen:
    #   print(p)  # type depends on your data type, first three entries are probably x,y,z  




def main(args):
    rospy.init_node('pc2converter', anonymous=True)
    pcc = point_cloud_converter()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")




if __name__ == "__main__":
    try:
        main(sys.argv)     
    except rospy.ROSInterruptException:
        pass

