#!/usr/bin/env python
 
import rospy
import sys
import numpy as np
from math import sqrt, atan, tan, pi
import sensor_msgs.point_cloud2 as pc2
from message_filters import Subscriber, ApproximateTimeSynchronizer
from std_msgs.msg import String, Header
from sensor_msgs.msg import PointCloud2, PointField, Image
from ai_lidar_fusion.msg import Information
 
 
class Converter():
    def __init__(self, cloud_pub):
        self.cloud_pub = cloud_pub
 
    def callback(self, info, cloud):
        res_h, res_w = 360, 640
        
        assert isinstance(cloud, PointCloud2)
        gen = pc2.read_points(cloud, field_names=('x','y','z'), skip_nans=True)
        points_xyz = []
        for p in gen:
            if not(p[0]>-2.0 and p[0]<2.0 and p[1]>-2.0 and p[1]<2.0 and p[2]>-2.0 and p[2]<2.0):
                points_xyz.append(p)
        points_xyz = np.reshape(points_xyz, (len(points_xyz), 3))
 
        # process stuff
        angleH, angleV = calc_angle(res_h, res_w, info.ptx, info.pty)
        print(angleH, angleV)
 
        points_d = getpointsub(points_xyz, angleH, angleV, info.cam_id)
        pub_pointcloud(points_d, self.cloud_pub)
 
        distance = calc_distance(points_d)
        rospy.loginfo(distance)
 
    
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
 
    return (angleH, angleV)
 
 
# Function to improve on !!!
def getpointsub(points, angleH, angleV, cam_id, desvio=0.04):
    # for now assume camera_pos = lidar_pos = origin (0, 0, 0)
    lim_sup_H = angleH + desvio
    lim_inf_H = angleH - desvio
    lim_sup_V = angleV + desvio
    lim_inf_V = angleV - desvio
    
    # calculation of angle and filter pointcloud to only points in the right angles
    pontos = []
    distances = []
    for i, p in enumerate(points):
        if cam_id == 'Front':
            angH = atan(p[0]/p[1])      # atan( x / y )
            angV = atan(p[2]/p[1])      # atan( z / y )
            dist = sqrt(pow(p[0],2) + pow(p[1],2) + pow(p[2],2) )  # distance to the origin
            if (p[1] > 0) and (angH > lim_inf_H and angH < lim_sup_H) and (angV > lim_inf_V and angV < lim_sup_V) :
                pontos.append(p)
                distances.append(dist)
        elif cam_id == 'Back':
            angH = atan(-p[0]/(-p[1]))  # atan( -x / -y )
            angV = atan(p[2]/(-p[1]))   # atan( z / -y )
            dist = sqrt(pow(p[0],2) + pow(p[1],2) + pow(p[2],2) )  # distance to the origin
            if (p[1] < 0) and (angH > lim_inf_H and angH < lim_sup_H) and (angV > lim_inf_V and angV < lim_sup_V) :
                pontos.append(p)
                distances.append(dist)            
        elif cam_id == 'Port':
            angH = atan(p[1]/(-p[0]))   # atan( y / -x )
            angV = atan(p[2]/(-p[0]))   # atan( z / -x )
            dist = sqrt(pow(p[0],2) + pow(p[1],2) + pow(p[2],2) )  # distance to the origin
            if (p[0] < 0) and (angH > lim_inf_H and angH < lim_sup_H) and (angV > lim_inf_V and angV < lim_sup_V) :
                pontos.append(p)
                distances.append(dist)
        elif cam_id == 'Starboard':
            angH = atan(-p[1]/p[0])     # atan( -y / x )
            angV = atan(p[2]/p[0])      # atan( z / x )
            dist = sqrt(pow(p[0],2) + pow(p[1],2) + pow(p[2],2) )  # distance to the origin
            if (p[0] > 0) and (angH > lim_inf_H and angH < lim_sup_H) and (angV > lim_inf_V and angV < lim_sup_V) :
                pontos.append(p)
                distances.append(dist)
        else:
            pass
    pontos = np.reshape(pontos, (len(pontos), 3))
    distances = np.reshape(distances, (len(pontos), 1))
    pontos2 = np.append(pontos, distances, axis=1)
 
    return pontos2
 
 
def calc_distance(points):    
    if len(points) != 0:
      return np.mean(points[:,3])
    else: 
      return -1
 
 
def pub_pointcloud(pontos, publisher):
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
    publisher.publish(pointcloud)
 
 
def main(args):
    rospy.init_node('sync_test_yo')
    cloud_sub = Subscriber('lidar', PointCloud2)
    info_sub = Subscriber('test_info', Information)
 
    cloud_pub = rospy.Publisher('testpc2', PointCloud2, queue_size=10)
    
    converter = Converter(cloud_pub)
 
    ats = ApproximateTimeSynchronizer([info_sub, cloud_sub], queue_size=10, slop=0.3, allow_headerless=False)
    ats.registerCallback(converter.callback)
 
    rospy.spin()
    
 
if __name__ == "__main__":
    try:
        main(sys.argv)     
    except rospy.ROSInterruptException:
        pass