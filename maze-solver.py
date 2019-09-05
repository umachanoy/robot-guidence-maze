#!/usr/bin/env python
import rospy
import cv2
import numpy as np
import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

def Lidar(laser):
    img = np.zeros((500, 500,3), np.uint8)
    angle = laser.angle_max
    Tx = 250
    Ty = 250
    for d in laser.ranges:
        if d == float ('Inf'):
            d = laser.range_max
        a = math.trunc( (d * 10)*math.sin(angle + (-90*3.1416/180)) )
        b = math.trunc( (d * 10)*math.cos(angle + (90*3.1416/180)) )
        cv2.line(img,(250, 250),(a+250,b+250),(255,0,0),1)
        Tx+=a
        Ty+=b
        ang= ang - laser.angle_increment

    cv2.line(img,(250, 250),(250+Tx,250+Ty),(0,0,255),3)
    ang = -(math.atan2(Ty,Tx)-3.1416)*180/3.1416
    vel = Twist() 
    vel.angular.z = ang
    pub.publish(msg)
    cv2.imshow('img',img)
    cv2.waitKey(1)


def laser_listener():
    rospy.init_node('laser_listener', anonymous=True)
    rospy.Subscriber("/scan", LaserScan, Lidar)
    rospy.spin()

if __name__ == '__main__':
    laser_listener()