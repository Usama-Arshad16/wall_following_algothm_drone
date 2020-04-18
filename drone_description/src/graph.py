#!/usr/bin/env python

import time
import roslib
import rospy
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Range
from matplotlib import pyplot as plt
import matplotlib.animation as animation

def height_callback(msg):
    global height
    height = msg.range
    return height

def scan_callback(msg):
    global right
    right = msg.ranges[len(msg.ranges)*3/4]				#Calculate the distance from righti wall

scan_sub = rospy.Subscriber('/scan', LaserScan, scan_callback)				#Subscribe to lidar sensor
height_sub = rospy.Subscriber('/sensor_hieght', Range, height_callback)	
rospy.init_node('graph')

fig=plt.figure()
plt.axis([0,1000,0,1])

n=0
T = list()
R = list()
H = list()

while not rospy.is_shutdown():
	H.append(height);
	R.append(right);
	T.append(n);
	plt.cla()
	plt.plot(T,R,label="righti_Distance");
	plt.plot(T,H,label="Hieght")
	plt.grid(True)
	plt.xlim(left = max(0,n-125))
	plt.xlim(right = n+50)
	plt.pause(0.01)
	print(n)
	n+=1;