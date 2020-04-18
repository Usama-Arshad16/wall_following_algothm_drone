#!/usr/bin/env python

#Adding libraries
import time
import roslib
from geometry_msgs.msg import Twist
import rospy
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Range

#Defining varibales
hieght_speed = 0.3				#How fast the drone go up
desired_hieght=1.5				#height from the ground
height=0
right_distance_from_wall=0.6	#maintain distance from the right wall
front_distance_from_wall=0.7	#maintain this minimum distance from the front wall
right_speed = 0.6				#How fast the drone go right
forward_speed=0.2				#How fast the drone move forward
angular_speed=0.5				#How fast the drone rotate
left = 0
right = 0
back = 0
front = 0
right_front=0
right_back=0
n=0
m=0
PI = 3.1415926535897

#This function gets value from mini lidar for height
def height_callback(msg):
    global height
    height = msg.range
    return height

#It will maintain desired height from ground
def fly():
	height_factor=desired_hieght-height 		#Calculating the difference between the desired height and the actual height
	twist.linear.z = hieght_speed*height_factor #Give the above information to drone to decide how much It has to move up and down
	cmd_vel_pub.publish(twist)					#publish the height value

#This function gets value from lidar sensor and calculate the distances from walls
def scan_callback(msg):
    global left
    global right
    global front
    global back
    global right_front
    global right_back

    back = msg.ranges[len(msg.ranges)/2]				#Calculate the distance from back wall
    front = msg.ranges[0]								#Calculate the distance from front wall
    right = msg.ranges[len(msg.ranges)*3/4]				#Calculate the distance from right wall
    left = msg.ranges[len(msg.ranges)/4] 				#Calculate the distance from left wall
    right_back = msg.ranges[len(msg.ranges)*119/160] 	#Calculate the distance between right and back 
    right_front = msg.ranges[len(msg.ranges)*121/160] 	#Calculate the distance between right and front 

#Stops the drone
def stop():
    twist.linear.x = 0
    twist.linear.y = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = 0

#go to the right wall
def wall_selection():
    right_factor=right_distance_from_wall-right		#Calculating the difference between the desired right wall distance and the actual right wall distance
    twist.linear.y = -1*right_speed*right_factor	#Give the above information to drone to decide how much It has to move right
    cmd_vel_pub.publish(twist)
   
#It keeps the angle parrallel to the wall
def right_angle_adjust():
    speed=10
    angular_factor=right_back-right_front
    twist.angular.z = speed*angular_factor
    cmd_vel_pub.publish(twist)

#It moves the drone forward
def move_forward():
    if front>front_distance_from_wall:
        twist.linear.x = -1*forward_speed
        cmd_vel_pub.publish(twist)

#It rotates the drone to left when a sharp turn is ahead
def left_turn():
    twist.angular.z = angular_speed
    t0  = rospy.Time.now().to_sec()
    angle_travelled = 0

    while ( angle_travelled < PI/1.6 ):
        cmd_vel_pub.publish(twist)
        t1 = rospy.Time.now().to_sec()
        angle_travelled = angular_speed*(t1-t0)
    stop()
    rospy.sleep(0.5)

#If the drone strike with a wall then it will turn the robot
def semi_turn():
    twist.angular.z = angular_speed
    t0  = rospy.Time.now().to_sec()
    angle_travelled = 0

    while ( angle_travelled < PI/2.5 ):
        cmd_vel_pub.publish(twist)
        t1 = rospy.Time.now().to_sec()
        angle_travelled = angular_speed*(t1-t0)

#It decides where the drone has to move
def right_turn_algo():
    if front>front_distance_from_wall and right<right_distance_from_wall+0.3:
        move_forward()
        right_angle_adjust()
        print("1st Condition")
        wall_selection()

    elif front<front_distance_from_wall and right<right_distance_from_wall+0.3:
        stop()
        print("2nd Condition")
        rospy.sleep(1)
        left_turn()

    elif right>right_distance_from_wall+0.3:
        stop()
        rospy.sleep(0.1)
        wall_selection()
        right_angle_adjust()
        print("3rd Condition")

    elif front<0.49:
        stop()
        rospy.sleep(0.5)
        semi_turn()
        print("4th Condition")
    else:
        move_forward()
        print("5th Condition")

scan_sub = rospy.Subscriber('/scan', LaserScan, scan_callback)				#Subscribe to lidar sensor
height_sub = rospy.Subscriber('/sensor_hieght', Range, height_callback)		#Subscribe to mini lidar sensor
cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)				#Publisher the publish to cmd_vel
rospy.init_node('range_ahead')												#Initialize the node
twist = Twist()
rate = rospy.Rate(100)

#The loop that will work untill we press crl+c
while not rospy.is_shutdown():
    while n<=400:				#Provide the naccessary time to adjust hieght and right wall distance
    	fly()
        wall_selection()
        right_angle_adjust()
        rospy.sleep(0.01)
        n+=1
        print("Adjusting Height and selecting wall")
        pass
    fly()
    right_turn_algo()
    rospy.sleep(0.01)