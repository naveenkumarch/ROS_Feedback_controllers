#!/usr/bin/env python2

from __future__ import division 
import rospy
from numpy import inf
from math import sqrt
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from time import sleep
""" Importing the required libraries for rospy """
from Fuzzy_Cnt_OA import fuzzy_controller_OA
""" Importing the obstacle avoidance fuzzy logic controller """

inf = float('inf')  		# Defining the inf value for handling infinte value from sensor reading 

 
def forwards(speed, turn):
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=100)
    rate = rospy.Rate(100)
    vel_x = Twist()
    vel_x.linear.x = speed
    vel_x.angular.z = turn
    pub.publish(vel_x)
    rate.sleep()

def callback(msg):
    cntrlr = fuzzy_controller_OA()				# Defining an object for the obstacle avoidance fuzzy controller  
    sensor_value = msg.ranges
    # Instead of selecting a specific position to determine the minimum distance, a range of values are considered to find the front right side, front left side & front center side distnace of the robot.
    current_front_right_small = sensor_value[670]
    current_front_center_small = sensor_value[701]
    current_front_left_small  = sensor_value[20] 
    for i in range(670, 700):
	if sensor_value[i] < current_front_right_small:
            current_front_right_small = sensor_value[i] 
   	else:
	    pass
    for i in range(701, 719):
	if sensor_value[i] < current_front_center_small:
            current_front_center_small = sensor_value[i] 
   	else:
	    pass
    for i in range(0,19):
        if sensor_value[i] < current_front_center_small:
            current_front_center_small = sensor_value[i] 
   	else:
	    pass
    for i in range(20, 50):
	if sensor_value[i] < current_front_left_small:
            current_front_left_small = sensor_value[i] 
   	else:
	    pass
    # If no edge is found then laser scan will send 'inf' as reading. For calculation purposes replacing inf with a largest value.
    if current_front_right_small == inf:
	current_front_right_small =10
    if current_front_center_small  == inf:
	current_front_center_small = 10
    if current_front_left_small  == inf:
	current_front_left_small = 5
    print("smalls", current_front_left_small,current_front_right_small)
    linear_vel, angular_vel = cntrlr.fuzzy_controller_FO(current_front_left_small,current_front_center_small,current_front_right_small)
    # Determining the linear and angular velocity from the obstacle avoidance fuzzy controller
    forwards(linear_vel, angular_vel)


def get_reading():
    sub = rospy.Subscriber('/scan', LaserScan, callback)
    rospy.spin()


if __name__ == '__main__':
    try:
        rospy.init_node('script', anonymous=True)
        while not rospy.is_shutdown():
            get_reading()
	    sleep(1)	
    except rospy.ROSInterruptException:
        pass
