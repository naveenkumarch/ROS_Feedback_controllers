#!/usr/bin/env python2

from __future__ import division 
import rospy
from numpy import inf
from math import sqrt
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from time import sleep
""" Importing the required libraries for rospy """
from Fuzzy_Cnt_REF import fuzzy_controller_RE 
""" Importing the right edge following fuzzy logic controller """

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
    controller = fuzzy_controller_RE()                                       # Defining an object for the Right edge following fuzzy controller
    sensor_value = msg.ranges
    # Instead of selecting a specific position to determine the minimum distance, a range of values are considered to find right front side distance and right back side distance of the robot.
    current_right_back_small = sensor_value[390]
    current_right_front_small = sensor_value[541]
    # If no obstacle is found then laser scan will send 'inf' as reading. For calulation purposes replacing inf values with largest value
    if current_right_back_small == inf:
	current_right_back_small = 5
    if current_right_front_small == inf:
	current_right_front_small = 5
    for i in range(390, 540):
        if sensor_value[i] < current_right_back_small:
            current_right_back_small = sensor_value[i]    
        else:
            pass
    for i in range(541, 719):
        if sensor_value[i] < current_right_front_small:
            current_right_front_small = sensor_value[i]    
        else:
            pass
    print("Front small distance ", current_right_front_small)
    print("back small distance", current_right_back_small)

    linear_vel, angular_vel = controller.fuzzy_controller(current_right_front_small,current_right_back_small)
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
