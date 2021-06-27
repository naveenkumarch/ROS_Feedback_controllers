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
from Fuzzy_Cnt_OA import fuzzy_controller_OA
""" Importing the fuzzy logic controller classes for both right edge following and obstacle avoidance """

inf = float('inf')  		# Defining the inf value for handling infinte value from sensor reading

def forwards(speed, turn):
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=100)
    rate = rospy.Rate(100)
    vel_x = Twist()
    vel_x.linear.x = speed
    vel_x.angular.z = turn
    pub.publish(vel_x)
    rate.sleep()
""" A fuzzy coordinator function for merging both right edge following and obstacle avoidance behaviour """ 
def fuzzy_coordinator(Rig_frnt_dis,Rig_bck_dis,Frnt_left_dis,Frnt_center_dis,Frnt_right_dis):
    controller_1 =  fuzzy_controller_RE()
    # defining an object for right edge follwoing fuzzy logic controller class
    controller_2 =  fuzzy_controller_OA()
    # defining an object for obstacle avoidance fuzzy logic controller class
    linear_RE, angular_RE = controller_1.fuzzy_controller(Rig_frnt_dis,Rig_bck_dis)
    # calculating the linear speed and angualr speed output from right edge following fuzzy controller   
    linear_FO, angular_FO = controller_2.fuzzy_controller_FO(Frnt_left_dis,Frnt_center_dis,Frnt_right_dis)
    # calculating the linear speed and angualr speed output from obstacle avoidance fuzzy controller
    FO_min_dis = min(Frnt_left_dis,Frnt_center_dis,Frnt_right_dis)
    # Finding the min distance to obstacle from the obstacle avoidance controller inputs
    """ Calculating the firing strength for both obstacle avoidance and right edge following behaviours""" 
    if FO_min_dis<=0.5:
        FO_FS = 1.0
    elif FO_min_dis >0.5 and FO_min_dis <= 1:
        FO_FS = (1-FO_min_dis)/0.5
    else:
        FO_FS = 0.0
    if FO_min_dis <=0.5:
        RE_FS = 0.0
    elif FO_min_dis > 0.5 and FO_min_dis <=1:
        RE_FS = (FO_min_dis-0.5)/0.5    
    else: 
        RE_FS = 1.0
    print("FO MIn dis is ", FO_min_dis)
    """ calculating the amount of speed to be set based on the firing strength and the output being sent by both behaviours """
    linear_out  = ((linear_RE*RE_FS)+(linear_FO*FO_FS))/(RE_FS+FO_FS)
    angular_out = ((angular_RE*RE_FS)+(angular_FO*FO_FS))/(RE_FS+FO_FS)
    print("RE firing strength is ", RE_FS)
    print("FO firing strength is ",FO_FS)
    print("linear_out calculated is ", linear_out)
    print("angular_out calculated is ", angular_out)
    return linear_out,angular_out

def callback(msg):

    sensor_value = msg.ranges
    """ Determining the min distance to the right side edge from the robot current position """ 
    current_right_back_small = sensor_value[399]
    current_right_front_small = sensor_value[541]
    # Instead of selecting a specific position to determine the minimum distance, a range of values are considered to find right front side distance and right back side distance of the robot.
    for i in range(399, 540):
        if sensor_value[i] < current_right_back_small:
            location_right = i
            current_right_back_small = sensor_value[i]    
        else:
            pass
    for i in range(541, 680):
        if sensor_value[i] < current_right_front_small:
            location_right = i
            current_right_front_small = sensor_value[i]    
        else:
            pass
    # Instead of selecting a specific position to determine the minimum distance, a range of values are considered to find the front right side, front left side & front center side distnace of the robot.
    current_front_right_small  = sensor_value[681]
    current_front_center_small = sensor_value[701] 
    current_front_left_small   = sensor_value[20] 
    for i in range(681, 700):
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
    # If no obstacle is found then laser scan will send 'inf' as reading. For calulation purposes replacing inf values with largest value
    if current_front_right_small == inf:
	current_front_right_small = 50
    if current_front_left_small  == inf:
	current_front_left_small = 50
    # If no edge is found then laser scan will send 'inf' as reading. For calculation purposes replacing inf with a largest value.
    if current_right_back_small == inf:
        current_right_back_small = 5
    if current_right_front_small == inf:
        current_right_front_small = 5
    print("RE distances ", current_right_front_small,current_right_back_small)
    print("FO Distances ", current_front_left_small,current_front_right_small)
    print("/n")
    """ Calling the fuzzy coordinator function with the currently obtained inputs """
    linear_vel, angular_vel = fuzzy_coordinator(current_right_front_small,current_right_back_small,current_front_left_small,current_front_center_small,current_front_right_small)
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
