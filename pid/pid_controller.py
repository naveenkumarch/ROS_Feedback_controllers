#!/usr/bin/env python2

from __future__ import division 
import rospy
from numpy import inf
from math import sqrt
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from time import sleep
""" Importing the required libraries for rospy """

inf = float('inf')  		# Defining the inf value for handling infinte value from sensor reading


def forwards(speed, turn):
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=100)
    rate = rospy.Rate(100)
    vel_x = Twist()
    vel_x.linear.x = speed
    vel_x.angular.z = turn
    pub.publish(vel_x)
    rate.sleep()


RightEdge_MIN_Value = 1.0  	#Minimum distance to maintain from right edge
FrontEnd_MIN_Value = 1.0  	#Minimum distance to maintain from obstacle

diff_error_d = 0 		    # Differentiation error for linear speed initialization
integ_error_d = 0  		    # Integral erorr component for linear speed initialization
prev_error_d = 0  		    # Initialization of previous state error for linear speed 
current_error_d = 0  	    # Initialization of current state error for linear speed
diff_error_a = 0 		    # Differentiation error for angular speed initialization
integ_error_a = 0  		    # Integral error component for angular speed initialization
prev_error_a = 0  		    # Initialization of previous state error for angular speed 
current_error_a = 0  	    # Initialization of current state error for angualr speed

kp_d = 0.05 			    # Proportional component parameter for linear speed PID controller 
ki_d = 0.00000001		    # Integral component parameter for linear speed PID controller
kd_d = 0.05			        # Differentiation component parameter for linear speed PID controller

kp_a = 0.3                  # Proportional component parameter for angular speed PID controller
ki_a = 0.000001             # Integral component parameter for angular speed PID controller
kd_a = 0.25                 # Differentiation component parameter for angular speed PID controller
count = 0

linear_speed  = 0.5         # Setting initial linear base speed
angular_speed = 0.0		    # Setting initial angular speed


def pid(sensor_value):
    location_right = 379
    location_front = 700
    global RightEdge_MIN_Value  # Getting Global values 
    global FrontEnd_MIN_Value
    global linear_speed
    global diff_error_d
    global integ_error_d
    global prev_error_d
    global current_error_d
    global kp_d
    global Ki_d
    global kd_d
    global diff_error_a
    global integ_error_a
    global prev_error_a
    global current_error_a
    global kp_a
    global Ki_a
    global kd_a
    global angular_speed
    global count
    
    current_right_small = sensor_value[379]
    # Instead of selecting a specific position to determine the minimum distance, a range of values are considered to find right side minimum distance and location of minimum distance.
    for i in range(379, 699):
        if sensor_value[i] < current_right_small:
            location_right = i
            current_right_small = sensor_value[i]
	    
        else:
            pass
    current_front_small = sensor_value[700]
    # Instead of selecting a specific position to determine the minimum distance, a range of values are considered to find front obstacle minimum distance and location of minimum distance.
    for i in range(700, 719):
	if sensor_value[i] < current_front_small:
	    location_front = i
            current_front_small = sensor_value[i] 
   	else:
	    pass
    for i in range(0, 20):
	if sensor_value[i] < current_front_small:
	    location_front = i
            current_front_small = sensor_value[i] 
   	else:
	    pass
    print('right_side_small_distance', current_right_small)
    print('sensor location:', location_right)
    print('front_side_small_distance', current_front_small)
    print('sensor location:', location_front)
    # if no edge is found then laser scan sends 'inf' as distance value. Replacing inf with least posible value for calculations 
    if current_right_small == inf:
	current_right_small = 0
        location_right = 379
    # calculating the distance error by subtracting the desired distance to be mainted from wall.
    Right_Edge_error = current_right_small - RightEdge_MIN_Value
    current_error_d = Right_Edge_error
    """ The calculated distance erorr along with the position where min distnace was found is used to calculate the angular velocity error"""
    # if min distnsce position found is less than 540 then robot is liclined towards left and needs to take a right turn to correct itslef.
    if location_right < 540:
        # if the distance error is less then only small correction is required so only 75% of the error value is used for new setpoint calculation
	if Right_Edge_error < 0.5:
        	current_error_a = ((location_right - 700)/50)*0.75
	else:
		current_error_a = (location_right - 700)/50
    elif location_right > 610:
        # if the min distance position found is greather than 610 then robot is already inclined towards right. Now based on distance error we can decide whether to increase error or send it as same 
        current_error_a = -(540 - location_right)/50
	if Right_Edge_error < 0:
		current_error_a = current_error_a+(current_error_a*0.25)
	else :
		pass
    # if min distance position is found in between the above two ranges then robot is almost paralley aligined to the wall. now based on distance error decision will be taken whether to take a right turn or left turn
    elif Right_Edge_error > 0.05:
	current_error_a = (540 - location_right)/75
    elif Right_Edge_error < -0.25:
	current_error_a = (610 - location_right)/75
    else:
        current_error_a = 0
 
    integ_error_d = integ_error_d + current_error_d                                     # Integrating error of distance over all cycles
    diff_error_d = current_error_d - prev_error_d                                       # calculating differential error of distance for current state 
    prev_error_d = current_error_d                                                      # saving current state error to previous state error variable
    integ_error_a = integ_error_a + current_error_a                                     # Integrating error of position over all cycles                                  
    diff_error_a = current_error_a - prev_error_a                                       # calculating differential error of position for  current state
    prev_error_a = current_error_a                                                      # saving current state error to previous state error variable
    print(current_error_d, integ_error_d, diff_error_d )
    print(current_error_a, integ_error_a, diff_error_a )
    linear_vel = linear_speed + (kp_d * current_error_d + ki_d * integ_error_d + kd_d * diff_error_d)
    angular_vel =0.0 + (kp_a* current_error_a + ki_a * integ_error_a + kd_a * diff_error_a)
    # calculated new setpoints for both linear speed and angular speed. 
    if sensor_value[0] < 0.5:
        linear_vel = 0
        angular_vel = 0.25
    else:
        pass
    print("Current state error:", current_error_d, current_error_a)
    print("setpoints = linear,angular", linear_vel, angular_vel)
    print('\n')
    """ if an obstacle is found then pid out put is supressed and new setpoints are sent in for obstacle avoidance"""
    if current_front_small < 0.65 :
	return 0.2, 0.8
    else:
        return linear_vel, angular_vel   


def callback(msg):
    ##print("Front:", msg.ranges[0])
    ##print("left:", msg.ranges[179])
    ##print("Right:", msg.ranges[539])
    linear_vel, angular_vel = pid(msg.ranges)
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
