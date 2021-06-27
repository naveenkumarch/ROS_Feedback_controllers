#!/usr/bin/env python2

from __future__ import division 
""" Class definition of the fuzzy logic controller for Obstacle avoidance following behaviour"""
class fuzzy_controller_OA():
	def __init__ (self):
	    """ Centriod values for the output labels of both outputs linear speed and angular speed"""
        """ For all ouput lables I choose equilateral triangle shapes so the mid point is my centroid"""
        self.Turn_right_cent = -0.9
	    self.Correction_cent = 0
	    self.Turn_left_cent  = 0.9
	    self.linear_slow_cent = 0.35
	    self.linear_med_cent = 0.45
	    self.linear_fast_cent = 0.55
	def front_near_MShip_val(self,min_distance):
	    """ Calculatign the Near label memebership value for the input provided. This label has a half trapezoid shape with one arm missing on left side """ 
	    if min_distance <= 0.5:
		Membership_value = 1.0
	    elif (min_distance > 0.5 and min_distance <= 1):
		Membership_value = (1.0-min_distance)/(0.5)
	    else:
		Membership_value = 0.0
	    return Membership_value
	def front_Med_MShip_val(self,min_distance):
	    """ Calculating the Med membership value for the input distance provide. This label has a equilateral triangle shape """
	    if min_distance <= 0.5:
		Membership_value = 0.0 
	    elif (min_distance > 0.5 and min_distance <= 1.0):
		Membership_value = (min_distance - 0.5)/(0.5)
	    elif min_distance >1.0 and min_distance <= 1.50:
		Membership_value = (1.5 - min_distance)/(0.5)
	    else:
		Membership_value = 0.0
	    return Membership_value
	def front_far_MShip_val(self,min_distance):
	    """ Calculating the far label membership value for the input distance provided. Thsi label has a half trapezoid shape with one arm missing on right side """
	    if min_distance >=1.0 and min_distance <= 1.50:
		Membership_value = (min_distance - 1.0)/(0.5)
	    elif min_distance > 1.50:
		Membership_value = 1.0
	    else:
		Membership_value = 0.0	
	    return Membership_value
	def Rule_fr_Strength(self,Rule1_Mship_value, Rule2_Mship_value, Rule3_Mship_value):
	    """ A function to determine the firing strenght of the rule by using the min membdership function value of the labels in the rule"""
	    return min(Rule1_Mship_value, Rule2_Mship_value, Rule3_Mship_value)



	def fuzzy_controller_FO(self,front_left_distance, front_center_distance, front_right_distance):
	    """ The front Obstacle avoidance fuzzy controller takes three inputs one front right side min distance , front center min distance and front left side min distance right back side min distance """
	    """ Calculating the membership values of all labels for three inputs """
	    front_left_near_MS_val   = self.front_near_MShip_val(front_left_distance)
	    front_left_med_MS_val    = self.front_Med_MShip_val(front_left_distance)
	    front_left_far_MS_val    = self.front_far_MShip_val(front_left_distance)
	    front_center_near_MS_val = self.front_near_MShip_val(front_center_distance)
	    front_center_med_MS_val  = self.front_Med_MShip_val(front_center_distance)
	    front_center_far_MS_val  = self.front_far_MShip_val(front_center_distance)
	    front_right_near_MS_val  = self.front_near_MShip_val(front_right_distance)
	    front_right_med_MS_val   = self.front_Med_MShip_val(front_right_distance)
	    front_right_far_MS_val   = self.front_far_MShip_val(front_right_distance)
	    """ Using the above calculated membership values each rule's firing strength is calculated"""
	    Rule1_FO_FS  = self.Rule_fr_Strength(front_right_near_MS_val  , front_center_near_MS_val , front_left_near_MS_val)
	    Rule2_FO_FS  = self.Rule_fr_Strength(front_right_near_MS_val  , front_center_near_MS_val , front_left_med_MS_val) 
	    Rule3_FO_FS  = self.Rule_fr_Strength(front_right_near_MS_val  , front_center_near_MS_val , front_left_far_MS_val)
	    Rule4_FO_FS  = self.Rule_fr_Strength(front_right_near_MS_val  , front_center_med_MS_val  , front_left_near_MS_val)
	    Rule5_FO_FS  = self.Rule_fr_Strength(front_right_near_MS_val  , front_center_med_MS_val  , front_left_med_MS_val)
	    Rule6_FO_FS  = self.Rule_fr_Strength(front_right_near_MS_val  , front_center_med_MS_val  , front_left_far_MS_val)
	    Rule7_FO_FS  = self.Rule_fr_Strength(front_right_near_MS_val  , front_center_far_MS_val  , front_left_near_MS_val)
	    Rule8_FO_FS  = self.Rule_fr_Strength(front_right_near_MS_val  , front_center_far_MS_val  , front_left_med_MS_val)
	    Rule9_FO_FS  = self.Rule_fr_Strength(front_right_near_MS_val  , front_center_far_MS_val  , front_left_far_MS_val)
	    Rule10_FO_FS  = self.Rule_fr_Strength(front_right_med_MS_val   , front_center_near_MS_val , front_left_near_MS_val)
	    Rule11_FO_FS  = self.Rule_fr_Strength(front_right_med_MS_val   , front_center_near_MS_val , front_left_med_MS_val)
	    Rule12_FO_FS  = self.Rule_fr_Strength(front_right_med_MS_val   , front_center_near_MS_val , front_left_far_MS_val)
	    Rule13_FO_FS  = self.Rule_fr_Strength(front_right_med_MS_val   , front_center_med_MS_val  , front_left_near_MS_val)
	    Rule14_FO_FS  = self.Rule_fr_Strength(front_right_med_MS_val   , front_center_med_MS_val  , front_left_med_MS_val)
	    Rule15_FO_FS  = self.Rule_fr_Strength(front_right_med_MS_val   , front_center_med_MS_val  , front_left_far_MS_val)
	    Rule16_FO_FS  = self.Rule_fr_Strength(front_right_med_MS_val   , front_center_far_MS_val  , front_left_near_MS_val)
	    Rule17_FO_FS  = self.Rule_fr_Strength(front_right_med_MS_val   , front_center_far_MS_val  , front_left_med_MS_val)
	    Rule18_FO_FS  = self.Rule_fr_Strength(front_right_med_MS_val   , front_center_far_MS_val  , front_left_far_MS_val)
	    Rule19_FO_FS  = self.Rule_fr_Strength(front_right_far_MS_val   , front_center_near_MS_val , front_left_near_MS_val)
	    Rule20_FO_FS  = self.Rule_fr_Strength(front_right_far_MS_val   , front_center_near_MS_val , front_left_med_MS_val)
	    Rule21_FO_FS  = self.Rule_fr_Strength(front_right_far_MS_val   , front_center_near_MS_val , front_left_far_MS_val)
	    Rule22_FO_FS  = self.Rule_fr_Strength(front_right_far_MS_val   , front_center_med_MS_val  , front_left_near_MS_val)
	    Rule23_FO_FS  = self.Rule_fr_Strength(front_right_far_MS_val   , front_center_med_MS_val  , front_left_med_MS_val)
	    Rule24_FO_FS  = self.Rule_fr_Strength(front_right_far_MS_val   , front_center_med_MS_val  , front_left_far_MS_val)
	    Rule25_FO_FS  = self.Rule_fr_Strength(front_right_far_MS_val   , front_center_far_MS_val  , front_left_near_MS_val)
	    Rule26_FO_FS  = self.Rule_fr_Strength(front_right_far_MS_val   , front_center_far_MS_val  , front_left_med_MS_val)
	    Rule27_FO_FS  = self.Rule_fr_Strength(front_right_far_MS_val   , front_center_far_MS_val  , front_left_far_MS_val)
	    # calculating the overall rule base firing strength
	    Overall_rules_FS = 	Rule1_FO_FS+Rule2_FO_FS+Rule3_FO_FS+Rule4_FO_FS+Rule5_FO_FS+Rule6_FO_FS+Rule7_FO_FS+Rule8_FO_FS+Rule9_FO_FS+Rule10_FO_FS+Rule11_FO_FS+Rule12_FO_FS+Rule13_FO_FS+Rule14_FO_FS+Rule15_FO_FS+Rule16_FO_FS+Rule17_FO_FS+Rule18_FO_FS+Rule19_FO_FS+Rule20_FO_FS+Rule21_FO_FS+Rule22_FO_FS+Rule23_FO_FS+Rule24_FO_FS+Rule25_FO_FS+Rule26_FO_FS+Rule27_FO_FS
	    # individual firing strengths for all the angualr velocity output labels are calulated using the above defined centriod values 
	    Turn_left_FS  = (self.Turn_left_cent*Rule1_FO_FS)+(self.Turn_left_cent*Rule2_FO_FS)+(self.Turn_left_cent*Rule3_FO_FS)+(self.Turn_left_cent*Rule4_FO_FS)+(self.Turn_left_cent*Rule5_FO_FS)+(self.Turn_left_cent*Rule6_FO_FS)+(self.Turn_left_cent*Rule12_FO_FS)+(self.Turn_left_cent*Rule15_FO_FS)
	    Correction_FS = (self.Correction_cent*Rule7_FO_FS)+(self.Correction_cent*Rule8_FO_FS)+(self.Correction_cent*Rule9_FO_FS)+(self.Correction_cent*Rule14_FO_FS)+(self.Correction_cent*Rule16_FO_FS)+(self.Correction_cent*Rule17_FO_FS)+(self.Correction_cent*Rule18_FO_FS)+(self.Correction_cent*Rule25_FO_FS)+(self.Correction_cent*Rule26_FO_FS)+(self.Correction_cent*Rule27_FO_FS)
	    Turn_right_FS = (self.Turn_right_cent*Rule10_FO_FS)+(self.Turn_right_cent*Rule11_FO_FS)+(self.Turn_right_cent*Rule13_FO_FS)+(self.Turn_right_cent*Rule19_FO_FS)+(self.Turn_right_cent*Rule20_FO_FS)+(self.Turn_right_cent*Rule21_FO_FS)+(self.Turn_right_cent*Rule22_FO_FS)+(self.Turn_right_cent*Rule23_FO_FS)+(self.Turn_right_cent*Rule24_FO_FS)
	    # individual firing strengths for all the lnear velocity output labels are calulated using the above defined centriod values
	    linear_slow_FS= (self.linear_slow_cent*Rule1_FO_FS)+(self.linear_slow_cent*Rule2_FO_FS)+(self.linear_slow_cent*Rule3_FO_FS)+(self.linear_slow_cent*Rule4_FO_FS)+(self.linear_slow_cent*Rule5_FO_FS)+(self.linear_slow_cent*Rule6_FO_FS)+(self.linear_slow_cent*Rule7_FO_FS)+(self.linear_slow_cent*Rule8_FO_FS)+(self.linear_slow_cent*Rule9_FO_FS)+(self.linear_slow_cent*Rule10_FO_FS)+ (self.linear_slow_cent*Rule11_FO_FS)+(self.linear_slow_cent*Rule12_FO_FS)+(self.linear_slow_cent*Rule13_FO_FS)+(self.linear_slow_cent*Rule16_FO_FS)+(self.linear_slow_cent*Rule19_FO_FS)+(self.linear_slow_cent*Rule20_FO_FS)+ (self.linear_slow_cent*Rule21_FO_FS)+(self.linear_slow_cent*Rule22_FO_FS)+(self.linear_slow_cent*Rule23_FO_FS)+(self.linear_slow_cent*Rule25_FO_FS)
	    linear_med_FS = (self.linear_med_cent*Rule14_FO_FS)+(self.linear_med_cent*Rule15_FO_FS)+(self.linear_med_cent*Rule17_FO_FS)+(self.linear_med_cent*Rule18_FO_FS)+(self.linear_med_cent*Rule24_FO_FS)+(self.linear_med_cent*Rule26_FO_FS)
	    linear_fast_FS= (self.linear_fast_cent*Rule27_FO_FS)
	    #Defuzzified angular velocity is calculated from the above calculated values
	    angular_velocity = (Turn_left_FS+Correction_FS+Turn_right_FS)/Overall_rules_FS
	    #Defuzzified linear velocity is calculated from the above calcualted values
	    linear_velocity  = (linear_slow_FS+linear_med_FS+linear_fast_FS)/Overall_rules_FS
	    print("Angular Velocity being sent by OA is ", angular_velocity)
	    print("Linear Velocity being sent by OA is ", linear_velocity)
	    return linear_velocity, angular_velocity 

