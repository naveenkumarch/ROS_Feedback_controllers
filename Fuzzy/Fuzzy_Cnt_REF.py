#!/usr/bin/env python2

""" Class definition of the fuzzy logic controller for right edge following behaviour"""
class fuzzy_controller_RE():
        def __init__(self):
	    """ Centriod values for the output labels of both outputs linear speed and angular speed"""
        """ For all ouput lables I choose equilateral triangle shapes so the mid point is my centroid"""
 	    self.Turn_right_cent = -0.8
	    self.Correction_cent = -0.05
	    self.Turn_left_cent  = 0.4
	    self.linear_slow_cent = 0.35
	    self.linear_med_cent = 0.4
	    self.linear_fast_cent = 0.6
	def right_2near_MShip_val(self,min_right_distance):
            """ Calculatign the  Too Near label memebership value for the input provided. This label has a half trapezoid shape with one arm missing on left side """ 
	    if min_right_distance <= 0.50:
		Membership_value = 1.0
	    elif (min_right_distance > 0.50 and min_right_distance <= 1):
		Membership_value = (1.0-min_right_distance)/(1-0.5)
	    else:
		Membership_value = 0.0
	    return Membership_value
	def right_near_MShip_val(self,min_right_distance):
            """ Calculating the Near membership value for the input distance provide. This label has a equilateral triangle shape """
	    if min_right_distance <= 0.50:
		Membership_value = 0.0 
	    elif (min_right_distance > 0.50 and min_right_distance <= 1):
		Membership_value = (min_right_distance - 0.5)/(1.0-0.5)
	    elif min_right_distance > 1.0 and min_right_distance <= 1.5:
		Membership_value = (1.5 - min_right_distance)/(1.5 - 1.0)
	    else:
		Membership_value = 0.0
	    return Membership_value
	def right_medium_MShip_val(self,min_right_distance):
	    """ Calculating the medium membership value for the input distance provided. This label has a equilateral triangle shape """
	    if min_right_distance >= 1.0 and min_right_distance <= 1.5:
		Membership_value = (min_right_distance - 1.0)/(1.5 - 1.0)
	    elif min_right_distance >1.5 and min_right_distance <= 2:
		Membership_value = (2 - min_right_distance)/(2 - 1.5)
	    else:
		Membership_value = 0.0
	    return Membership_value
	def right_far_MShip_val(self,min_right_distance):
	    """ Calculating the far label membership value for the input distance provided. Thsi label has a half trapezoid shape with one arm missing on right side """
	    if min_right_distance >=1.5 and min_right_distance <= 2:
		Membership_value = (min_right_distance - 1.5)/(2 - 1.5)
	    elif min_right_distance > 2:
		Membership_value = 1.0
	    else:
		Membership_value = 0.0	
	    return Membership_value
	def Rule_fr_Strength(self,Rule1_Mship_value, Rule2_Mship_value):
	    """ A function to determine the firing strenght of the rule by using the min membdership function value of the labels in the rule"""
	    return min(Rule1_Mship_value, Rule2_Mship_value)


	def fuzzy_controller(self,right_front_distance,right_back_distance):
	    """ The right edge following fuzzy controller takes two inputs one right front side min distance and right back side min distance """
	    """ Calculating the membership values of all labels for both inputs """ 
	    right_fr_2near_MS_val  = self.right_2near_MShip_val(right_front_distance)
	    right_fr_near_MS_val   = self.right_near_MShip_val(right_front_distance)
	    right_fr_med_MS_val    = self.right_medium_MShip_val(right_front_distance)
	    right_fr_far_MS_val    = self.right_far_MShip_val(right_front_distance)
	    right_bck_2near_MS_val = self.right_2near_MShip_val(right_back_distance)
	    right_bck_near_MS_val  = self.right_near_MShip_val(right_back_distance)
	    right_bck_med_MS_val   = self.right_medium_MShip_val(right_back_distance)
	    right_bck_far_MS_val   = self.right_far_MShip_val(right_back_distance)
	    """ Using the above calculated membership values each rule's firing strength is calculated""" 
	    Rule1_RE_FS  = self.Rule_fr_Strength(right_fr_2near_MS_val , right_bck_2near_MS_val)
	    Rule2_RE_FS  = self.Rule_fr_Strength(right_fr_2near_MS_val , right_bck_near_MS_val)
	    Rule3_RE_FS  = self.Rule_fr_Strength(right_fr_2near_MS_val , right_bck_med_MS_val)
	    Rule4_RE_FS  = self.Rule_fr_Strength(right_fr_2near_MS_val , right_bck_far_MS_val)
	    Rule5_RE_FS  = self.Rule_fr_Strength(right_fr_near_MS_val  , right_bck_2near_MS_val)
	    Rule6_RE_FS  = self.Rule_fr_Strength(right_fr_near_MS_val  , right_bck_near_MS_val)
	    Rule7_RE_FS  = self.Rule_fr_Strength(right_fr_near_MS_val  , right_bck_med_MS_val)
	    Rule8_RE_FS  = self.Rule_fr_Strength(right_fr_near_MS_val  , right_bck_far_MS_val)
	    Rule9_RE_FS  = self.Rule_fr_Strength(right_fr_med_MS_val   , right_bck_2near_MS_val)
	    Rule10_RE_FS = self.Rule_fr_Strength(right_fr_med_MS_val   , right_bck_near_MS_val)
	    Rule11_RE_FS = self.Rule_fr_Strength(right_fr_med_MS_val   , right_bck_med_MS_val)
	    Rule12_RE_FS = self.Rule_fr_Strength(right_fr_med_MS_val   , right_bck_far_MS_val)
	    Rule13_RE_FS = self.Rule_fr_Strength(right_fr_far_MS_val   , right_bck_2near_MS_val)
	    Rule14_RE_FS = self.Rule_fr_Strength(right_fr_far_MS_val   , right_bck_near_MS_val)
	    Rule15_RE_FS = self.Rule_fr_Strength(right_fr_far_MS_val   , right_bck_med_MS_val)
	    Rule16_RE_FS = self.Rule_fr_Strength(right_fr_far_MS_val   , right_bck_far_MS_val)
	    # calculating the overall rule base firing strength 
	    Overall_rules_FS = 	Rule1_RE_FS+Rule2_RE_FS+Rule3_RE_FS+Rule4_RE_FS+Rule5_RE_FS+Rule6_RE_FS+Rule7_RE_FS+Rule8_RE_FS+Rule9_RE_FS+Rule10_RE_FS+Rule11_RE_FS+Rule12_RE_FS+Rule13_RE_FS+Rule14_RE_FS+Rule15_RE_FS+Rule16_RE_FS
	    # individual firing strengths for all the angualr velocity output labels are calulated using the above defined centriod values  
	    Turn_left_FS  = (self.Turn_left_cent*Rule1_RE_FS)+(self.Turn_left_cent*Rule2_RE_FS)+(self.Turn_left_cent*Rule4_RE_FS)+(self.Turn_left_cent*Rule3_RE_FS)+(self.Turn_left_cent*Rule7_RE_FS)
	    Correction_FS = (self.Correction_cent*Rule6_RE_FS)+(self.Correction_cent*Rule9_RE_FS)+(self.Correction_cent*Rule10_RE_FS)+(self.Correction_cent*Rule5_RE_FS)
	    Turn_right_FS = (self.Turn_right_cent*Rule13_RE_FS)+(self.Turn_right_cent*Rule14_RE_FS)+(self.Turn_right_cent*Rule15_RE_FS)+(self.Turn_right_cent*Rule16_RE_FS)+(self.Turn_right_cent*Rule11_RE_FS)+(self.Turn_right_cent*Rule12_RE_FS)+(self.Turn_right_cent*Rule9_RE_FS)+(self.Turn_right_cent*Rule10_RE_FS)+(self.Turn_right_cent*Rule5_RE_FS)+(self.Turn_right_cent*Rule8_RE_FS)
	    # individual firing strengths for all the lnear velocity output labels are calulated using the above defined centriod values
	    linear_slow_FS= (self.linear_slow_cent*Rule1_RE_FS)+(self.linear_slow_cent*Rule2_RE_FS)+(self.linear_slow_cent*Rule3_RE_FS)+(self.linear_slow_cent*Rule4_RE_FS)+(self.linear_slow_cent*Rule5_RE_FS)+(self.linear_slow_cent*Rule6_RE_FS)+(self.linear_slow_cent*Rule9_RE_FS)+(self.linear_slow_cent*Rule10_RE_FS)+(self.linear_slow_cent*Rule11_RE_FS)+(self.linear_slow_cent*Rule13_RE_FS)+(self.linear_slow_cent*Rule14_RE_FS)+(self.linear_slow_cent*Rule15_RE_FS)
	    linear_med_FS = (self.linear_med_cent*Rule7_RE_FS)+(self.linear_med_cent*Rule8_RE_FS)
	    linear_fast_FS= (self.linear_fast_cent*Rule12_RE_FS)+(self.linear_fast_cent*Rule16_RE_FS)
	    #Defuzzified angular velocity is calculated from the above calculated values 
	    angular_velocity = (Turn_left_FS+Correction_FS+Turn_right_FS)/Overall_rules_FS
	    print("Linear REF FS", linear_slow_FS,linear_med_FS,linear_fast_FS)
	    #Defuzzified linear velocity is calculated from the above calcualted values
	    linear_velocity  = (linear_slow_FS+linear_med_FS+linear_fast_FS)/Overall_rules_FS
	    print("Angular Velocity being sent by RE is ", angular_velocity)
	    print("Linear Velocity being sent by RE is  ", linear_velocity)
	    return linear_velocity, angular_velocity 
