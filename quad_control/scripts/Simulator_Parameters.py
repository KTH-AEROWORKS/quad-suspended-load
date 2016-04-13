#!/usr/bin/env python
# this is just to define file type

import rospy
import numpy


# parameters_system
class parameters_sys(object):
    
    # acceleration due to gravity (m/s^2)
    # g = 9.81
    # This parameter is defined in the Launch file
    g = rospy.get_param("gravity_sim",9.81)
    
    # mass of vehicles (kg)
    m = rospy.get_param("mass_quad_sim",1.442)

    # throttle that cancels weight
    Throttle_neutral = rospy.get_param("Throttle_neutral_sim",1484.0)

    # ACRO mode (angular velocity sensitivity)
    ACRO_RP_P = rospy.get_param("ACRO_RP_P_sim",4.5)

    #--------------------------------------------------------------#
    # Simulator 1
    MAX_ANGLE_DEG     = rospy.get_param("MAX_ANGLE_DEG",45.0)
    ktt_inner_loop    = rospy.get_param("ktt_inner_loop",10.0)

    # The default of 4.5 commands a 200 deg/sec rate
    # of rotation when the yaw stick is held fully left or right.
    MAX_PSI_SPEED_Deg = rospy.get_param("MAX_PSI_SPEED_Deg",200.0)

    #--------------------------------------------------------------#

    # # The class "constructor" - It's actually an initializer
    # def __init__(self,parameters):
    #     self.m = 1.1