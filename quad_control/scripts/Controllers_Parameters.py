#!/usr/bin/env python
# this is just to define file type

# we will import some values from roslaunch when launch files are used
import rospy

import numpy

class parameters_sys(object):
    
    # acceleration due to gravity (m/s^2)
    g  = rospy.get_param("gravity_ctr",9.81)
    
    # mass of vehicles (kg)
    m  = rospy.get_param("mass_quad_ctr",1.442)


    # FOr ACRO Controller
    # controller gains
    ktt  = rospy.get_param("ktt",100.0)
    ktt2 = rospy.get_param("ktt2",100.0)

    # SOME USEFUL RULES: kp = wn*wn  AND kv = 2 xsi wn    
    wn  = 1.0
    xsi = numpy.sqrt(2)/2
    kv   = rospy.get_param("kv",2.0*xsi*wn)
    kp   = rospy.get_param("kp",wn*wn)

    # sigma_p = rospy.get_param("sigma_p",0.3)
    # sigma_v = rospy.get_param("sigma_v",0.4)
    sigma_p = rospy.get_param("sigma_p",1.0)
    sigma_v = rospy.get_param("sigma_v",1.0)

    # throttle that cancels weight
    Throttle_neutral = rospy.get_param("Throttle_neutral_ctr",1430.0)

    # ACRO mode (angular velocity sensitivity)
    ACRO_RP_P = rospy.get_param("ACRO_RP_P_ctr",4.5)

    # yaw control: gain
    k_yaw    = rospy.get_param("k_yaw_ctr",3.0);
    
    # desired yaw: psi_star (RADIANS)
    psi_star = 0; 

    # ---------------------------------------------------------- #
    # CONTROLLER 0
    MAX_ANGLE_DEG     = 45.0
    # The default of 4.5 commands a 200 deg/sec rate
    # of rotation when the yaw stick is held fully left or right.
    MAX_PSI_SPEED_Deg = 200.0
    
    # ---------------------------------------------------------- #
    # ---------------------------------------------------------- #

    # CONTROLLER 1
    # SOME USEFUL RULES: kp = wn*wn  AND kv = 2 xsi wn    
    wn   = 1
    xsi  = numpy.sqrt(2)/2
    kv_C1   = rospy.get_param("kv",2.0*xsi*wn)
    kp_C1   = rospy.get_param("kp",wn*wn)
    ki_C1   = rospy.get_param("ki",2.0)

    kv_z_C1   = rospy.get_param("kv_z",2.0*xsi*wn)
    kp_z_C1   = rospy.get_param("kp_z",wn*wn)
    ki_z_C1   = rospy.get_param("ki_z",2.0)

    Max_disturbance_C1 = numpy.array([1.0,1.0,1.0])

    # ---------------------------------------------------------- #
    # ---------------------------------------------------------- #


    # The class "constructor" - It's actually an initializer
    # def __init__(self):
    #   self.M = 1.1