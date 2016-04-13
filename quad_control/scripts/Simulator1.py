#!/usr/bin/env python
# this is just to define file type

import numpy

from Simulator_Parameters import parameters_sys

# we need some functions
from SomeFunctions import skew,GetEulerAngles,GetRotFromEulerAngles

# we use pi
import math

def sys_dynamics(states , U , parameters):
    
    # U = Full actuation vehicles

    # acceleration due to gravity (m/s^2)
    g  = parameters.g

    # mass of vehicles (kg)
    m = parameters.m

    # third canonical basis vector
    e3 = numpy.array([0.0,0.0,1.0])

    # states

    # transported mass: position and velocity
    x = states[0:3];
    v = states[3:6];

    # thrust unit vector
    R  = states[6:15];
    R  = numpy.reshape(R,(3,3))
    n  = R.dot(e3)


    # ----------------------------------------------#
    # ----------------------------------------------#
    # rearrage to proper order
    # [roll,pitch,throttle,yaw] in sticks
    # in model order is [throttle,roll,pitch,yaw]
    U_new = numpy.zeros(4)
    U_new[0] = U[2]
    U_new[1] = U[0]
    U_new[2] = U[1]
    U_new[3] = U[3]        

    U = U_new

    # ----------------------------------------------#
    # This model would require more work
    # U[0:1] : as desired full actuation
    # Td      = U[0:3];
    # Tddot   = .... some derivator
    # Tdnorm  = numpy.linalg.norm(Td);
    # nTd     = Td/Tdnorm;
    # w_feedforward = skew(nTd).dot(TdDot)/numpy.linalg.norm(Td)
    # w = ktt*skew(n).dot(nTd) + wd_feedforward
    # ----------------------------------------------#

    # current  euler angles
    ee  = GetEulerAngles(R)
    # current psi
    psi = ee[2]

    # degrees per second
    MAX_PSI_SPEED_Deg = parameters.MAX_PSI_SPEED_Deg
    MAX_PSI_SPEED_Rad = MAX_PSI_SPEED_Deg*math.pi/180.0

    MAX_ANGLE_DEG = parameters.MAX_ANGLE_DEG
    MAX_ANGLE_RAD = MAX_ANGLE_DEG*math.pi/180.0

    # desired roll and pitch
    # U[1:3] : desired roll and pitch
    roll_des  =  (U[1] - 1500.0)*MAX_ANGLE_RAD/500.0;
    
    # ATTENTTION TO PITCH: WHEN STICK IS FORWARD PITCH,
    # pwm GOES TO 1000, AND PITCH IS POSITIVE 
    pitch_des = -(U[2] - 1500)*MAX_ANGLE_RAD/500.0;
    # desired euler angles
    ee_des = numpy.array([roll_des,pitch_des,psi])

    # gain of inner loop for attitude control
    ktt = parameters.ktt_inner_loop

    nTd     = GetRotFromEulerAngles(ee_des).dot(e3);
    w       = ktt*skew(n).dot(nTd)
    RT      = numpy.transpose(R)
    w       = RT.dot(w)

    # U[3]: yaw angular speed
    w[2]    = -(U[3] - 1500.0)*MAX_PSI_SPEED_Rad/500.0

    # Throttle neutral
    Throttle_neutral = parameters.Throttle_neutral
    K_Throttle       = m*g/Throttle_neutral

    # -----------------------------------------------------------------------------#
    # STABILIZE MODE:APM COPTER
    # The throttle sent to the motors is automatically adjusted based on the tilt angle
    # of the vehicle (i.e increased as the vehicle tilts over more) to reduce the 
    # compensation the pilot must fo as the vehicles attitude changes
    # -----------------------------------------------------------------------------#
    # Throttle = U[0]
    # this increases the actual throtle
    Throttle = U[0]/numpy.dot(n,e3)

    pDot = v
    # acceleration of quad
    vDot = K_Throttle*(Throttle*n)/m - g*e3;

    RDot = R.dot(skew(w))
    RDot = numpy.reshape(RDot,9)

    # collecting derivatives
    derivatives = numpy.concatenate([pDot,vDot,RDot])
      
    return derivatives



class AttitudeInnerLoop(object):

    # some necessary attributes
    parameters = parameters_sys()
    
    def __init__(self,parameters = None):
        
        # if parameters == None:
        # stick with the initialized parameter

        if parameters != None:
            # update mass and throttle neutral
            self.parameters.m = parameters[0]
            self.parameters.Throttle_neutral = parameters[1]
            self.ktt_inner_loop = parameters[2]

    def update_parameters(self,parameters):
        self.parameters = parameters

    def output(self,t, y, U):
        # states = y
        # return sys_dynamics(states,states_d,self.parameters)
        UU = numpy.array([U.U0,U.U1,U.U2,U.U3])        
        return sys_dynamics(y,UU,self.parameters)

#--------------------------------------------------------------------------------------------#
#--------------------------------------------------------------------------------------------#
