#!/usr/bin/env python
# this is just to define file type

import numpy

from Simulator_Parameters import parameters_sys

# we use pi
import math

# we need some functions
from SomeFunctions import skew

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

    #------------------------------------------------#
    #------------------------------------------------#

    # rearrage to proper order
    # [roll,pitch,throttle,yaw] in sticks
    # in model order is [throttle,roll,pitch,yaw]
    U_new = numpy.zeros(4)
    U_new[0] = U[2]
    U_new[1] = U[0]
    U_new[2] = U[1]
    U_new[3] = U[3]        

    U = U_new

    #------------------------------------------------#
    #------------------------------------------------#

    T = U[0];
    w = U[1:4]


    # Throttle neutral
    Throttle_neutral = parameters.Throttle_neutral
    ACRO_RP_P = parameters.ACRO_RP_P

    K_Throttle = m*g/Throttle_neutral
    Max = ACRO_RP_P*4500/100*math.pi/180;
    w[0] =  (w[0] - 1500)/500*Max;
    w[1] = -(w[1] - 1500)/500*Max;
    w[2] = -(w[2] - 1500)/500*Max;

    pDot = v
    # acceleration of quad
    vDot = K_Throttle*(T*n)/m - g*e3;
    
    RDot = R.dot(skew(w))
    RDot = numpy.reshape(RDot,9)

    # collecting derivatives
    derivatives = numpy.concatenate([pDot,vDot,RDot])
      
    return derivatives

#--------------------------------------------------------------------------------------------#
#--------------------------------------------------------------------------------------------#


class DynamicsOmega(object):

    # some necessary attributes
    parameters = parameters_sys()
    
    def __init__(self,parameters = None):
        
        # if parameters == None:
        # stick with the initialized parameter

        if parameters != None:
            # update mass and throttle neutral
            self.parameters.m = parameters[0]
            self.parameters.Throttle_neutral = parameters[1]

    def update_parameters(self,parameters):
        self.parameters = parameters

    def output(self,t, y, U):
        # states = y
        # return sys_dynamics(states,states_d,self.parameters)
        UU = numpy.array([U.U0,U.U1,U.U2,U.U3])        
        return sys_dynamics(y,UU,self.parameters)

#--------------------------------------------------------------------------------------------#
#--------------------------------------------------------------------------------------------#
