#!/usr/bin/env python
# this line is just used to define the type of document

# useful for rospy.logwar()
import rospy

import numpy

from SomeFunctions import Rz,GetEulerAngles,GetRotFromEulerAnglesDeg

from Controllers_Parameters import parameters_sys

from numpy import cos as c
from numpy import sin as s

# we use pi
import math

class ControllerPD():
    
    parameters = parameters_sys()

    # this is a static controller, but I include this here for convenience
    # so that all controllers have an estimated disturbance
    d_est      = numpy.zeros(3)


    def __init__(self,parameters = None):

        if parameters != None:
            # update mass and throttle neutral
            self.parameters.kp   = parameters[0]
            self.parameters.kv   = parameters[1]
            # self.parameters.kp_z = parameters[2]
            # self.parameters.kv_z = parameters[3]
            self.parameters.Throttle_neutral = parameters[4]        

    # def reset_estimate_xy(self):
    #     return

    # def reset_estimate_z(self):
    #     return   

    def update_parameters(self,parameters):
        self.parameters = parameters

    def output(self,t,states,states_d):
        # convert euler angles from deg to rotation matrix
        ee = states[6:9]
        R  = GetRotFromEulerAnglesDeg(ee)
        R  = numpy.reshape(R,9)
        # collecting states
        states  = numpy.concatenate([states[0:6],R])
        return controller(states,states_d,self.parameters)



# Controller
def controller(states,states_d,parameters):
    
    # mass of vehicles (kg)
    m = parameters.m
    
    # acceleration due to gravity (m/s^2)
    g  = parameters.g

    # Angular velocities multiplication factor
    # Dw  = parameters.Dw
    
    # third canonical basis vector
    e3 = numpy.array([0.0,0.0,1.0])
    
    #--------------------------------------#
    # transported mass: position and velocity
    x  = states[0:3];
    v  = states[3:6];
    # thrust unit vector and its angular velocity
    R  = states[6:15];
    R  = numpy.reshape(R,(3,3))
    n  = R.dot(e3)

    # print(GetEulerAngles(R)*180/3.14)

    
    #--------------------------------------#
    # desired quad trajectory
    xd = states_d[0:3];
    vd = states_d[3:6];
    ad = states_d[6:9];
    
    #--------------------------------------#
    # position error and velocity error
    ep = xd - x
    ev = vd - v
    
    
    u = cmd_di_3D(ep,ev,parameters)

    # -----------------------------------------------------------------------------#
    # vector of commnads to be sent by the controller
    U = numpy.zeros(4)

    # -----------------------------------------------------------------------------#
    # STABILIZE MODE:APM COPTER
    # The throttle sent to the motors is automatically adjusted based on the tilt angle
    # of the vehicle (i.e increased as the vehicle tilts over more) to reduce the 
    # compensation the pilot must fo as the vehicles attitude changes
    # -----------------------------------------------------------------------------#
    Full_actuation = m*(ad + u + g*e3)

    Throttle = numpy.dot(Full_actuation,n)
    # this decreases the throtle, which will be increased
    Throttle = Throttle*numpy.dot(n,e3)
    U[0]     = Throttle

    #--------------------------------------#
    # current  euler angles
    euler  = GetEulerAngles(R)
    # current phi and theta
    phi   = euler[0];
    theta = euler[1];
    # current psi
    psi   = euler[2];


    #--------------------------------------#
    # Rz(psi)*Ry(theta_des)*Rx(phi_des) = n_des

    # desired roll and pitch angles
    n_des     = Full_actuation/numpy.linalg.norm(Full_actuation)
    n_des_rot = Rz(-psi).dot(n_des)


    sin_phi   = -n_des_rot[1]
    sin_phi   = bound(sin_phi,1,-1)
    phi       = numpy.arcsin(sin_phi)
    U[1]      = phi

    sin_theta = n_des_rot[0]/c(phi)
    sin_theta = bound(sin_theta,1,-1)
    cos_theta = n_des_rot[2]/c(phi)
    cos_theta = bound(cos_theta,1,-1)
    pitch     = numpy.arctan2(sin_theta,cos_theta)
    U[2]      = pitch

    #--------------------------------------#
    # yaw control: gain
    k_yaw    = parameters.k_yaw; 
    # desired yaw: psi_star
    psi_star = parameters.psi_star; 

    psi_star_dot = 0;
    psi_dot      = psi_star_dot - k_yaw*s(psi - psi_star);
    U[3]         = 1/c(phi)*(c(theta)*psi_dot - s(phi)*U[2]);

    U = Cmd_Converter(U,parameters)

    return U

#--------------------------------------------------------------------------#

def bound(x,maxmax,minmin):

    return numpy.maximum(minmin,numpy.minimum(maxmax,x))

#--------------------------------------------------------------------------#


# Comand converter (from 1000 to 2000)
def Cmd_Converter(U,parameters):

    # mass of vehicles (kg)
    m = parameters.m
    
    # acceleration due to gravity (m/s^2)
    g  = parameters.g

    # degrees per second
    MAX_PSI_SPEED_Deg = parameters.MAX_PSI_SPEED_Deg
    MAX_PSI_SPEED_Rad = MAX_PSI_SPEED_Deg*math.pi/180.0

    MAX_ANGLE_DEG = parameters.MAX_ANGLE_DEG
    MAX_ANGLE_RAD = MAX_ANGLE_DEG*math.pi/180.0

    # angles comand between 1000 and 2000 PWM

    # desired roll and pitch
    # U[1:3] : desired roll and pitch
    # Roll
    U[1]  =  1500.0 + U[1]*500.0/MAX_ANGLE_RAD;    
    # Pitch: 

    # ATTENTTION TO PITCH: 
    # when stick is 1000, it produces POSITIVE pitch
    U[2]  =  1500.0 - U[2]*500.0/MAX_ANGLE_RAD;    

    # psi angular speed 
    # ATTENTTION TO YAW RATE:
    # when stick is 2000, it produces NEGATIVE yaw rate    
    U[3]  =  1500.0 - U[3]*500.0/MAX_PSI_SPEED_Rad;    

    Throttle = U[0]
    # REMARK: the throtle comes between 1000 and 2000 PWM
    # conversion gain
    Throttle_neutral = parameters.Throttle_neutral;
    K_THROTLE = m*g/Throttle_neutral;
    U[0] = Throttle/K_THROTLE;

    U_new = numpy.zeros(4)
    U_new[0] = U[1]
    U_new[1] = U[2]
    U_new[2] = U[0]
    U_new[3] = U[3]  

    return U_new


#--------------------------------------------------------------------------#
# FOR DOUBLE INTEGRATOR

def cmd_di_3D(ep,ev,parameters):

    u    = numpy.array([0.0,0.0,0.0])
    u[0] = cmd_di(ep[0],ev[0],parameters)
    u[1] = cmd_di(ep[1],ev[1],parameters)
    u[2] = cmd_di(ep[2],ev[2],parameters)
    
    return u
############################################################################

def cmd_di(ep,ev,parameters):
    # command for double integrator

    # gains
    kp = parameters.kp;
    kv = parameters.kv;

    sigma_p  = parameters.sigma_p;
    sigma_v  = parameters.sigma_v;


    h1  = kp*sigma_p*sat(ep/sigma_p);
    h2  = kv*sigma_v*sat(ev/sigma_v);
    f   = fgain(ev/sigma_v);

    u = f*h1 + h2;

    return u

############################################################################

def cmd_di_dot_3D(ep,ev,evD,parameters):

    udot    = numpy.array([0.0,0.0,0.0])
    udot[0] = cmd_di_dot(ep[0],ev[0],evD[0],parameters)
    udot[1] = cmd_di_dot(ep[1],ev[1],evD[1],parameters)
    udot[2] = cmd_di_dot(ep[2],ev[2],evD[2],parameters)

    return udot

############################################################################

def cmd_di_dot(ep,ev,evD,parameters):
    # command for double integrator

    # gains
    kp = parameters.kp
    kv = parameters.kv

    sigma_p  = parameters.sigma_p
    sigma_v  = parameters.sigma_v

    h1  = kp*sigma_p*sat(ep/sigma_p)
    h2  = kv*sigma_v*sat(ev/sigma_v)
    f   = fgain(ev/sigma_v)

    h1Dot = kp*Dsat(ep/sigma_p)*ev
    h2Dot = kv*Dsat(ev/sigma_v)*evD
    fDot  = Dfgain(ev/sigma_v)*evD/sigma_v

    udot  = fDot*h1 + f*h1Dot + h2Dot

    return udot

############################################################################

def cmd_di_2dot_3D(ep,ev,evD,ev2D,parameters):

    u2dot    = numpy.array([0.0,0.0,0.0])
    u2dot[0] = cmd_di_2dot(ep[0],ev[0],evD[0],ev2D[0],parameters)
    u2dot[1] = cmd_di_2dot(ep[1],ev[1],evD[1],ev2D[1],parameters)
    u2dot[2] = cmd_di_2dot(ep[2],ev[2],evD[2],ev2D[2],parameters)

    return u2dot

############################################################################

def cmd_di_2dot(ep,ev,evD,ev2D,parameters):
    # command for double integrator

    # gains
    kp = parameters.kp
    kv = parameters.kv
    
    sigma_p  = parameters.sigma_p
    sigma_v  = parameters.sigma_v

    h1  = kp*sigma_p*sat(ep/sigma_p);
    h2  = kv*sigma_v*sat(ev/sigma_v);
    f   = fgain(ev/sigma_v);

    h1Dot = kp*Dsat(ep/sigma_p)*ev;
    h2Dot = kv*Dsat(ev/sigma_v)*evD;
    fDot  = Dfgain(ev/sigma_v)*evD/sigma_v;

    h12Dot = kp*D2sat(ep/sigma_p)*ev/sigma_p*ev + kp*Dsat(ep/sigma_p)*evD;
    h22Dot = kv*D2sat(ev/sigma_v)*evD/sigma_v*evD + kv*Dsat(ev/sigma_v)*ev2D;
    f2Dot  = D2fgain(ev/sigma_v)*evD/sigma_v*evD/sigma_v + Dfgain(ev/sigma_v)*ev2D/sigma_v;

    u2dot  = f2Dot*h1 + fDot*h1Dot + fDot*h1Dot + f*h12Dot + h22Dot;

    return u2dot


def sat(x):

    return x/numpy.sqrt(1 + x**2.0)


def Dsat(x):

    return 1.0/numpy.power(1 + x**2.0,1.5)


def D2sat(x):

    return -3.0*x/numpy.power(1.0+x**2.0, 2.5)


def sat_Int(x):
    # primitive of saturation function

    return numpy.sqrt(1.0 + x**2.0) - 1.0;



def fgain(x):

    return 1.0/numpy.sqrt(1.0 + x**2.0)


def Dfgain(x):

    return -x/numpy.power(1.0 + x**2.0, 1.5);


def D2fgain(x):

    return (-1.0 + 2.0*x**2.0)/numpy.power(1.0 + x**2.0, 2.5);


def fgain_int(x):

    # integral of x/fgain(x) from 0 to in

    return 1.0/3.0*(-1.0 + numpy.power(1.0 + x**2.0, 1.5));

def fgain_int2(x):

    # integral of sat(x)*Dsat(x)/fgain(x) from 0 to x

    return 1.0 - 1.0/sqrt(1.0 + x**2.0);

