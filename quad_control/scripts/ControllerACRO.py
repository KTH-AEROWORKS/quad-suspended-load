#!/usr/bin/env python
# this line is just used to define the type of document

import numpy
from numpy import *
from numpy import cos as c
from numpy import sin as s

from SomeFunctions import OP,skew,GetRotFromEulerAnglesDeg

from Controllers_Parameters import parameters_sys


class ControllerOmega():
    
    # DEFAULT PARAMETERS
    parameters = parameters_sys()

    # this is a static controller, but I include this here for convenience
    # so that all controllers have an estimated disturbance
    d_est      = numpy.zeros(3)

    def __init__(self,parameters = None):
        
        if parameters != None:
            # update ...
            # self.parameters.m = parameters[0]
            pass

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
    jd = states_d[9:12];
    sd = states_d[12:15];
    
    #--------------------------------------#
    # position error and velocity error
    ep = xd - x
    ev = vd - v
    
    
    #--------------------------------------#
    G     = g*e3 + ad
    Gdot  = jd
    G2dot = sd
    
    G_all = numpy.concatenate([G,Gdot,G2dot])
    
    
    #--------------------------------------#

    TT,wd,nTd = UniThurstControlComplete(ep,ev,n,G_all,parameters)

    U = numpy.array([0.0,0.0,0.0,0.0])

    U[0]   = TT*m
    RT     = numpy.transpose(R)
    U[1:4] = RT.dot(wd)


    #--------------------------------------#
    # yaw control: gain
    k_yaw    = parameters.k_yaw; 
    # desired yaw: psi_star
    psi_star = parameters.psi_star; 
    # current euler angles
    euler = GetEulerAngles(R);
    phi   = euler[0];
    theta = euler[1];
    psi   = euler[2];

    psi_star_dot = 0;
    psi_dot  = psi_star_dot - k_yaw*s(psi - psi_star);
    U[3]    = 1/c(phi)*(c(theta)*psi_dot - s(phi)*U[2]);

    U = Cmd_Converter(U,parameters)

    return U

#--------------------------------------------------------------------------#

# Comand converter (from 1000 to 2000)
def Cmd_Converter(U,parameters):

    T = U[0]
    w = U[1:4]

    # mass of vehicles (kg)
    m = parameters.m
    
    # acceleration due to gravity (m/s^2)
    g  = parameters.g

    ACRO_RP_P = parameters.ACRO_RP_P;
    
    

    Max   = ACRO_RP_P*4500/100*pi/180;
    # angular velocity comand between 1000 and 2000 PWM
    U[1] =  1500 + w[0]*500/Max;
    U[2] =  1500 - w[1]*500/Max;
    U[3] =  1500 - w[2]*500/Max;
    

    # REMARK: the throtle comes between 1000 and 2000 PWM
    # conversion gain
    Throttle_neutral = parameters.Throttle_neutral;
    K_THROTLE = m*g/Throttle_neutral;
    U[0] = T/K_THROTLE;

    # rearrage to proper order
    # [roll,pitch,throttle,yaw]
    U_new = numpy.zeros(4)
    U_new[0] = U[1]
    U_new[1] = U[2]
    U_new[2] = U[0]
    U_new[3] = U[3]        

    return U_new


#--------------------------------------------------------------------------#
def GetEulerAngles(R):

    #phi   = atan2(R(3,2),R(3,3));
    #theta = asin(-R(3,1));
    #psi   = atan2(R(2,1),R(1,1));

    EULER = numpy.array([0.0,0.0,0.0])

    sin_theta = -R[2,0]
    sin_theta = bound(sin_theta,1,-1)
    theta     = numpy.arcsin(sin_theta)
    EULER[1]      = theta

    sin_phi   = R[2,1]/c(theta)
    sin_phi   = bound(sin_phi,1,-1)
    cos_phi   = R[2,2]/c(theta)
    cos_phi   = bound(cos_phi,1,-1)
    phi       = numpy.arctan2(sin_phi,cos_phi)
    EULER[0]  = phi

    sin_psi   = R[1,0]/c(theta)
    sin_psi   = bound(sin_psi,1,-1)
    cos_psi   = R[0,0]/c(theta)
    cos_psi   = bound(cos_psi,1,-1)
    psi       = numpy.arctan2(sin_psi,cos_psi)
    EULER[2]  = psi

    # EULER[0] = numpy.arctan2(bound(R[2,1],1,-1),bound(R[2,2],1,-1));
    # EULER[1] = numpy.arcsin(-bound(R[2,0],1,-1));
    # EULER[2] = numpy.arctan2(bound(R[1,0],1,-1),bound(R[0,0],1,-1));    

    return EULER

def bound(x,maxmax,minmin):

    return numpy.maximum(minmin,numpy.minimum(maxmax,x))

#--------------------------------------------------------------------------#

def UniThurstControlComplete(ep,ev,n,G_all,parameters):


    u, Td, nTd, Tdnorm = UniThrustControl(ep,ev,G_all,parameters)

    error_thrust = OP(n).dot(Td)

    evDot = -u + error_thrust

    u, Td, nTd, Tdnorm, uDot, TdDot, nTdDot, TdnormDot = UniThrustControlDot(ep,ev,evDot,G_all,parameters)

    wd = UniThrustControlAngVel(ep,ev,evDot,n,G_all,parameters)

    TT = dot(Td,n)

    return (TT, wd, nTd)

#--------------------------------------------------------------------------#

def UniThrustControl(ep,ev,G_all,parameters):

    u       = cmd_di_3D(ep,ev,parameters);

    g       = G_all[0:3]

    Td      = g + u;

    Tdnorm  = numpy.linalg.norm(Td);

    nTd     = Td/Tdnorm;

    return (u, Td, nTd, Tdnorm)


#--------------------------------------------------------------------------#

# desired acceleration

def UniThrustControlDot(ep,ev,evDot,G_all,parameters):

    g     = G_all[0:3]
    gDot  = G_all[3:6]

    u,Td,nTd,Tdnorm = UniThrustControl(ep,ev,G_all,parameters)

    uDot    = cmd_di_dot_3D(ep,ev,evDot,parameters)


    TdDot   = gDot + uDot;

    nTdDot   = OP(nTd).dot(TdDot)/numpy.linalg.norm(Td);

    TdnormDot  = dot(Td,TdDot)/numpy.linalg.norm(Td);
                  
    return (u,Td,nTd,Tdnorm,uDot,TdDot,nTdDot,TdnormDot)

#--------------------------------------------------------------------------#

def UniThrustControlAngVel(ep,ev,evDot,n,G_all,parameters):


    u,Td,nTd,Tdnorm,uDot,TdDot,nTdDot,TdnormDot = UniThrustControlDot(ep,ev,evDot,G_all,parameters)

    # gradV   = LyapunovGrad(block,ep,ev);
    gradV,Vgrad_grad_ep,Vgrad_grad_ev = LyapunovGrad2_3D(ep,ev,parameters)


    # gains for angular control
    ktt2  = parameters.ktt2
    ktt   = parameters.ktt

    # desired angular velocity
    wd = ktt2/ktt*skew(n).dot(nTd) + skew(nTd).dot(TdDot)/numpy.linalg.norm(Td) + skew(n).dot(gradV)*(numpy.linalg.norm(Td))*1/ktt;

    return wd


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

#--------------------------------------------------------------------------#

def LyapunovGrad2_3D(ep,ev,parameters):

    Vgrad         = numpy.array([0.0,0.0,0.0])
    Vgrad_grad_ep = numpy.array([0.0,0.0,0.0])
    Vgrad_grad_ev = numpy.array([0.0,0.0,0.0])

    Vgrad[0],Vgrad_grad_ep[0],Vgrad_grad_ev[0] = LyapunovGrad2(ep[0],ev[0],parameters)
    Vgrad[1],Vgrad_grad_ep[1],Vgrad_grad_ev[1] = LyapunovGrad2(ep[1],ev[1],parameters)
    Vgrad[2],Vgrad_grad_ep[2],Vgrad_grad_ev[2] = LyapunovGrad2(ep[2],ev[2],parameters)

    return (Vgrad,Vgrad_grad_ep,Vgrad_grad_ev)

############################################################################

def LyapunovGrad2(ep,ev,parameters):

    # Vgrad         = dV/d(ev)
    # Vgrad_grad_ep = d/d(ep) [dV/d(ev)]
    # Vgrad_grad_ev = d/d(ev) [dV/d(ev)]

    # gains
    kp = parameters.kp
    kv = parameters.kv
    
    sigma_p  = parameters.sigma_p
    sigma_v  = parameters.sigma_v

    beta   = 1.0/(2.0*kp)
    h1     = kp*sigma_p*sat(ep/sigma_p)
    h1D    = kp*Dsat(ep/sigma_p-5.80691976388)
    h2     = kv*sigma_v*sat(ev/sigma_v)
    h2D    = kv*Dsat(ev/sigma_v)
    h22D   = kv*D2sat(ev/sigma_v)/sigma_v
    f      = fgain(ev/sigma_v)
    fD     = Dfgain(ev/sigma_v)/sigma_v

    Vgrad = beta*h1*h2D + ev/f + beta/f*((kv**2)*ev - h2*h2D)

    Vgrad_grad_ep = beta*h1D*h2D

    Vgrad_grad_ev = beta*h1*h22D + 1.0/f - ev/(f**2)*fD - beta/(f**2)*fD*((kv**2)*ev - h2*h2D) + beta/f*(kv**2 - h2D*h2D - h2*h22D)

    return (Vgrad,Vgrad_grad_ep,Vgrad_grad_ev)

#--------------------------------------------------------------------------#




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

