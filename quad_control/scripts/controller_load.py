#!/usr/bin/env python
# this line is just used to define the type of document

import numpy
from numpy import *
from numpy.linalg import *
from numpy import cos as c
from numpy import sin as s

import rospy

from SomeFunctions import Rz,GetEulerAngles,GetRotFromEulerAnglesDeg,skew,OP

from Controllers_Parameters import parameters_sys


class ControllerLoad():
    
    # DEFAULT PARAMETERS
    parameters = parameters_sys()

    # this is a static controller, but I include this here for convenience
    # so that all controllers have an estimated disturbance
    d_est      = numpy.zeros(3)

    def __init__(self,parameters = None):
        
        if parameters != None:
            self.parameters.Throttle_neutral = parameters[0]

        self._old_input = array([1500.,1500.,0.,1500.])

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
        states_body = numpy.concatenate([states[0:6],R])

        if states.size == 18:

            # load
            eel = states[15:18]
            Rl = GetRotFromEulerAnglesDeg(eel)
            Rl = numpy.reshape(Rl,9)

            states_load = numpy.concatenate([states[9:15],Rl])

            self._old_input = controller(states_body,states_load,states_d,self.parameters)

            return self._old_input

        return self._old_input


# Controller
def controller(states,states_load,states_d,parameters):
    
    # mass of vehicles (kg)
    m = parameters.m
    ml = .136
    # is there some onboard thrust control based on the vertical acceleration?
    ml = .01

    # acceleration due to gravity (m/s^2)
    g  = parameters.g
    rospy.logwarn('g: '+str(g))

    # Angular velocities multiplication factor
    # Dw  = parameters.Dw
    
    # third canonical basis vector
    e3 = numpy.array([0.0,0.0,1.0])
    
    #--------------------------------------#
    # transported mass: position and velocity
    p  = states[0:3];
    v  = states[3:6];
    # thrust unit vector and its angular velocity
    R  = states[6:15];
    R  = numpy.reshape(R,(3,3))
    r3  = R.dot(e3)

    # load
    pl = states_load[0:3]
    vl = states_load[3:6]

    Lmeas = norm(p-pl)
    rospy.logwarn('rope length: '+str(Lmeas))
    L = 0.66

    #rospy.logwarn('param Throttle_neutral: '+str(parameters.Throttle_neutral))

    rl = (p-pl)/Lmeas
    omegal = skew(rl).dot(v-vl)/Lmeas
    omegal = zeros(3)

    
    #--------------------------------------#
    # desired quad trajectory
    pd = states_d[0:3] - L*e3;
    vd = states_d[3:6];
    ad = states_d[6:9];
    jd = states_d[9:12];
    sd = states_d[12:15];

    # rospy.logwarn(numpy.concatenate((v,vl,vd)))
    
    #--------------------------------------#
    # position error and velocity error
    ep = pl - pd
    ev = vl - vd

    rospy.logwarn('ep: '+str(ep))

    #--------------------------------------#
    gstar = g*e3 + ad
    d_gstar = jd
    dd_gstar = sd

    #rospy.logwarn('rl: '+str(rl))
    #rospy.logwarn('omegal: '+str(omegal))
    #rospy.logwarn('ev: '+str(ev))

# temp override
    #rl = array([0.,0.,1.])
    #omegal = zeros(3)
    #L = 1
    
    #--------------------------------------#

    # rospy.logwarn('ep='+str(ep)+' ev='+str(ev)+' rl='+str(rl)+' omegal='+str(omegal))

    Tl, tau, _, _ = backstep_ctrl(ep, ev, rl, omegal, gstar, d_gstar, dd_gstar)
    rospy.logwarn('g: '+str(g))

    U = (eye(3)-outer(rl,rl)).dot(tau*m*L)+rl*(
        # -m/L*inner(v-vl,v-vl)
        +Tl*(m+ml))
    U = R.T.dot(U)

    n = rl

    # -----------------------------------------------------------------------------#
    # STABILIZE MODE:APM COPTER
    # The throttle sent to the motors is automatically adjusted based on the tilt angle
    # of the vehicle (i.e increased as the vehicle tilts over more) to reduce the 
    # compensation the pilot must fo as the vehicles attitude changes
    # -----------------------------------------------------------------------------#
    # Full_actuation = m*(ad + u + g*e3 + self.d_est)
    Full_actuation = U

    # -----------------------------------------------------------------------------#

    U = numpy.zeros(4)

    Throttle = numpy.dot(Full_actuation,r3)
    # this decreases the throtle, which will be increased
    Throttle = Throttle*numpy.dot(n,r3)
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
    # Rz(psi)*Ry(theta_des)*Rx(phi_des) = r3_des

    # desired roll and pitch angles
    r3_des     = Full_actuation/numpy.linalg.norm(Full_actuation)
    r3_des_rot = Rz(-psi).dot(r3_des)


    sin_phi   = -r3_des_rot[1]
    sin_phi   = bound(sin_phi,1,-1)
    phi       = numpy.arcsin(sin_phi)
    U[1]      = phi

    sin_theta = r3_des_rot[0]/c(phi)
    sin_theta = bound(sin_theta,1,-1)
    cos_theta = r3_des_rot[2]/c(phi)
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

# controller code by Manuel H.:

def _sat(s):
    '''
    saturation function
    '''
    sat = s/sqrt(1. + s**2)
    d_sat = (1. + s**2)**(-3./2.)
    dd_sat = -3.*s*(1. + s**2)**(-5./2.)
    ddd_sat = (-3 + 12*s**2)/(1 + s**2)**(7./2.)

    int_sat = sqrt(1. + s**2) - 1.

    return sat, d_sat, dd_sat, ddd_sat, int_sat

def _Om(s):
    if s == 0.:
        Om = 0.
        d_Om = 1.
        dd_Om = 0.
        ddd_Om = 0.
    else:
        # sign and absolute value of s (implement positive part and use symmetry)
        sign_s = sign(s)
        s = abs(s)

        if s <= 1.:
            Om = s
            d_Om = 1.
            dd_Om = 0.
            ddd_Om = 0.
        elif s <= 2.:
            Om = 1./24.*(s**4 -4.*s**3 +6.*s**2 +20.*s +1)
            d_Om = 1./6.*(5. +3.*s -3.*s**2 +s**3)
            dd_Om = 1./2.*(-1.+s)**2
            ddd_Om = -1.+s
        else:
            Om = 1./24.*(4*s**3 -18*s**2 +52*s -15)
            d_Om = 1./6.*(13 -9*s +3*s**2)
            dd_Om = -(3./2.)+s
            ddd_Om = 1.

        # negative part of function (function is odd):
        Om = sign_s*Om
        dd_Om = sign_s*dd_Om

    return Om, d_Om, dd_Om, ddd_Om

def _Psi(v, Omv, sigp, d_Omv, d_sigp, dd_Omv, dd_sigp):
    delta = 1.

    if abs(v) < delta:
        Psi = 1.
        Psi_p = 0.
        Psi_v = 0.
        Psi_p_p = 0.
        Psi_p_v = 0.
        Psi_v_v = 0.
    else:
        Psi = (v + sigp)/(Omv + sigp)
        Psi_p = d_sigp*(Omv - v)/(Omv + sigp)**2
        Psi_v = (Omv + sigp - d_Omv*(v + sigp))/(Omv + sigp)**2
        Psi_p_p = -(Omv + sigp)**(-3)*((-2)*d_sigp**2 + dd_sigp*(Omv + sigp))*((-1)*Omv + v)
        Psi_p_v = d_sigp*(Omv + sigp)**(-3)*(-(1 + d_Omv)*Omv + (-1 + d_Omv)*sigp + 2*d_Omv*v)
        Psi_v_v = (Omv + sigp)**(-3)*(-dd_Omv*(Omv + sigp)*(sigp + v) + 2.*d_Omv*(-Omv - sigp + d_Omv*(sigp + v)))

    return Psi, Psi_p, Psi_v, Psi_p_p, Psi_p_v, Psi_v_v

def _rho(s, krho):
    return tuple(krho*array(_sat(s)))

def _bounded_ctrl_comp(p, v, k, krho):
    # evaluate subexpressions
    Omv, d_Omv, dd_Omv, ddd_Omv = _Om(v)
    sigp, d_sigp, dd_sigp, ddd_sigp, int_sigp = tuple(0.8*array(_sat(p)))
    rho_omv_sigp, d_rho_omv_sigp, dd_rho_omv_sigp, _, _ = _rho(Omv + sigp, krho)
    Psi, Psi_p, Psi_v, Psi_p_p, Psi_p_v, Psi_v_v = _Psi(v, Omv, sigp, d_Omv, d_sigp, dd_Omv, dd_sigp)

    # output and its derivatives
    u = -rho_omv_sigp - k*Psi*sigp/d_Omv - d_sigp*v/d_Omv
    u_p = -d_rho_omv_sigp*d_sigp - k*Psi_p*sigp/d_Omv - k*Psi*d_sigp/d_Omv - dd_sigp*v/d_Omv
    u_v = -d_rho_omv_sigp*d_Omv - k*Psi_v*sigp/d_Omv + k*Psi*sigp*dd_Omv/d_Omv**2 - d_sigp*(d_Omv - v*dd_Omv)/d_Omv**2

    u_p_p = -1/d_Omv*(
        dd_sigp*d_Omv*d_rho_omv_sigp + dd_rho_omv_sigp*d_Omv*d_sigp**2 + dd_sigp*k*Psi + 2*d_sigp*k*Psi_p + k*Psi_p_p*sigp + ddd_sigp*v)

    u_p_v = d_Omv**(-2)*(
        -d_sigp*(dd_rho_omv_sigp*d_Omv**3 - dd_Omv*k*Psi + d_Omv*k*Psi_v) - d_Omv*(
            dd_sigp + k*Psi_p_v*sigp) + dd_Omv*(
            k*Psi_p*sigp + dd_sigp*v))

    u_v_v = -d_Omv**(-3)*(
        dd_rho_omv_sigp*d_Omv**5 + dd_Omv*d_Omv**3*d_rho_omv_sigp + d_Omv**2*k*Psi_v_v*sigp + 2.*dd_Omv**2*(
            k*Psi*sigp + d_sigp*v) - d_Omv*(k*(ddd_Omv*Psi + 2*dd_Omv*Psi_v)*sigp + d_sigp*(2.*dd_Omv + ddd_Omv*v)))

    # Lyapunov function and its derivatives
    V = int_sigp*k + 0.5*(Omv + sigp)**2
    V_p = k*sigp + d_sigp*(Omv + sigp)
    V_v = d_Omv*(Omv + sigp)
    # V_p_p = d_sigp**2+d_sigp*k+dd_sigp*(Omv+sigp)
    V_p_v = d_Omv*d_sigp
    V_v_v = d_Omv**2 + dd_Omv*(Omv + sigp)

    d_V = V_p*v + V_v*u

    return u, u_p, u_v, u_p_p, u_v_v, u_p_v, V, d_V, V_p, V_v, V_p_v, V_v_v

def _unbounded_ctrl_comp(p, v):
    # 4*k1 has to be > k2**2
    k1 = .5
    k2 = .5
    w1 = k2/2./k1
    w2 = 1./k1

    kV = .1

    u = -k1*p -k2*v
    u_p = -k1
    u_v = -k2
    u_p_p = 0
    u_v_v = 0
    u_p_v = 0

    V = .5*p**2+w1*p*v+.5*w2*v**2
    V_p = p+w1*v
    V_v = w1*p+w2*v
    V_p_v = w1
    V_v_v = w2

    V = kV*V
    V_p = kV*V_p
    V_v = kV*V_v
    V_p_v = kV*V_p_v
    V_v_v = kV*V_v_v


    d_V = V_p*v + V_v*u

    return u, u_p, u_v, u_p_p, u_v_v, u_p_v, V, d_V, V_p, V_v, V_p_v, V_v_v

def _bounded_ctrl(p, v):
    # parameters for horizontal controller components
    krhoxy = .5
    kxy = 2

    # parameters for vertical controller components
    krhoz = .5
    kz = .5

    u = zeros(3)
    u_p = zeros((3, 3))
    u_v = zeros((3, 3))
    u_p_p = zeros((3, 3, 3))
    u_v_v = zeros((3, 3, 3))
    u_p_v = zeros((3, 3, 3))

    # V     = zeros(1)
    # VD    = zeros(1)

    V_p = zeros(3)
    V_v = zeros(3)
    V_v_p = zeros((3, 3))
    V_v_v = zeros((3, 3))

    u[0], u_p[0, 0], u_v[0, 0], u_p_p[0, 0, 0], u_v_v[0, 0, 0], u_p_v[0, 0, 0], V0, VD0, V_p[0], V_v[0], V_v_p[
        0, 0], V_v_v[0, 0] = \
        _bounded_ctrl_comp(p[0], v[0], kxy, krhoxy)

    u[1], u_p[1, 1], u_v[1, 1], u_p_p[1, 1, 1], u_v_v[1, 1, 1], u_p_v[1, 1, 1], V1, VD1, V_p[1], V_v[1], V_v_p[
        1, 1], V_v_v[1, 1] = \
        _bounded_ctrl_comp(p[1], v[1], kxy, krhoxy)

    u[2], u_p[2, 2], u_v[2, 2], u_p_p[2, 2, 2], u_v_v[2, 2, 2], u_p_v[2, 2, 2], V2, VD2, V_p[2], V_v[2], V_v_p[
        2, 2], V_v_v[2, 2] = \
        _bounded_ctrl_comp(p[2], v[2], kz, krhoz)

    V = V0 + V1 + V2
    VD = VD0 + VD1 + VD2

    return u, u_p, u_v, u_p_p, u_v_v, u_p_v, V, VD, V_p, V_v, V_v_p, V_v_v

def backstep_ctrl( p, v, r, omega, gstar, d_gstar, dd_gstar):
    # evaluate controller for double integrator
    u, u_p, u_v, u_p_p, u_v_v, u_p_v, Vpv, d_Vpv, Vpv_p, Vpv_v, Vpv_v_p, Vpv_v_v = _bounded_ctrl(p, v)

    kr = .5
    wr = 250.
    womega = 20.
    komega = .5

    Ul = u + gstar
    Tl = dot(Ul, r)
    # Tl = Ul[2] # for testing

    # Lie derivative of v
    d_v = u - (eye(3) - outer(r, r)).dot(Ul)

    rstar = Ul/norm(Ul)

    # Lie derivative of Ul
    d_Ul = diagonal(u_p)*v + diagonal(u_v)*d_v + d_gstar

    # turn 3rd-order tensors into vectors (assuming u is component wise -> tensors are diagonal)
    u_p_p_vec = sum(u_p_p, (1, 2))
    u_p_v_vec = sum(u_p_v, (1, 2))
    u_v_v_vec = sum(u_v_v, (1, 2))

    # Lie derivative of partial derivatives of u assuming u is component wise
    d_u_p = diagflat(u_p_p_vec*v + u_p_v_vec*d_v)
    d_u_v = diagflat(u_p_v_vec*v + u_v_v_vec*d_v)

    d_r = skew(omega).dot(r)
    dd_v = u_p.dot(v) + u_v.dot(d_v) + (outer(d_r, r) + outer(r, d_r)).dot(Ul) - (eye(3) - outer(r, r)).dot(
        d_Ul)
    dd_Ul = d_u_p.dot(v) + u_p.dot(d_v) + d_u_v.dot(d_v) + u_v.dot(dd_v) + dd_gstar

    d_rstar = (eye(3) - outer(rstar, rstar)).dot(d_Ul/norm(Ul))
    dd_rstar = -(outer(d_rstar, rstar) + outer(rstar, d_rstar)).dot(d_Ul)/norm(Ul) + (
        eye(3) - outer(rstar, rstar)).dot(dd_Ul/norm(Ul) - d_Ul*inner(d_Ul, Ul)/norm(Ul)**3)

    omegastar = kr*skew(r).dot(rstar) + skew(rstar).dot(d_rstar) - 1./wr*norm(Ul)*skew(r).dot(Vpv_v)
    d_omegastar = kr*skew(d_r).dot(rstar) + kr*skew(r).dot(d_rstar) + skew(rstar).dot(dd_rstar) - inner(Ul, d_Ul)/(
        wr*norm(Ul))*skew(r).dot(Vpv_v) - norm(Ul)/wr*skew(d_r).dot(Vpv_v) - norm(Ul)/wr*skew(r).dot(
        Vpv_v_p.dot(v) + Vpv_v_v.dot(d_v))

    # this is the actual control input
    tau = wr/womega*rstar + skew(d_r).dot(omega - omegastar) + komega*skew(r).dot(omega - omegastar) - skew(r).dot(
        d_omegastar)
    tau = (eye(3) - outer(r,r)).dot(tau)

    # Lyapunov function
    V = Vpv + wr*(1. - inner(r, rstar)) + .5*womega*norm(skew(r).dot(omega - omegastar))**2

    d_V = Vpv_p.dot(v) + Vpv_v.dot(u) - wr*kr*norm(skew(r).dot(rstar))**2 - womega*komega*norm(
        skew(r).dot(omega - omegastar))**2

    rospy.logwarn('Tl: '+str(Tl)+' Ul:'+str(Ul))
    rospy.logwarn('rl. '+str(r))

    return Tl, tau, d_V, V

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

