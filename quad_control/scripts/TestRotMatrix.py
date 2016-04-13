#!/usr/bin/env python
# this line is just used to define the type of document

import numpy
from numpy import *
from numpy import cos as c
from numpy import sin as s

import math

def bound(x,maxmax,minmin):

    return numpy.maximum(minmin,numpy.minimum(maxmax,x))


def Rx(tt):
    
    return numpy.array([[1.0,0.0,0.0],[0.0,c(tt),-s(tt)],[0.0,s(tt),c(tt)]])

# print Rx(60*3.14/180)

def Ry(tt):
    
    return numpy.array([[c(tt),0.0,s(tt)],[0.0,1,0.0],[-s(tt),0.0,c(tt)]])

# print Ry(60*3.14/180)

def Rz(tt):
    
    return numpy.array([[c(tt),-s(tt),0.0],[s(tt),c(tt),0.0],[0.0,0.0,1]])

# print Rz(60*3.14/180)


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


def GetRotFromEulerAngles(ee_rad):
    return Rz(ee_rad[2]).dot(Ry(ee_rad[1]).dot(Rx(ee_rad[0])))


a = numpy.array([1.0,2.0,3.0])
print a
print GetEulerAngles(GetRotFromEulerAngles(a))
print numpy.array([1.0,-2.0,3.0]) - numpy.array([math.pi,-math.pi,math.pi])

a = numpy.array([-1.0,1.0,0.5])
print GetEulerAngles(GetRotFromEulerAngles(a))
print a

a = numpy.array([-1.0,0.7,-0.5])
print a
print GetEulerAngles(GetRotFromEulerAngles(a))

a = numpy.array([-1.0,-0.8,-0.5])
print a
print GetEulerAngles(GetRotFromEulerAngles(a))