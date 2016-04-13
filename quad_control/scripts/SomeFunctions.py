#!/usr/bin/env python
# this is just to define file type

import rospy

import numpy

from numpy import *

from numpy import cos as c
from numpy import sin as s


# we want access to pi
import math


def Rx(tt):
    
    return numpy.array([[1.0,0.0,0.0],[0.0,c(tt),-s(tt)],[0.0,s(tt),c(tt)]])

# print Rx(60*3.14/180)

def Ry(tt):
    
    return numpy.array([[c(tt),0.0,s(tt)],[0.0,1,0.0],[-s(tt),0.0,c(tt)]])

# print Ry(60*3.14/180)

def Rz(tt):
    
    return numpy.array([[c(tt),-s(tt),0.0],[s(tt),c(tt),0.0],[0.0,0.0,1]])

# print Rz(60*3.14/180)

def skew(xx):
    
    x = xx[0]
    y = xx[1]
    z = xx[2]
    
    return numpy.array([[0,-z,y],[z,0,-x],[-y,x,0]])

# print skew([1,2,3])

#--------------------------------------------------------------------------#
# orthogonal projection operator
def OP(x):
    
    return -skew(x).dot(skew(x))

#print OP([1,2,3])
#print OP([1,0,0])

#--------------------------------------------------------------------------#
# unit vector
def unit_vec(psi,theta):

    e1  = numpy.array([1.0,0.0,0.0])
    aux = Rz(psi).dot(e1)
    aux = Ry(theta).dot(aux)

    return aux

#print unit_vec(45*3.14/180,0)
#print unit_vec(45*3.14/180,45*3.14/180)
#print unit_vec(0*3.14/180,-90*3.14/180)




#--------------------------------------------------------------------------#
def GetEulerAngles(R):

    #phi   = atan2(R(3,2),R(3,3));
    #theta = asin(-R(3,1));
    #psi   = atan2(R(2,1),R(1,1));

    EULER = numpy.array([0.0,0.0,0.0])

    R

    EULER[0] = numpy.arctan2(bound(R[2,1],1,-1),bound(R[2,2],1,-1));
    EULER[1] = numpy.arcsin(-bound(R[2,0],1,-1));
    EULER[2] = numpy.arctan2(bound(R[1,0],1,-1),bound(R[0,0],1,-1));    

    return EULER



def bound(x,maxmax,minmin):

    return numpy.maximum(minmin,numpy.minimum(maxmax,x))



def GetEulerAnglesDeg(R):

    return GetEulerAngles(R)*180.0/math.pi



def GetRotFromEulerAngles(ee_rad):
    return Rz(ee_rad[2]).dot(Ry(ee_rad[1]).dot(Rx(ee_rad[0])))

def GetRotFromEulerAnglesDeg(ee_deg):
    return GetRotFromEulerAngles(ee_deg*math.pi/180.0)

#--------------------------------------------------------------------------#
#--------------------------------------------------------------------------#
# For computing velocity from position measurements

class Median_Filter():
    # N is order of median filter
    def __init__(self, N):
        self.N = N
        self.data = numpy.zeros(N)
    
    def update_data(self,new_data):
        N = self.N
        self.data[:-1] = self.data[1:]
        self.data[-1]  = new_data

    def output(self):
        return numpy.median(self.data)

    def up_and_out(self,new_data):
        self.update_data(new_data)
        return self.output()

class Median_Filter_3D():
    # N is order of median filter
    def __init__(self, N):
        self.N = N
        self.Dx =  Median_Filter(N)
        self.Dy =  Median_Filter(N)
        self.Dz =  Median_Filter(N)

    def up_and_out(self,new_data):
        Dx_new = self.Dx.up_and_out(new_data[0])
        Dy_new = self.Dy.up_and_out(new_data[1])
        Dz_new = self.Dz.up_and_out(new_data[2])
        return numpy.array([Dx_new,Dy_new,Dz_new])


class Velocity_Filter():
    def __init__(self,N,old_position,old_time):
        self.median_filter = Median_Filter_3D(N)
        self.old_position = old_position
        self.old_time = old_time

    def out(self,new_position,new_time):
        dt = new_time - self.old_time
        vel_estimate =  (new_position - self.old_position)/dt
        self.old_position = new_position
        self.old_time = new_time
        return self.median_filter.up_and_out(vel_estimate)


#--------------------------------------------------------------------------#
#--------------------------------------------------------------------------#