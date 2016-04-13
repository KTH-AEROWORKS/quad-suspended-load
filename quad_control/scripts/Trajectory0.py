#!/usr/bin/env python
# this line is just used to define the type of document

import numpy

class traj_des_still():

    def __init__(self,offset,Rotation,parameters=None):

        self.offset   = offset

        self.Rotation = Rotation


    def traj_des(self):
        # by default, desired trajectory is to saty in origin
        return numpy.zeros(3*5)

    def output(self,t):
        return self.add_offset_and_rot(self.traj_des())


    def add_offset_and_rot(self,position):

        pos_out = numpy.zeros(3*5)
        
        R = self.Rotation
        
        pos_out[0:3]   = R.dot(position[0:3])
        pos_out[3:6]   = R.dot(position[3:6])
        pos_out[6:9]   = R.dot(position[6:9])
        pos_out[9:12]  = R.dot(position[9:12])
        pos_out[12:15] = R.dot(position[12:15])

        pos_out[0:3] = pos_out[0:3] + self.offset   

        return pos_out