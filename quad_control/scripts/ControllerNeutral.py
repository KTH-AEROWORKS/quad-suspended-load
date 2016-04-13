#!/usr/bin/env python
# this line is just used to define the type of document

# useful for rospy.logwar()
import rospy

import numpy

class ControllerNeutral():
    
    # this is a static controller, but I include this here for convenience
    # so that all controllers have an estimated disturbance
    d_est      = numpy.zeros(3)

    def __init__(self,parameters = None):
        pass    

    # def reset_estimate_xy(self):
    #     return

    # def reset_estimate_z(self):
    #     return   

    # def update_parameters(self,parameters):
        # self.parameters = parameters

    def output(self,t,states,states_d):
        # neutral values and minimum trottle
        U  = numpy.array([1500,1500,1000,1500])
        return U
