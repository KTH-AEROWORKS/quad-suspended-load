#!/usr/bin/env python
# this is just to define file type

import rospy

from std_msgs.msg import String


# simulator will publish quad state
from quad_control.msg import quad_state

# simulator will publish quad state
from quad_control.msg import quad_cmd

# import services defined in quad_control
# SERVICE for Starting Simulation
# SERVICE for Reseting Simulation
# SERVICE for changing Simulator
from quad_control.srv import *


# for integrating and solving differential equation
from scipy.integrate import ode

# from copy import deepcopy

# # for ploting
# import matplotlib.pyplot as plt

import numpy
from numpy import *
from numpy import cos as c
from numpy import sin as s


from SomeFunctions import GetEulerAnglesDeg

from Simulator0 import ZeroDynamics
from Simulator1 import AttitudeInnerLoop
from Simulator2 import DynamicsOmega

simulators_dictionary = {0:ZeroDynamics,1:AttitudeInnerLoop,2:DynamicsOmega}


#----------------------------------------------#
#----------------------------------------------#
# This class is necessary because of solver
# this DOES NOT work: r.set_f_params(deepcopy(self.U),parameters) if U is  ARRAY
# this works: r.set_f_params(Input(self.U),parameters) if U is  ARRAY

class Input(object):

    def __init__(self,U):

        self.U0 = U[0]
        self.U1 = U[1]
        self.U2 = U[2]
        self.U3 = U[3]

        return    
#----------------------------------------------#
#----------------------------------------------#


class Simulator():

    def __init__(self):

        # frequency of node (Hz)
        self.frequency = 100

        # delay for starting simulator (sec)
        self.TimeDelay = 2.0

        # input is initialized at zero
        # input comes from subscribing to topic that comes from controller node
        self.U = numpy.array([0.0,0.0,0.0,0.0])


        # by default, we get the class [0] which is staying still
        Sim_class = simulators_dictionary[0]
        # sim is an instant/object of class Sim_class
        sim       = Sim_class()
        # f is the dynamics, which is defined ALWAYS AS THE OUTPUT FUNCTION
        f         = sim.output
        # r is the solver object used for integration
        self.r    = ode(f).set_integrator('dopri5')

        #---------------------------------------------------------------------#
        # initial states

        # initial quad position (m)
        x0 = numpy.array([0,0,0])
        # initial quad velocity (m/s)
        v0 = numpy.array([0,0,0]) 
        # quad rotation matrix (body to inertial)
        # R0 = Rz(0).dot(Ry(0).dot(Rx(0.0*3.14/180)))
        # R0 = numpy.reshape(R0,9)
        # initial rotation matrix : Identity
        R0 = numpy.array([[1.0,0.0,0.0],[0.0,1.0,0.0],[0.0,0.0,1.0]])
        R0 = numpy.reshape(R0,9)

        # collecting all initial states0
        self.states0  = concatenate([x0,v0,R0])

        # self.r = ode(f).set_integrator('dopri5')
        self.r.set_initial_value(self.states0,0.0)

        #---------------------------------------------------------------------#

    # this is the callback function that is used when input is found to be published
    def get_input(self, data):
        # create zero vector
        U = numpy.zeros(4)
        U[0] = data.cmd_1
        U[1] = data.cmd_2
        U[2] = data.cmd_3
        U[3] = data.cmd_4

        # update input 
        self.U = U


    # callback used when starting simulator
    def handle_Start_service(self,req):
        
        if req.Start == True:
            # if GUI request simulation to be started
            self.StartFlag = True
        else:
            # if GUI request data NOT to be saved, set flag to False
            self.StartFlag = False

        # return message to Gui, to let it know resquest has been fulfilled
        return StartSimResponse(True)


    # callback used when reseting simulator
    def handle_Reset_service(self,req):
        
        # simulator has been asked to reset
        self.r.set_initial_value(self.states0,0.0)        

        # return message to Gui, to let it know resquest has been fulfilled
        return StartSimResponse(True)


    # callback used for changing simulator
    def handle_Simulator_Srv(self,req):

        # if GUI request certain simulator, change flag
        fg_Sim = req.flag_simulator

        # some parameters user can change easily 
        # req.parameters is a tuple
        if len(req.parameters) == 0:
            # if tuple req.parameters is empty:
            parameters = None
        else:     
            # if tuple is not empty, cast parameters as numpy array 
            parameters = numpy.array(req.parameters)

        old_states = self.r.y
        old_time   = self.r.t

        # update class for Simulator
        Sim_class = simulators_dictionary[fg_Sim]
        # sim is an instant/object of class Sim_class
        sim       = Sim_class(parameters)
        # f is the dynamics, which is defined ALWAYS AS THE OUTPUT FUNCTION
        f         = sim.output
        # r is the solver object used for integration
        self.r    = ode(f).set_integrator('dopri5')
        # initial conditions
        self.r.set_initial_value(old_states,old_time)


        # return message to Gui, to let it know resquest has been fulfilled
        return Simulator_SrvResponse(True)

    def write_state(self):

        # create a message of type quad_state_and_cmd
        state = quad_state()
        
        # get current time
        state.time = rospy.get_time()

        state.x  = self.r.y[0]
        state.y  = self.r.y[1]
        state.z  = self.r.y[2]
        state.vx = self.r.y[3]
        state.vy = self.r.y[4]
        state.vz = self.r.y[5]

        # rotation matrix
        R  = self.r.y[6:15]
        R  = numpy.reshape(R,(3,3))
        ee = GetEulerAnglesDeg(R)

        state.roll  = ee[0]
        state.pitch = ee[1]
        state.yaw   = ee[2]        

        return state


    def simulate_quad(self):

        # simulator node 
        rospy.init_node('simulate_quad', anonymous=True)

        #-----------------------------------------------------------------------#
        #-----------------------------------------------------------------------#

        # Simulator subscribes to command inputs, published by a controller
        rospy.Subscriber("quad_cmd", quad_cmd, self.get_input)

        #-----------------------------------------------------------------------#
        #-----------------------------------------------------------------------#

        # this node is a simulator, thus it will publish the state of the quad
        # it uses the commands -- that it is subscribed to -- to solve differential equations 
        pub = rospy.Publisher('quad_state', quad_state, queue_size=10)
        

        #-----------------------------------------------------------------------#
        #-----------------------------------------------------------------------#
        # TO SAVE FLAG
        # by default, no dynamics: everything stopped
        self.StartFlag = False
        # Service is created, so that simulation can be started or stopped
        Start_service = rospy.Service('StartSimulator', StartSim, self.handle_Start_service)
        # Service is created, so that simulation can be reseted
        Reset_service = rospy.Service('ResetSimulator', StartSim, self.handle_Reset_service)
        
        # NOTICE THAT SERVICE TYPE IS THE SAME FOR BOTH SERVICES ABOVE

        #-----------------------------------------------------------------------#
        #-----------------------------------------------------------------------#


        #-----------------------------------------------------------------------#
        #-----------------------------------------------------------------------#
        # Service is created, so that user can change simulator on GUI
        Chg_Simulator = rospy.Service('Simulator_GUI', Simulator_Srv, self.handle_Simulator_Srv)


        # solve differential equations at frequency
        rate = rospy.Rate(self.frequency)


        while not rospy.is_shutdown():

            
            # WARNING: IT IS VERY IMPORTANT THAT U0, U1, U2 AND U3 ARE PROVIDED THIS WAY
            # I CANNOT PROVIDE SELF.U TO THE INTEGRATION BECAUSE IT IS CHANGING 
            # AND IT MESSES UP THE INTEGRATION!!! 
            # thid DOES NOT work: r.set_f_params(deepcopy(self.U),parameters)
            
            # we delay system the initialization of the system by a TimeDelay
            if (self.r.t >= self.TimeDelay and self.StartFlag):
                # set dynamics vector accordinf to current input vector
                # input vector is assumed constant during integration
                self.r.set_f_params(Input(self.U))
                #  integrate equation for period of loop
                self.r.integrate(self.r.t + 1.0/self.frequency);
                # reset initial state and initial time
                self.r.set_initial_value(self.r.y, self.r.t)
            else:
                # need to update initial state and time
                self.r.set_initial_value(self.r.y, self.r.t + 1.0/self.frequency)
            
            # create a message of type quad_state with current state
            state = self.write_state()
            # publish current state
            pub.publish(state)

            # let node sleep
            rate.sleep()

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()        


if __name__ == '__main__':
    sim = Simulator()
    sim.simulate_quad()
