#!/usr/bin/env python
# this line is just used to define the type of document

import rospy

# controller node publishes message of this type so that GUI can plot stuff
from quad_control.msg import quad_state_and_cmd

# node will need to get state from qualysis, other sensor, or simulator
# it subscribes to a message of this tupe
from quad_control.msg import quad_state

# node will publish message quad_cmd that contains commands to simulator
from quad_control.msg import quad_cmd

# node will publish controller state if requested
from quad_control.msg import Controller_State

# import services defined in quad_control
# Four SERVICES ARE BEING USED: SaveData, TrajDes_GUI, Mocap_Id, StartSim
# SaveData is for saving data in txt file
# TrajDes is for selecting trajectory
# Mocap_Id for body detection from QUALISYS
# StartSim stop simulator
from quad_control.srv import *

# when Mocap is used this is necessary
import mocap_source

# to work with directories relative to ROS packages
from rospkg import RosPack

import numpy
from numpy import *
from numpy import cos as c
from numpy import sin as s

#-------------------------------------------------------#
# from SomeFunctions import *
from SomeFunctions import GetRotFromEulerAnglesDeg,Velocity_Filter

#-------------------------------------------------------#
controllers_dictionary = {}
from Controller0 import ControllerPD
controllers_dictionary[0] = ControllerPD
from Controller1 import ControllerPID
controllers_dictionary[1] = ControllerPID
from Controller2 import ControllerPID_bounded
controllers_dictionary[2] = ControllerPID_bounded
from ControllerACRO import ControllerOmega 
controllers_dictionary[3] = ControllerOmega

controllers_with_state = [1,2]

#-------------------------------------------------------#
trajectories_dictionary = {}

from Trajectory0 import traj_des_still
trajectories_dictionary[0] = traj_des_still
from Trajectory1 import traj_des_circle
trajectories_dictionary[1] = traj_des_circle

#-------------------------------------------------------#


class quad_controller():

    def __init__(self):

        # frequency of controlling action!!
        self.frequency = 90.0

        # state of quad: position, velocity and attitude 
        # ROLL, PITCH, AND YAW (EULER ANGLES IN DEGREES)
        self.state_quad = numpy.zeros(3+3+3)

        # dy default, desired trajectory is staying still in origin
        traj_class = trajectories_dictionary[0]
        # zero vector
        zvec  = numpy.zeros(3)
        # Identity matrix
        Ident = numpy.array([[1.0,0.0,0.0],[0.0,1.0,0.0],[0.0,0.0,1.0]])
        # dy default
        self.TrajGenerator = traj_class(zvec,Ident)

        # initialize counter for publishing to GUI
        # we only publish when self.PublishToGUI =1
        self.PublishToGUI = 1
        # Frequency of publishing to GUI (Hz)
        frequency_PubToGui = 10
        # we reset counter when self.PublishToGUI >= PublishToGUIBound
        self.PublishToGUIBound = int(self.frequency/frequency_PubToGui)

        # intiialization should be done in another way,
        # but median will take care of minimizing effects
        self.VelocityEstimator = Velocity_Filter(3,numpy.zeros(3),0.0)

        # controller selected by default
        Cler_class = controllers_dictionary[3]
        self.controller = Cler_class()


    def GET_STATE_(self):

        # if simulator is on, state is updated by subscription
        # if mocap is on, state comes from mocap
        if self.flagMOCAP == True:            

            bodies = self.Qs.get_body(self.body_id)

            # rospy.logwarn(bodies)

            if bodies != 'off':    


                x=bodies["x"]
                y=bodies["y"]
                z=bodies["z"]   
                # position
                p = numpy.array([x,y,z])

                # velocity
                #v = numpy.array([data.vx,data.vy,data.vz])
                v = self.VelocityEstimator.out(p,rospy.get_time())

                roll =bodies["roll"]
                # THIS IS VERY IMPORTANT: PITCH FROM QUALISYS IS COMING NEGATIVE
                # WHEN IT SHOULD BE POSITIVE
                pitch=-bodies["pitch"]
                yaw  =bodies["yaw"]
                # attitude: euler angles
                ee = numpy.array([roll,pitch,yaw])

                # collect all components of state
                self.state_quad = numpy.concatenate([p,v,ee])  

            # else:
            # do nothing, keep previous state

    # callback when simulator publishes states
    def get_state(self, data):

        # position
        p = numpy.array([data.x,data.y,data.z])

        # velocity
        #v = numpy.array([data.vx,data.vy,data.vz])
        v = self.VelocityEstimator.out(p,rospy.get_time())


        # attitude: euler angles
        ee = numpy.array([data.roll,data.pitch,data.yaw])

        # collect all components of state
        self.state_quad = numpy.concatenate([p,v,ee])  

    # for obtaining current desired state
    def traj_des(self):

        # time for trajectory generation
        time_TrajDes = rospy.get_time() - self.time_TrajDes_t0

        return self.TrajGenerator.output(time_TrajDes)


    # callback for when Saving data is requested
    def handle_Save_Data(self,req):
        
        if req.ToSave == True:
            # if GUI request data to be saved create file
            
            # namespace, e.g. /Iris1/
            ns = rospy.get_namespace()
            # remove / symbol to namespace: e.g, we get ns= Iris1
            ns = ns.replace("/", "")

            # string for time: used for generating files
            tt = str(int(rospy.get_time() - self.TimeSaveData))

            # determine ROS workspace directory
            rp = RosPack()
            package_path = rp.get_path('quad_control')
            self.file_handle  = file(package_path+'/../../'+ns+'_data_'+tt+'.txt', 'w')

            # if GUI request data to be saved, set falg to true
            self.SaveDataFlag = True
        else:
            # if GUI request data NOT to be saved, set falg to False
            self.SaveDataFlag = False

        # return message to Gui, to let it know resquest has been fulfilled
        return SaveDataResponse(True)


    # callback for publishing state of controller (or stop publishing)
    def handle_Controller_State_Srv(self,req):

        # if controller belongs to list of controllers that have a state
        if req.flag_controller in controllers_with_state:
            # if GUI request for state of controller, parameters = 0
            if int(req.parameters[0]) == 0:

                    # if publishging already, stop publish
                    # if not publishing, start publishing
                    self.flagPublish_ctr_st = not self.flagPublish_ctr_st

            # if GUI request for reseting state of controller, parameters = 1
            elif int(req.parameters[0]) == 1:
                self.controller.reset_estimate_xy()
            elif int(req.parameters[0]) == 2:
                self.controller.reset_estimate_z()

        # return message to Gui, to let it know resquest has been fulfilled
        return Controller_SrvResponse(True)



    # callback for when changing controller is requested
    def handle_Controller_Srv(self,req):

        # if GUI request certain controller, update flag on desired controller
        fg_Cler = req.flag_controller

        # some parameters user can change easily 
        # req.parameters is a tuple
        if len(req.parameters) == 0:
            # if tuple req.parameters is empty:
            parameters = None
        else:     
            # if tuple is not empty, cast parameters as numpy array 
            parameters = numpy.array(req.parameters)

        # update class for Controller
        Cler_class = controllers_dictionary[fg_Cler]
        # Cler_class = controllers_dictionary[2]

        self.controller = Cler_class(parameters)

        # return message to Gui, to let it know resquest has been fulfilled
        return Controller_SrvResponse(True)


    # callback for when changing desired trajectory is requested
    def handle_TrajDes_service(self,req):

        # if GUI request certain trajectory, update flag on desired trajectory 
        flagTrajDes = req.trajectory

        TrajDes_OffSet = numpy.array(req.offset)

        ee     = numpy.array(req.rotation)
        TrajDes_Rotation = GetRotFromEulerAnglesDeg(ee)

        # some parameters user can change easily 
        # req.parameters is a tuple
        if len(req.parameters) == 0:
            # if tuple req.parameters is empty:
            TrajDes_parameters = None
        else:     
            # if tuple is not empty, cast parameters as numpy array 
            TrajDes_parameters = numpy.array(req.parameters)   

        # update class for TrajectoryGenerator
        traj_class = trajectories_dictionary[flagTrajDes]

        self.TrajGenerator = traj_class(TrajDes_OffSet,TrajDes_Rotation,TrajDes_parameters)

        # we need to update initial time for trajectory generation
        self.time_TrajDes_t0 = rospy.get_time()

        # return message to Gui, to let it know resquest has been fulfilled
        return TrajDes_SrvResponse(True)

    # function to stop simulator
    def stop_simulator(self):
        try: 
            rospy.wait_for_service('StartSimulator',1.0)

            try:
                AskForStart = rospy.ServiceProxy('StartSimulator', StartSim)

                reply = AskForStart(False)
                if reply.Started == True:
                    return True
                else:
                    return False
            except:
                return False

        except:
            return False

    #callback for turning ON/OFF Mocap and turning OFF/ON the subscription to the simulator
    def handle_Mocap(self,req):

        # if mocap is turned on
        if self.flagMOCAP_On == True:

            # request to turn OFF Mocap, and turn on subscription to Simulator messages
            if req.On == False:

                # in case Qs is not defined yet
                try: 
                    # close mocap connection
                    # self.Qs._stop_measurement()
                    del self.Qs

                    self.flagMOCAP_On = False

                    # set flag to OFF
                    self.flagMOCAP = False

                    # subscribe again to simultor messages
                    self.SubToSim = rospy.Subscriber("quad_state", quad_state, self.get_state) 

                    # service has been provided
                    return Mocap_IdResponse(True,True)
                except:
                    # service was NOT provided
                    return Mocap_IdResponse(True,False) 

            # if we are requested to change the body Id
            if req.change_id == True:

                # see list of available bodies
                bodies = self.Qs.get_updated_bodies()

                # check if body_id available
                body_indice = -1

                # Get the corresponding id of the body
                if isinstance(bodies,list):
                    for i in range(0,len(bodies)):
                        # rospy.logwarn(bodies[i]['id'])
                        if(bodies[i]['id']==req.id):
                            body_indice=i

                # save body id
                self.body_id = req.id                        

                # stop simulator
                stop = self.stop_simulator()

                # if simulator stopped
                if stop == True:

                    if body_indice == -1:

                        # body does not exist
                        self.flagMOCAP = False

                        # body does not exist, but service was provided
                        return Mocap_IdResponse(False,True)
                    else:
                        # body exists
                        
                        # set flag to on
                        self.flagMOCAP = True

                        # body EXISTS, and service was provided
                        return Mocap_IdResponse(True,True)
                else:
                    self.flagMOCAP = False
                    # service was NOT provided
                    return Mocap_IdResponse(False,False)


        else:
            # if Mocap is turned off, and we are requested to turn it on
            if req.On == True:
                # establish connection to qualisys
                self.Qs = mocap_source.Mocap(info=0)

                # stop subscription to data from simulator
                # unsubscribe to topic
                self.SubToSim.unregister()

                self.flagMOCAP_On = True

                # service was provided
                return Mocap_IdResponse(False,True)

    # return list of bodies detected by mocap or numbers 1 to 99 if not available
    def handle_available_bodies(self, dummy):
        if self.flagMOCAP_On:
            try: # sometimes mocap causes unpredictable errors
                bodies = self.Qs.find_available_bodies(False)
                if len(bodies) > 0:
                    return {"bodies": bodies[0]}
            except:
                pass

        return {"bodies": range(0,100)}


    def PublishToGui(self,states_d,Input_to_Quad):

        # WE ONLY PUBLIS TO TO GUI AT A CERTAIN FREQEUNCY
        # WHICH IS NOT NECESSARILY THE FREQUENCY OF THE NODE
        if self.PublishToGUI <= self.PublishToGUIBound:
            # if we dont publish, we increase counter
            self.PublishToGUI = self.PublishToGUI + 1
        else:
            # if we publish, we increase counter
            self.PublishToGUI = 1

            # create a message of type quad_state_and_cmd
            st_cmd = quad_state_and_cmd()

            # get current time
            st_cmd.time  = rospy.get_time()

            # state of quad comes from QUALISYS, or other sensor
            st_cmd.x     = self.state_quad[0]
            st_cmd.y     = self.state_quad[1]
            st_cmd.z     = self.state_quad[2]
            st_cmd.vx    = self.state_quad[3]
            st_cmd.vy    = self.state_quad[4]
            st_cmd.vz    = self.state_quad[5]
            st_cmd.roll  = self.state_quad[6]
            st_cmd.pitch = self.state_quad[7]
            st_cmd.yaw   = self.state_quad[8]
            
            st_cmd.xd    = states_d[0]
            st_cmd.yd    = states_d[1]
            st_cmd.zd    = states_d[2]
            st_cmd.vxd   = states_d[3]
            st_cmd.vyd   = states_d[4]
            st_cmd.vzd   = states_d[5]

            st_cmd.cmd_1     = Input_to_Quad[0]
            st_cmd.cmd_2     = Input_to_Quad[1]
            st_cmd.cmd_3     = Input_to_Quad[2]
            st_cmd.cmd_4     = Input_to_Quad[3]

            st_cmd.cmd_5     = 1500.0
            st_cmd.cmd_6     = 1500.0
            st_cmd.cmd_7     = 1500.0
            st_cmd.cmd_8     = 1500.0

            self.pub.publish(st_cmd)     

            # controller is supposed to be published
            if self.flagPublish_ctr_st:
                # publish controller state
                msg       = Controller_State()
                msg.time  = rospy.get_time()
                msg.d_est = self.controller.d_est
                self.pub_ctr_st.publish(msg) 
        
    def PublishToQuad(self,Input_to_Quad):

        # create a message of the type quad_cmd, that the simulator subscribes to 
        cmd = quad_cmd()
        cmd.cmd_1 = Input_to_Quad[0];
        cmd.cmd_2 = Input_to_Quad[1];
        cmd.cmd_3 = Input_to_Quad[2];
        cmd.cmd_4 = Input_to_Quad[3];

        cmd.cmd_5 = 1500.0
        cmd.cmd_6 = 1500.0
        cmd.cmd_7 = 1500.0
        cmd.cmd_8 = 1500.0

        self.pub_cmd.publish(cmd)


    def control_compute(self):

        # node will be named quad_control (see rqt_graph)
        rospy.init_node('quad_control', anonymous=True)

        # message published by quad_control to GUI 
        self.pub = rospy.Publisher('quad_state_and_cmd', quad_state_and_cmd, queue_size=10)
        
        # message published by quad_control that simulator will subscribe to 
        self.pub_cmd = rospy.Publisher('quad_cmd', quad_cmd, queue_size=10)

        # for publishing state of the controller
        self.pub_ctr_st = rospy.Publisher('ctr_state', Controller_State, queue_size=10)
        # initialize flag for publishing controller state at false
        self.flagPublish_ctr_st = False

        # controller needs to have access to STATE of the system
        # this can come from QUALISYS, a sensor, or the simulator
        self.SubToSim = rospy.Subscriber("quad_state", quad_state, self.get_state) 

        #-----------------------------------------------------------------------#
        #-----------------------------------------------------------------------#
        # TO SAVE DATA FLAG
        # by default, NO data is saved
        self.SaveDataFlag = False
        # we will use this just for convenience, when generating the names of the files
        # where the data will be saved
        self.TimeSaveData = rospy.get_time()
        # Service is created, so that data is saved when GUI requests
        Save_data_service = rospy.Service('SaveDataFromGui', SaveData, self.handle_Save_Data)


        #-----------------------------------------------------------------------#
        #-----------------------------------------------------------------------#
        # SERVICE FOR SELECTING DESIRED TRAJECTORY
        # by default, STAYING STILL IN ORIGIN IS DESIRED TRAJECTORY
        # self.flagTrajDes = 0
        # Service is created, so that data is saved when GUI requests
        TrajDes_service = rospy.Service('TrajDes_GUI', TrajDes_Srv, self.handle_TrajDes_service)

        # initialize initial time for trajectory generation
        self.time_TrajDes_t0 = rospy.get_time()

        #-----------------------------------------------------------------------#
        #-----------------------------------------------------------------------#
        # flag for MOCAP is initialized as FALSE
        # flag for wheter Mocap is being used or not
        self.flagMOCAP    = False
        # Flag for whether Mocap is ON or OFF
        self.flagMOCAP_On = False
        # Service is created, so that Mocap is turned ON or OFF whenever we want
        Save_MOCAP_service = rospy.Service('Mocap_Set_Id', Mocap_Id, self.handle_Mocap)


        # Service for providing list of available mocap bodies to GUI
        mocap_available_bodies = rospy.Service('MocapBodies', MocapBodies, self.handle_available_bodies)


        #-----------------------------------------------------------------------#
        #-----------------------------------------------------------------------#
        # Service is created, so that user can change controller on GUI
        Chg_Contller = rospy.Service('Controller_GUI', Controller_Srv, self.handle_Controller_Srv)


        #-----------------------------------------------------------------------#
        #-----------------------------------------------------------------------#
        # Service for publishing state of controller 
        # we use same type of service as above
        Contller_St = rospy.Service('Controller_State_GUI', Controller_Srv, self.handle_Controller_State_Srv)


        rate = rospy.Rate(self.frequency)

        while not rospy.is_shutdown():

            time = rospy.get_time()

            # get state:
            # if MOCAP in on we ask MOCAP
            # if MOCAP in off, we are subscribed to Simulator topic
            self.GET_STATE_()

            # states for desired trajectory
            states_d = self.traj_des()

            # compute input to send to QUAD
            Input_to_Quad = self.controller.output(time,self.state_quad,states_d)
            
            # Publish commands to Quad
            self.PublishToQuad(Input_to_Quad)

            # publish to GUI (it also contains publish state of Control to GUI)
            self.PublishToGui(states_d,Input_to_Quad)


            if self.SaveDataFlag == True:
                # if we want to save data
                numpy.savetxt(self.file_handle, [concatenate([[rospy.get_time()], self.state_quad, states_d[0:9], Input_to_Quad])],delimiter=' ')

            # go to sleep
            rate.sleep()    


if __name__ == '__main__':
    A_Controller = quad_controller()
    try:
        A_Controller.control_compute()
    except rospy.ROSInterruptException:
        pass
