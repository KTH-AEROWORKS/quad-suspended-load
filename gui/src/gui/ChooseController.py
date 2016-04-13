import os
import rospy
import QtGui
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget
from PyQt4.QtCore import QObject, pyqtSignal
import PyQt4.QtCore
import pyqtgraph as pg

# necessary to have gui as a client, asking controller to save data
# from python_qt_binding.QtCore import QTimer, Slot
# from python_qt_binding.QtCore import pyqtSlot


import numpy

# import analysis
# import utils
import subprocess



# import services defined in quad_control
# SERVICE BEING USED: Controller_srv
from quad_control.srv import *

# import message of the type controller_state
# because this gui will be able to display the state of the controller
from quad_control.msg import Controller_State


import argparse


class ChooseControllerPlugin(Plugin):

    # for controller state
    ctr_st = pyqtSignal(numpy.ndarray)

    # ---------------------------------------------- #
    # ---------------------------------------------- #
    # Necessary constants

    # size of vectors: INTEGER
    Size_Vector = 100


    def __init__(self, context,namespace = None):

        # it is either "" or the input given at creation of plugin
        self.namespace = self._parse_args(context.argv())


        super(ChooseControllerPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('ChooseControllerPlugin')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns
        
        
        
        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which is a sibling of this file
        # in this example the .ui and .py file are in the same folder
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'ChooseController.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('ChooseControllerUi')
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        # ---------------------------------------------- #
        # ---------------------------------------------- #

        # BUTTON TO SET DESIRED CONTROLLER
        self._widget.SetControllerButton.clicked.connect(self.SetController)

        # BUTTON TO SET DESIRED TRAJECTORY
        self._widget.Reset_Int_XY.clicked.connect(self.RESET_xy_PID)

        # BUTTON TO SET DESIRED TRAJECTORY
        self._widget.Reset_Int_Z.clicked.connect(self.RESET_z_PID)

        # ------------------------------------------------------------------#
        # ------------------------------------------------------------------#
        # get some values from launch file if theyre available
        wn  = 1
        xsi = numpy.sqrt(2)/2
        kv   = rospy.get_param("kv",2.0*xsi*wn)
        kp   = rospy.get_param("kp",wn*wn)

        Throttle_neutral = rospy.get_param("Throttle_neutral_ctr",1484.0)


        # ------------------------------------------------------------------#
        # ------------------------------------------------------------------#

        # Default values for buttons: PD Controller
        self._widget.PgainXY.setValue(kp)
        self._widget.DgainXY.setValue(kv)
        self._widget.PgainZ.setValue(kp)
        self._widget.DgainZ.setValue(kv)
        self._widget.ThrottleNeutral.setValue(Throttle_neutral)

        # Default values for buttons: PID Controller
        self._widget.PgainXY_PID.setValue(kp)
        self._widget.DgainXY_PID.setValue(kv)
        self._widget.IgainXY_PID.setValue(0.0)
        self._widget.PgainZ_PID.setValue(kp)
        self._widget.DgainZ_PID.setValue(kv)
        self._widget.DgainXY_PID.setValue(kv)
        self._widget.IgainZ_PID.setValue(0.0)
        self._widget.ThrottleNeutral_PID.setValue(Throttle_neutral)


        # ---------------------------------------------- #
        # ---------------------------------------------- #   

        # window for plots
        plotwidget = pg.PlotWidget()
        plotwidget.getPlotItem().addLegend()
        plotwidget.setYRange(-2.0,2.0)      
        
        layout        = QtGui.QGridLayout()
        layout.addWidget(plotwidget)


        # labels for window with positions
        plotwidget.getPlotItem().setLabel('left','disturbance estimate','m/s/s')
        plotwidget.getPlotItem().setLabel('bottom','time','s')

        # complete putting labels
        self._widget.frame.setLayout(layout)

        # ---------------------------------------------- #
        # ---------------------------------------------- #          

        # time vector
        self.timevector = [0]*self.Size_Vector

        
        #Setting variables for each coordinate and channel

        # ---------------------------------------------- #  
        # positions x,y,z
        self.DXplotvector = [0]*self.Size_Vector
        self.DXcurve = plotwidget.getPlotItem().plot(self.timevector,self.DXplotvector, name='x')
        self.DXcurve.setPen(pg.mkPen('r'))
        
        self.DYplotvector = [0]*self.Size_Vector
        self.DYcurve = plotwidget.getPlotItem().plot(self.timevector,self.DYplotvector, name='y')
        self.DYcurve.setPen(pg.mkPen('g'))

        self.DZplotvector = [0]*self.Size_Vector
        self.DZcurve = plotwidget.getPlotItem().plot(self.timevector,self.DZplotvector, name='z')
        self.DZcurve.setPen(pg.mkPen('b'))


        # ---------------------------------------------- #
        # ---------------------------------------------- #
        # initial time: this will be used to offset to time to 0 
        # instead of plotting with "real" time
        self.time0 = rospy.get_time()


        # Connecting slots to signals
        self.ctr_st.connect(self.ctr_st_update)

        # BUTTON TO SUBSCRIBE AND UNSUBSCRIBE To Controller state
        self._widget.ButtonCtrStateSubscribe.stateChanged.connect(self.SetCtrStateSubscription)         


        self._widget.GainsOption1.toggled.connect(self.DefaultOptions)
        self._widget.GainsOption2.toggled.connect(self.DefaultOptions)
        self._widget.GainsOption3.toggled.connect(self.DefaultOptions)

    def DefaultOptions(self):

        if self._widget.GainsOption1.isChecked():
            wn  = 1
            xsi = numpy.sqrt(2)/2
            ki  = 0.0

            wn_z  = 1
            xsi_z = numpy.sqrt(2)/2
            ki_z  = 0.0

            Throttle_neutral = 1400

        if self._widget.GainsOption2.isChecked():
            wn  = 1.7
            xsi = numpy.sqrt(2)/2
            ki  = 0.0

            wn_z  = 1
            xsi_z = numpy.sqrt(2)/2
            ki_z  = 0.1

            Throttle_neutral = 1400

        if self._widget.GainsOption3.isChecked():
            wn  = 1.7
            xsi = numpy.sqrt(2)/2
            ki  = 0.1

            wn_z  = 1
            xsi_z = numpy.sqrt(2)/2
            ki_z  = 0.1

            Throttle_neutral = 1400            


        kv   = 2.0*xsi*wn
        kp   = wn*wn

        kv_z   = 2.0*xsi_z*wn_z
        kp_z   = wn_z*wn_z


        # Default values for buttons: PID Controller
        self._widget.PgainXY_PID.setValue(kp)
        self._widget.DgainXY_PID.setValue(kv)
        self._widget.IgainXY_PID.setValue(ki)
        self._widget.PgainZ_PID.setValue(kp_z)
        self._widget.DgainZ_PID.setValue(kv_z)
        self._widget.IgainZ_PID.setValue(ki_z)
        self._widget.ThrottleNeutral_PID.setValue(Throttle_neutral)


    def SetCtrStateSubscription(self):


        if self._widget.ButtonCtrStateSubscribe.isChecked():

            self.sub = rospy.Subscriber(self.namespace +'ctr_state', Controller_State, self.callback)

            self.ReceiveControllerState(True,[0])

        else:
            # subscriber may not exist yet
            try:
                # unsubscribe to topic
                self.sub.unregister()
            except:
                pass

            self.ReceiveControllerState(False,[0])

            # clear all plots 
            self.DXcurve.setData([],[])
            self.DYcurve.setData([],[])
            self.DZcurve.setData([],[])

    def RESET_xy_PID(self):
        # IMPORTANT
        flag_PID_Controller = 1
        self.ReceiveControllerState(flag_PID_Controller,[1])
            
    def RESET_z_PID(self):
        # IMPORTANT
        flag_PID_Controller = 1
        self.ReceiveControllerState(flag_PID_Controller,[2])
 

    def callback(self,data):
     
        # data.d_est is tuple, cast it as as numpy array 
        disturbance = numpy.array(data.d_est)
        # time
        time = data.time

        data_recasted = numpy.concatenate([[time], disturbance])

        self.ctr_st.emit(data_recasted)       

    def ctr_st_update(self,data):

        self.timevector[:-1] = self.timevector[1:]
        self.timevector[-1]  = data[0] - self.time0      

        self.DXplotvector[:-1] = self.DXplotvector[1:]
        self.DXplotvector[-1]  = data[1]
        self.DXcurve.setData(self.timevector,self.DXplotvector)

        self.DYplotvector[:-1] = self.DYplotvector[1:]
        self.DYplotvector[-1]  = data[2]        
        self.DYcurve.setData(self.timevector,self.DYplotvector)

        self.DZplotvector[:-1] = self.DZplotvector[1:]
        self.DZplotvector[-1]  = data[3]
        self.DZcurve.setData(self.timevector,self.DZplotvector)
      

    #@Slot(bool)
    def SetController(self):

        try: 
            # time out of one second for waiting for service
            rospy.wait_for_service(self.namespace+'Controller_GUI',1.0)
            
            try:
                SettingController = rospy.ServiceProxy(self.namespace+'Controller_GUI', Controller_Srv)

                # self._widget.ControllerSelect.currentIndex() is the tab number
                # first tab is 0
                # second tab is 1
                # ... 

                if self._widget.ControllerSelect.currentIndex() == 0:
                    # PD controller selected
                   controller,parameters = self.PD_parameters()

                if self._widget.ControllerSelect.currentIndex() == 1:
                    # PID controller selected
                    controller,parameters = self.PID_parameters()

                if self._widget.ControllerSelect.currentIndex() == 2:
                    # ACRO controller selected
                    controller,parameters = self.ACRO_parameters()

                if self._widget.ControllerSelect.currentIndex() == 3:
                    # LOAD controller
                    controller,parameters = self.LOAD_parameters()

                reply = SettingController(controller,parameters)


                if reply.received == True:
                    # if controller receives message, we know it
                    # print('Trajectory has been set')
                    self._widget.Success.setChecked(True) 
                    self._widget.Failure.setChecked(False) 


            except rospy.ServiceException:
                self._widget.Success.setChecked(False) 
                self._widget.Failure.setChecked(True) 
                # print "Service call failed: %s"%e   
            
        except:
            self._widget.Success.setChecked(False) 
            self._widget.Failure.setChecked(True) 
            # print "Service not available ..."        
            pass     


    def PD_parameters(self):

        controller = 0
        P = self._widget.PgainXY_PID.value()
        D = self._widget.DgainXY_PID.value()
        P_z = self._widget.PgainZ_PID.value()
        D_z = self._widget.DgainZ_PID.value()
        # Throttle Neutral
        TN = self._widget.ThrottleNeutral.value()

        parameters = numpy.array([P,D,P_z,D_z,TN])

        return controller,parameters

    def PID_parameters(self):

        if self._widget.PID_OPTION.value() == 1:
            controller = 1
        if self._widget.PID_OPTION.value() == 2:
            controller = 2
        P = self._widget.PgainXY_PID.value()
        D = self._widget.DgainXY_PID.value()
        I = self._widget.IgainXY_PID.value()
        P_z = self._widget.PgainZ_PID.value()
        D_z = self._widget.DgainZ_PID.value()
        I_z = self._widget.IgainZ_PID.value()

        # Throttle Neutral
        TN = self._widget.ThrottleNeutral_PID.value()

        time = rospy.get_time()
        parameters = numpy.array([P,D,I,P_z,D_z,I_z,TN,time])

        return controller,parameters

    def ACRO_parameters(self):

        controller = 3
        parameters = None

        return controller,parameters

    def LOAD_parameters(self):

        controller = 5
        parameters = numpy.array([self._widget.ThrottleNeutralLoad.value()])

        return controller,parameters


    #@Slot(bool)
    def ReceiveControllerState(self,YesOrNo,parameter):
        
        # parameter = [0], means get controller state
        # parameter = [1], means reset controller state in x and y
        # parameter = [2], means reset controller state in z

        try:

            # time out of one second for waiting for service
            rospy.wait_for_service(self.namespace+'Controller_State_GUI',1.0)

            try:

                AskForControllerState = rospy.ServiceProxy(self.namespace+'Controller_State_GUI', Controller_Srv)

                reply = AskForControllerState(YesOrNo,parameter)

                if reply.received == True:
                    # if controller receives message, we know it
                    self._widget.Success.setChecked(True) 
                    self._widget.Failure.setChecked(False) 
                else:
                    # if controller does not receive message, we know it
                    self._widget.Success.setChecked(False) 
                    self._widget.Failure.setChecked(True) 

            except rospy.ServiceException:
                # print "Service call failed: %s"%e   
                self._widget.Success.setChecked(False) 
                self._widget.Failure.setChecked(True) 
            
        except: 
            # print "Service not available ..."        
            self._widget.Success.setChecked(False) 
            self._widget.Failure.setChecked(True)
            pass  


    def _parse_args(self, argv):

        parser = argparse.ArgumentParser(prog='saver', add_help=False)

        # args = parser.parse_args(argv)

        if argv:
            namespace = argv[0]
            return namespace            
        else:
            # is argv is empty return empty string
            return ""
    
    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog

    

