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
# from mavros.msg import OverrideRCIn
# from mocap.msg import QuadPositionDerived

from quad_control.msg import quad_state_and_cmd
from quad_control.msg import quad_state
from mavros_msgs.msg import State


# import services defined in quad_control
# SERVICE BEING USED: TrajDes_GUI
from quad_control.srv import *

from std_srvs.srv import Empty

import argparse




class TrajectorySelectionPlugin(Plugin):


    def __init__(self, context,namespace = None):

        # it is either "" or the input given at creation of plugin
        self.namespace = self._parse_args(context.argv())


        super(TrajectorySelectionPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('TrajectorySelectionPlugin')

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
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'TrajectorySelection.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('TrajectorySelectionUi')
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

        # BUTTON TO SET DESIRED TRAJECTORY

        self.CurrentX = 0
        self.CurrentY = 0
        self.CurrentZ = 0

        self.InitialX = 0
        self.InitialY = 0
        self.InitialZ = 0

        self.FlagFlying = False

        self.subPosition = rospy.Subscriber(self.namespace+'quad_state_and_cmd', quad_state_and_cmd, self.GetQuadPosition)
        self.subPosition = rospy.Subscriber(self.namespace+'mavros/set_mode', State, self.CheckMode)

        self._widget.SaveHome.clicked.connect(self.SaveHome)
        self._widget.TakeOff.clicked.connect(self.TakeOff)
        self._widget.GoUp.clicked.connect(self.GoUp)
        self._widget.GoDown.clicked.connect(self.GoDown)
        self._widget.GoHome.clicked.connect(self.GoHome)

        self._widget.SetTrajectory.clicked.connect(self.SetTrajectory)


        self._widget.DefaultOption1.toggled.connect(self.DefaultOptions)
        self._widget.DefaultOption2.toggled.connect(self.DefaultOptions)
        self._widget.DefaultOption3.toggled.connect(self.DefaultOptions)

        # Planner buttons
        self._widget.planner_start_button.clicked.connect(self.planner_start)
        self._widget.planner_stop_button.clicked.connect(self.planner_stop)


    def DefaultOptions(self):

        # radius, period and height
        if self._widget.DefaultOption1.isChecked():
            r  = 0
            w  = 0
            z  = 0.6

        if self._widget.DefaultOption2.isChecked():
            r  = 0.5
            w  = 0.1
            z  = 0.6

        if self._widget.DefaultOption3.isChecked():
            r  = 1
            w  = 0.1
            z  = 0.6        
        if self._widget.DefaultOption4.isChecked():
            r  = 0.5
            w  = 0.2
            z  = 0.6                   

        # Default values for buttons
        self._widget.box_radius_circle.setValue(r)
        self._widget.box_omega_circle.setValue(w)
        self._widget.box_z_circle.setValue(z)

    def TakeOff(self):
    
        if self.FlagFlying == False:

            self.FlagFlying = True
            self.SaveHome()
            self.GoUp()

        else:
            rospy.logwarn('Already Flying')
            pass


    def GoUp(self):
    
        x  = self.CurrentX
        y  = self.CurrentY
        z  = self.CurrentZ+1

        self._widget.box_x.setValue(x)
        self._widget.box_y.setValue(y)
        self._widget.box_z.setValue(z)

        self.SetTrajectory()

    def GoDown(self):
        
        distZ = self.CurrentZ - self.InitialZ
        if distZ>1.5:
            d = 1
        else:
            rospy.logwarn('Current altitude (respect to initial): ', distZ)
            d = 0

        x  = self.CurrentX
        y  = self.CurrentY
        z  = self.CurrentZ-d
            
        self._widget.box_x.setValue(x)
        self._widget.box_y.setValue(y)
        self._widget.box_z.setValue(z)

        self.SetTrajectory()

    def SaveHome(self):

        self.initialX  = self.CurrentX
        self.initialY  = self.CurrentY
        self.initialZ  = self.CurrentZ

    def GoHome(self):

        x  = self.initialX
        y  = self.initialY
        z  = self.CurrentZ

        self._widget.box_x.setValue(x)
        self._widget.box_y.setValue(y)
        self._widget.box_z.setValue(z)

        self.SetTrajectory()


    def GetQuadPosition(self,data):
        self.CurrentX = data.x
        self.CurrentY = data.y
        self.CurrentZ = data.z

    def CheckMode(self,data):
        mode = data.mode
        if mode == 'LAND':
            self.FlagFlying = False
        else:
            pass

    #@Slot(bool)
    def SetTrajectory(self):

        # print(vars(self._widget))

        try: 
            # time out of one second for waiting for service
            rospy.wait_for_service(self.namespace+'TrajDes_GUI',1.0)
            
            try:
                SettingTrajectory = rospy.ServiceProxy(self.namespace+'TrajDes_GUI', TrajDes_Srv)

                # self._widget.TrajSelect.currentIndex() is the tab number
                # first tab is 0
                # second tab is 1
                # ... 

                if self._widget.TrajSelect.currentIndex() == 0:
                   traj,offset,rotation,parameters = self.fixed_point()

                if self._widget.TrajSelect.currentIndex() == 1:
                    traj,offset,rotation,parameters = self.circle()

                reply = SettingTrajectory(traj,offset,rotation,parameters)


                if reply.received == True:
                    # if controller receives message, we know it
                    # print('Trajectory has been set')
                    self._widget.Success.setChecked(True) 
                    self._widget.Failure.setChecked(False) 


            except rospy.ServiceException, e:
                self._widget.Success.setChecked(False) 
                self._widget.Failure.setChecked(True) 
                # print "Service call failed: %s"%e   
            
        except:
            self._widget.Success.setChecked(False) 
            self._widget.Failure.setChecked(True) 
            # print "Service not available ..."        
            pass     


    def fixed_point(self):

        traj = 0
        
        x = self._widget.box_x.value()
        y = self._widget.box_y.value()
        z = self._widget.box_z.value()
        offset = numpy.array([x,y,z])
        
        rotation = numpy.array([0.0,0.0,0.0])

        parameters = None

        return traj,offset,rotation,parameters        

    def circle(self):

        traj = 1
        x = self._widget.box_x_circle.value()
        y = self._widget.box_y_circle.value()
        z = self._widget.box_z_circle.value()
        offset = numpy.array([x,y,z])

        phi   = 0.0
        theta = self._widget.box_theta_circle.value()
        psi   = self._widget.box_psi_circle.value()
        rotation = numpy.array([phi,theta,psi])

        r = self._widget.box_radius_circle.value()
        # from revolutions per second to rad per second
        w = self._widget.box_omega_circle.value()*2*3.14
        parameters = numpy.array([r,w])

        return traj,offset,rotation,parameters


    def _parse_args(self, argv):

        parser = argparse.ArgumentParser(prog='saver', add_help=False)

        # args = parser.parse_args(argv)

        if argv:
            namespace = argv[0]
            return namespace            
        else:
            # is argv is empty return empty string
            return ""

    # start planned trajectory
    def planner_start(self):
        try:
            rospy.wait_for_service('planner_start',1)
            start = rospy.ServiceProxy('planner_start', PlannerStart)

            start(self._widget.planner_edit.toPlainText(), self.namespace)
        except rospy.ROSException, rospy.ServiceException:
            rospy.logwarn('could not start planned trajectory')

    # stop planned trajectory
    def planner_stop(self):
        try:
            rospy.wait_for_service('planner_stop',1)
            stop = rospy.ServiceProxy('planner_stop',Empty)

            stop()
        except rospy.ROSException, rospy.ServiceException:
            rospy.logwarn('could not start planned trajectory')
    
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

    
