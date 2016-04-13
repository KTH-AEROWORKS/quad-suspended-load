import os
import rospy
import QtGui
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget
from PyQt4.QtCore import QObject, pyqtSignal

#from mocap.msg import Record

#import analysis
#import utils
import subprocess


import argparse


# import services defined in quad_control
# TWO services being imported
from quad_control.srv import *


class saverPlugin(Plugin):


    
    def __init__(self, context,namespace = None):

        # it is either "" or the input given at creation of plugin
        self.namespace = self._parse_args(context.argv())


        super(saverPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('saverPlugin')

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
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'saver.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('saverUi')
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

        # BUTTON TO START EXPERIMENT
        self._widget.ButtonSTART_SIM.stateChanged.connect(self.StartSimulatorService)

        # Button to Reset Simulator
        self._widget.ResetSimulator.clicked.connect(self.ResetSimulator)

        # BUTTON TO CONNECT TO QUALISYS
        self._widget.Qualisys.stateChanged.connect(self.QualisysConnect)

        # BUTTON TO CHANGE ID
        self._widget.changeID.clicked.connect(self.changeID)

        # Default Value for Id of Quad (as in QUALISYS computer)
        self._widget.ID.setValue(13)


    #@Slot(bool)
    def StartSimulatorService(self):
        try: 
            # time out of one second for waiting for service
            rospy.wait_for_service(self.namespace+'StartSimulator',1.0)
            
            try:
                AskForStart = rospy.ServiceProxy(self.namespace+'StartSimulator', StartSim)

                # if button is pressed save data
                if self._widget.ButtonSTART_SIM.isChecked():
                    # request controller to save data
                    reply = AskForStart(True)
                    if reply.Started == True:
                        # if controller receives message, we know it
                        # print('Started')
                        self._widget.SimulatorSuccess.setChecked(True) 
                        self._widget.SimulatorFailure.setChecked(False) 
                else:
                    # request simulator to freeze and restart
                    reply = AskForStart(False)
                    if  reply.Started == True:
                        # if controller receives message, we know it
                        # print('Stopped')
                        self._widget.SimulatorSuccess.setChecked(True) 
                        self._widget.SimulatorFailure.setChecked(False) 

            except rospy.ServiceException, e:
                # print "Service call failed: %s"%e   
                self._widget.SimulatorSuccess.setChecked(False) 
                self._widget.SimulatorFailure.setChecked(True) 
            
        except: 
            # print "Service not available ..."        
            self._widget.SimulatorSuccess.setChecked(False) 
            self._widget.SimulatorFailure.setChecked(True)
            pass  


    #@Slot(bool)
    def ResetSimulator(self):
        
        try: 
            # time out of one second for waiting for service
            rospy.wait_for_service(self.namespace+'ResetSimulator',1.0)
            
            try:
                AskForStart = rospy.ServiceProxy(self.namespace+'ResetSimulator', StartSim)

                # if button is pressed save data
                if self._widget.ButtonSTART_SIM.isChecked():
                    # request controller to save data
                    reply = AskForStart(True)
                    if reply.Started == True:
                        # if controller receives message, we know it
                        # print('Reseted')
                        self._widget.SimulatorSuccess.setChecked(True) 
                        self._widget.SimulatorFailure.setChecked(False) 
                    else:
                        self._widget.SimulatorSuccess.setChecked(False) 
                        self._widget.SimulatorFailure.setChecked(True)                         
                # else: do nothing

            except rospy.ServiceException, e:
                # print "Service call failed: %s"%e   
                self._widget.SimulatorSuccess.setChecked(False) 
                self._widget.SimulatorFailure.setChecked(True) 
            
        except: 
            # print "Service not available ..."        
            self._widget.SimulatorSuccess.setChecked(False) 
            self._widget.SimulatorFailure.setChecked(True)
            pass  


    #@Slot(bool)
    def QualisysConnect(self):
        
        try: 
            # time out of one second for waiting for service
            rospy.wait_for_service(self.namespace+'Mocap_Set_Id',1.0)
            
            try:
                AskMocap = rospy.ServiceProxy(self.namespace+'Mocap_Set_Id', Mocap_Id)

                if self._widget.Qualisys.isChecked() == True:
                    reply = AskMocap(True,self._widget.ID.value(),True)

                    if reply.success == True:
                        # if controller receives message, we know it
                        self._widget.QualisysSuccess.setChecked(True) 
                        self._widget.QualisysFailure.setChecked(False) 
                    if reply.exists == True:
                        self._widget.Exists.setChecked(True)
                        self._widget.ExistsNot.setChecked(False)
                    else:
                        self._widget.Exists.setChecked(False)
                        self._widget.ExistsNot.setChecked(True)
                else:
                    reply = AskMocap(False,self._widget.ID.value(),True)
                    
                    if reply.success == True:
                        # if controller receives message, we know it
                        self._widget.QualisysSuccess.setChecked(True) 
                        self._widget.QualisysFailure.setChecked(False) 

                    self._widget.Exists.setChecked(False)
                    self._widget.ExistsNot.setChecked(False)


            except rospy.ServiceException, e:
                # print "Service call failed: %s"%e   
                self._widget.QualisysSuccess.setChecked(False) 
                self._widget.QualisysFailure.setChecked(True) 
                self._widget.Exists.setChecked(False)
                self._widget.ExistsNot.setChecked(False)
            
        except: 
            # print "Service not available ..."        
            self._widget.QualisysSuccess.setChecked(False) 
            self._widget.QualisysFailure.setChecked(True)
            self._widget.Exists.setChecked(False)
            self._widget.ExistsNot.setChecked(False)            
            pass  


    #@Slot(bool)
    def changeID(self):
        
        try: 
            # time out of one second for waiting for service
            rospy.wait_for_service(self.namespace+'Mocap_Set_Id',1.0)
            
            try:
                AskMocap = rospy.ServiceProxy(self.namespace+'Mocap_Set_Id', Mocap_Id)

                if self._widget.Qualisys.isChecked() == True:
                    reply = AskMocap(True,self._widget.ID.value(),True)
                    print(reply)
                    if reply.success == True:
                        # if controller receives message, we know it
                        self._widget.QualisysSuccess.setChecked(True) 
                        self._widget.QualisysFailure.setChecked(False)

                    if reply.exists == True:
                        self._widget.Exists.setChecked(True)
                        self._widget.ExistsNot.setChecked(False)
                    else:
                        self._widget.Exists.setChecked(False)
                        self._widget.ExistsNot.setChecked(True)

            except rospy.ServiceException, e:
                # print "Service call failed: %s"%e   
                self._widget.QualisysSuccess.setChecked(False) 
                self._widget.QualisysFailure.setChecked(True) 
                self._widget.Exists.setChecked(False)
                self._widget.ExistsNot.setChecked(False)
            
        except: 
            # print "Service not available ..."        
            self._widget.QualisysSuccess.setChecked(False) 
            self._widget.QualisysFailure.setChecked(True)
            self._widget.Exists.setChecked(False)
            self._widget.ExistsNot.setChecked(False)            
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

    
