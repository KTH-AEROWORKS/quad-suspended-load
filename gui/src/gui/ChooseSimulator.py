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
# SERVICE BEING USED: Simulator_srv
from quad_control.srv import *


import argparse


class ChooseSimulatorPlugin(Plugin):


    def __init__(self, context,namespace = None):

        # it is either "" or the input given at creation of plugin
        self.namespace = self._parse_args(context.argv())


        super(ChooseSimulatorPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('ChooseSimulatorPlugin')

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
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'ChooseSimulator.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('ChooseSimulatorUi')
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

        # BUTTON TO SET DESIRED SIMULATOR
        self._widget.SetSimulatorButton.clicked.connect(self.SetSimulator)


        # Default values for buttons: Fully Actuated
        self._widget.Mass.setValue(1.442)
        self._widget.AttitudeGain.setValue(0.5)
        self._widget.ThrottleNeutral.setValue(1484)

        # Default values for buttons: Non Fully Actuated
        self._widget.Mass_2.setValue(1.442)
        self._widget.ThrottleNeutral_2.setValue(1484)


    #@Slot(bool)
    def SetSimulator(self):

        try: 
            # time out of one second for waiting for service
            rospy.wait_for_service(self.namespace+'Simulator_GUI',1.0)

            try:

                SettingSimulator = rospy.ServiceProxy(self.namespace+'Simulator_GUI', Simulator_Srv)

                # self._widget.SimulatorSelect.currentIndex() is the tab number
                # first tab is 0
                # second tab is 1
                # ... 

                if self._widget.SimulatorSelect.currentIndex() == 0:
                    # Fully actuated
                   simulator,parameters = self.FullyActuated_parameters()

                if self._widget.SimulatorSelect.currentIndex() == 1:
                    # Non Fully selected
                    simulator,parameters = self.NonFullyActuated_parameters()

                reply = SettingSimulator(simulator,parameters)

                if reply.received == True:
                    # if controller receives message, we know it
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


    def FullyActuated_parameters(self):

        simulator = 1
        m   = self._widget.Mass.value()
        TN  = self._widget.ThrottleNeutral.value()
        ktt = self._widget.AttitudeGain.value()

        parameters = numpy.array([m,TN,ktt])

        return simulator,parameters

    def NonFullyActuated_parameters(self):

        simulator = 2
        m   = self._widget.Mass_2.value()
        TN  = self._widget.ThrottleNeutral_2.value()

        parameters = numpy.array([m,TN])

        return simulator,parameters


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

    

