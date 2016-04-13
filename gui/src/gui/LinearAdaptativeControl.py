import os
import rospy
import QtGui
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget
from controller.msg import Permission
from std_srvs.srv import Empty

import analysis
import utils

import os
import subprocess

import trajectory_generator
from trajectory import Trajectory
from trajectory_generato import TrajectoryGenerator
from Trajectory_node import TrajectoryNode
from mocap.msg import QuadPositionDerived
from controller.msg import Permission
from straight_line_class import StraightLineGen
from linear_adaptative_control import Point
from controller.srv import PlotLAC 


import threading

class LinearAdaptativeControl(Plugin):
    
    def __init__(self, context):
        super(LinearAdaptativeControl, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('LinearAdaptativeControl')

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
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'LinearAdaptativeControl.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('LinearAdaptativeControlUi')
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)


        self._widget.IrisInputBox.insertItems(0,['iris1','iris2','iris3','iris4'])
        self._widget.cVariable.insertItems(0,Point.params_name)

        self._widget.bLoad.clicked.connect(self.call_load)
        self._widget.bAddPoint.clicked.connect(self.call_add_point)
        self._widget.bUpdate.clicked.connect(self.call_update_controller)
        self._widget.bSave.clicked.connect(self.call_save)
        self._widget.bPlot.clicked.connect(self.call_plot)
        self._widget.bPrint.clicked.connect(self.call_print)


    def call_load(self):
        self.name = self._widget.IrisInputBox.currentText()
        add_point = rospy.ServiceProxy("/%s/LinearAC/load"%(self.name), Empty)

    def call_add_point(self):
        self.name = self._widget.IrisInputBox.currentText()
        add_point = rospy.ServiceProxy("/%s/LinearAC/add_point"%(self.name), Empty)
        add_point()

    def call_update_controller(self):
        self.name = self._widget.IrisInputBox.currentText()
        update_controller = rospy.ServiceProxy("/%s/LinearAC/update_controller"%(self.name), Empty)
        update_controller()

    def call_save(self):
        self.name = self._widget.IrisInputBox.currentText()
        save = rospy.ServiceProxy("/%s/LinearAC/save"%(self.name), Empty)
        save()

    def call_plot(self):
        self.name = self._widget.IrisInputBox.currentText()
        variable = self._widget.cVariable.currentText()
        pr = rospy.ServiceProxy("/%s/LinearAC/plot"%(self.name), PlotLAC)
        pr(0,variable)

    def call_print(self):
        self.name = self._widget.IrisInputBox.currentText()
        pr = rospy.ServiceProxy("/%s/LinearAC/print"%(self.name), Empty)
        pr()

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

    
