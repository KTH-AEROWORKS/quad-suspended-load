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
from trajectory_node import TrajectoryNode
from mocap.msg import QuadPositionDerived
from controller.msg import Permission

import threading

class StepPlugin(Plugin):
    
    def __init__(self, context):
        super(StepPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('StepPlugin')

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
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'Step.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('StepPluginUi')
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)


        self.tg = TrajectoryGenerator()        

        self._widget.bUp.clicked.connect(self.Up)
        self._widget.bDown.clicked.connect(self.Down)
        self._widget.bLeft.clicked.connect(self.Left)
        self._widget.bRight.clicked.connect(self.Right)
        self._widget.bFront.clicked.connect(self.Front)
        self._widget.bBack.clicked.connect(self.Back)
        self._widget.bStart.clicked.connect(self.Start)

        self._widget.IrisInputBox.insertItems(0,['iris1','iris2','iris3','iris4'])

        self.point = []

    def Start(self):
        abspath = "/"+self._widget.IrisInputBox.currentText()+"/"
        self.pub = rospy.Publisher(abspath+'trajectory_gen/target',QuadPositionDerived, queue_size=10)
        self.security_pub = rospy.Publisher(abspath+'trajectory_gen/done', Permission, queue_size=10)
        rospy.Subscriber(abspath+'security_guard/data_forward',QuadPositionDerived,self.get_position)


    def get_position(self,data):
        if self.point == []:
            self.point = [data.x,data.y,data.z]
            utils.logwarn("Initial position: %s"%(str(self.point)))

    def Up(self):
        self.Goto([0.0,0.0,1.0])

    def Down(self):
        self.Goto([0.0,0.0,0.5])

    def Left(self):
        self.Goto([-1.0,0.0,0.5])

    def Right(self):
        self.Goto([1.0,0.0,0.5])

    def Back(self):
        self.Goto([0.0,-1.0,0.5])

    def Front(self):
        self.Goto([0.0,1.0,0.5])

    def Goto(self, dest):
        utils.logwarn(str(dest))
        outpos = [sum(x) for x in zip(self.point, dest)]
        utils.logwarn(str(outpos))
        outpos.append(0.0)
        outvelo = [0.0]*3
        outvelo.append(0.0)
        outacc = [0.0]*3
        outacc.append(0.0)
        outmsg = self.tg.get_message(outpos, outvelo, outacc)
        self.pub.publish(outmsg)
        self.security_pub.publish(False)


    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        instance_settings.set_value("irisindex", self._widget.IrisInputBox.currentIndex())

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        index = instance_settings.value("irisindex",0)
        self._widget.IrisInputBox.setCurrentIndex(int(index))

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog

    
