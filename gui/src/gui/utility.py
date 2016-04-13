import os
import rospy
import QtGui
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget
from controller.msg import Permission
from std_srvs.srv import Empty
from PyQt4.QtCore import QObject, pyqtSignal

import analysis
import utils
import subprocess

import trajectory_generator
from trajectory import Trajectory
from trajectory_generato import TrajectoryGenerator
from trajectory_node import TrajectoryNode
from mocap.msg import QuadPositionDerived
from controller.msg import Permission
from straight_line_class import StraightLineGen






class utilityPlugin(Plugin):

    launch = pyqtSignal()
    
    def __init__(self, context):
        super(utilityPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('utilityPlugin')

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
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'utility.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('utilityUi')
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)


        
        self.ID = 0
        
        self.State = QuadPositionDerived()
        self.Target = QuadPositionDerived()
        self.sub = ''
        self.name = ''
        self.pwd = os.environ['PWD']
        
        

               

        self._widget.IrisInputBox.insertItems(0,['iris1','iris2','iris3','iris4',"iris5"])
        self._widget.bStart.clicked.connect(self.Start)
        self._widget.GravityCancelButton.clicked.connect(self.adjust_gravity_cancel)
        
        
        self._widget.XBox.setMinimum(-10.0)
        self._widget.XBox.setMaximum(10.0)
        self._widget.XBox.setSingleStep(0.1)
        self._widget.YBox.setMinimum(-10.0)
        self._widget.YBox.setMaximum(10.0)
        self._widget.YBox.setSingleStep(0.1)
        self._widget.ZBox.setMinimum(-10.0)
        self._widget.ZBox.setMaximum(10.0)
        self._widget.ZBox.setSingleStep(0.1)
        self._widget.GravitySpinBox.setMaximum(1800)
        self._widget.GravitySpinBox.setMinimum(1200)
        self._widget.GravitySpinBox.setSingleStep(10)
        self._widget.GravitySpinBox.setValue(1500)
    
   


    def execute(self,cmd):
        subprocess.Popen(["bash","-c","cd "+self.pwd+"/src/kampala/gui/scripts; echo "+cmd+" > pipefile" + self.name]) 

    def publish_trajectory_segment(self):
        inputstring = "roslaunch scenarios line_userinput.launch ns:=%s xstart:=%f ystart:=%f zstart:=%f xdest:=%f ydest:=%f zdest:=%f" % (self.name,self.State.x,self.State.y,self.State.z,self._widget.XBox.value(),self._widget.YBox.value(),self._widget.ZBox.value())
        self.execute(inputstring)

    def adjust_gravity_cancel(self):
        self.name = self._widget.IrisInputBox.currentText()
        rospy.set_param('/' + self.name + '/CONTROL_CANCEL_GRAVITY',int(self._widget.GravitySpinBox.value()))
        try: 
            params_load = rospy.ServiceProxy("/%s/blender/update_parameters"%(self.name), Empty)
            params_load_PID = rospy.ServiceProxy("/%s/PID_controller/update_parameters"%(self.name), Empty)
            params_load()
            params_load_PID()
        except rospy.ServiceException as exc:
            utils.loginfo("PID not reachable " + str(exc))

    def Start(self):
        self.name = self._widget.IrisInputBox.currentText()
        self.ID = rospy.get_param(self.name + '/body_id')
        if self.sub != '':
            self.sub.unregister()
        self.sub = rospy.Subscriber('/body_data/id_' + str(self.ID),QuadPositionDerived,self.UpdateState)
        self.publish_trajectory_segment()

    def UpdateState(self,data):
        self.State = data

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

    
