import os
import rospy
import QtGui
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget
from controller.msg import Permission
from std_srvs.srv import Empty
from std_msgs.msg import Int32
from gazebo_msgs.srv import DeleteModel
from mavros.msg import OverrideRCIn
from mavros.msg import BatteryStatus
import analysis
import utils

import os
import subprocess

class MyPlugin(Plugin):
    
    def __init__(self, context):
        self.pwd = os.environ['PWD']
        self.filelist = os.listdir(self.pwd+'/src/kampala/scenarios/launch')
        
        
        self.simulation = rospy.get_param('/simulation','false')
        self.land_pub = []

        self.controller_channel = []
        self.land_permission = Permission()
        self.controller_permission = Permission()
        self.lander_channel = []
        self.name = ''


        super(MyPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('MyPlugin')

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
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'MyPlugin.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('MyPluginUi')
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        

        self._widget.ConnectButton.clicked.connect(self.Connect)
        self._widget.LANDButton.clicked.connect(self.Land)
        self._widget.ArmButton.clicked.connect(self.Arm)
        self._widget.StartButton.clicked.connect(self.Start)
        self._widget.ParamButton.clicked.connect(self.Param)
        self._widget.TerminalButton.clicked.connect(self.Terminal)
        self._widget.StartInputField.returnPressed.connect(self.Autocomplete)
        self._widget.FileInputBox.currentIndexChanged.connect(self.FillIn)
        
        self._widget.IrisInputBox.insertItems(0,['iris1','iris2','iris3','iris4','iris5'])
        self._widget.FileInputBox.insertItems(0,self.filelist)
        if self.simulation:
            self._widget.TerminateButton.clicked.connect(self.Terminate)
        else:
            self._widget.TerminateButton.setEnabled(False)





    def execute(self,cmd):
        subprocess.Popen(["bash","-c","cd "+self.pwd+"/src/kampala/gui/scripts; echo "+cmd+" > pipefile" + self.name])

    def executeBlocking(self,cmd):
        os.system("bash -c 'cd "+self.pwd+"/src/kampala/gui/scripts; echo "+cmd+" > pipefile" + self.name + "'")

    def Terminal(self):
        self.name = self._widget.IrisInputBox.currentText()
        subprocess.Popen(["gnome-terminal", "-x" , "bash", "-c", 'source '+self.pwd+'/devel/setup.bash;roscd gui/scripts;./term-pipe-r.sh pipefile' + self.name + ';bash'])
    
    def Terminate(self):
        inputstring = 'rosnode kill `rosnode list | grep ' + self.name + '`'
        self.execute(inputstring)

        try:
            delete_model = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)
            delete_model(self.name)
        except rospy.ServiceException as exc:
            utils.loginfo('Failed to delete model ' + str(exc))


    def Param(self):
        self.name = self._widget.IrisInputBox.currentText()
        inputstring = "roslaunch scenarios %s.launch simulation:=%s" % (self.name,self.simulation)
        self.executeBlocking(inputstring)
        #A sleep for 0.2 seconds that allows the file to be properly launched before you try to load it. 
        rospy.sleep(2.)
        
        try: 
            params_load = rospy.ServiceProxy("/%s/blender/update_parameters"%(self.name), Empty)
            params_load_PID = rospy.ServiceProxy("/%s/PID_controller/update_parameters"%(self.name), Empty)
            obstacle_params_load = rospy.ServiceProxy("/%s/obstacle_avoidance/update_parameters"%(self.name), Empty)
            params_load()
            params_load_PID()
        except rospy.ServiceException as exc:
            utils.loginfo("PID not reachable " + str(exc))


    def Connect(self):
        self.rpi = utils.Get_Parameter('/'+self.name+'/rpi','false')            # raspberry pi parameter for the connection (connect.launch)
        inputstring = "roslaunch scenarios connect.launch simulation:=%s ns:=%s rpi:=%s" % (self.simulation,self.name,self.rpi)
        self.execute(inputstring)
        
        self.land_pub = rospy.Publisher('/%s/gui/land'%(self.name),Permission,queue_size=10)
        self.lander_channel = rospy.Publisher('/%s/security_guard/lander'%(self.name),Permission,queue_size=10)
        self.controller_channel = rospy.Publisher('/%s/security_guard/controller'%(self.name),Int32,queue_size=10)
        


    def Land(self):
      #self.lander_channel.publish(Permission(True))
      self.land_pub.publish(Permission(True))
      #land = rospy.ServiceProxy("/%s/SecurityGuard/land"%(self.name), Empty)
      #land()
      


    def Start(self):
        inputstring = "roslaunch scenarios %s ns:=%s" % (self._widget.StartInputField.text(),self.name)
        self.execute(inputstring)

    def Arm(self):
        self.land_pub.publish(Permission(False))
        inputstring = "roslaunch scenarios iris_nodes.launch ns:=%s simulation:=%s" % (self.name,self.simulation)
        self.execute(inputstring)

    def Autocomplete(self):
        exists = False
        unique = True
        completed_text=""
        text = self._widget.StartInputField.text()
        textlength = len(text)
        for filename in self.filelist:
            if text == filename[0:textlength]:
                exists = True
                if completed_text != "":
                    unique = False
                completed_text = filename
        if exists and unique:
            self._widget.StartInputField.setText(completed_text)

    def FillIn(self):
        self._widget.StartInputField.setText(self._widget.FileInputBox.currentText())






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
