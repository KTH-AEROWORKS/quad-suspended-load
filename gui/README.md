GUI package
===========

RQT plugins used for an easier interface for the quad than the launchfiles and services command line one.

# Create a new plugin
 Create the description file of your plugin:
```XML
<library path="src">
  <class name="NAME IN RQT" type="gui.NAME_OF_MY_SCRIPT_IN_THE_SRC_FOLDER.NAME_OF_THE_CLASS" base_class_type="rqt_gui_py::Plugin">
    <description>
      DESCRIPTION IN RQT
    </description>
    <qtgui>
      <group>
        <label>Iris</label>
        <icon type="theme">folder</icon>
      </group>
      <label>NAME IN RQT MENU</label>
      <icon type="theme">applications-science</icon>
      <statustip>DECSCRIPTION IN RQT.</statustip>
    </qtgui>
  </class>
</library>
```
Import it into the [gui](scripts/gui) file inside the scripts/ folder:
```Python
from gui.NAME_OF_MY_SCRIPT_IN_THE_SRC_FOLDER import NAME_OF_THE_CLASS
```
Create your script in the [src/gui](src/gui) folder:
```Python
# A GUI plugin with which you can send listed instructions to the drone one at a time.
# You can also create, modify, save and load instruction lists with the plugin.

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




class NAME_OF_THE_CLASS(Plugin):

    # The variable launch is a signal specific to this class, emitted to make the plugin
    # send the next instruction on the current list.
    launch = pyqtSignal()
    
    def __init__(self, context):
        super(NAME_OF_THE_CLASS, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('NAME_OF_THE_CLASS')

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
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'MY_GUI.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('MY_GUIUi')
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

    
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

    

```
An example of rqt plugin can be found [here](http://wiki.ros.org/rqt/Tutorials/Create%20your%20new%20rqt%20plugin#Install_.26_Run_your_plugin).


# Installation
After having built the workspace, if the plugin doesn't show up in rqt, consider running with this command:
```Bash
run "rqt --force-discover"
```