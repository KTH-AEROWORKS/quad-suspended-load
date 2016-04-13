# Erik Berglund 2015
# A GUI plugin to monitor the channel outputs and the battery voltage, displaying the values
# with text and with bars.

import os
import rospy
import QtGui
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget
from mavros.msg import OverrideRCIn
from mavros.msg import BatteryStatus
from std_srvs.srv import Empty

import analysis
import utils

import subprocess

class RCDisplayPlugin(Plugin):
    
    def __init__(self, context):
        

        super(RCDisplayPlugin, self).__init__(context)
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
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'RCDisplay.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('RCDisplayUi')
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        # Setting maximum and minimum on the bars:
        self._widget.channel1bar.setMinimum(0)
        self._widget.channel1bar.setMaximum(2000)
        self._widget.channel2bar.setMinimum(0)
        self._widget.channel2bar.setMaximum(2000)
        self._widget.channel3bar.setMinimum(0)
        self._widget.channel3bar.setMaximum(2000)
        self._widget.channel4bar.setMinimum(0)
        self._widget.channel4bar.setMaximum(2000)
        self._widget.channel5bar.setMinimum(0)
        self._widget.channel5bar.setMaximum(2000)
        self._widget.channel6bar.setMinimum(0)
        self._widget.channel6bar.setMaximum(2000)
        self._widget.channel7bar.setMinimum(0)
        self._widget.channel7bar.setMaximum(2000)
        self._widget.channel8bar.setMinimum(0)
        self._widget.channel8bar.setMaximum(2000)
        self._widget.Batterybar.setMinimum(0)

        # The batterybar can only handle integer values, so its maximum is set to 10000 to get a good resolution.
        self._widget.Batterybar.setMaximum(10000)

        # Connecting slots to signals. Each time channelXdisplay emits the signal textChanged, 
        # the function self.change_barX will be called.
        self._widget.channel1display.textChanged.connect(self.change_bar1)
        self._widget.channel2display.textChanged.connect(self.change_bar2)
        self._widget.channel3display.textChanged.connect(self.change_bar3)
        self._widget.channel4display.textChanged.connect(self.change_bar4)
        self._widget.channel5display.textChanged.connect(self.change_bar5)
        self._widget.channel6display.textChanged.connect(self.change_bar6)
        self._widget.channel7display.textChanged.connect(self.change_bar7)
        self._widget.channel8display.textChanged.connect(self.change_bar8)
        self._widget.Batterydisplay.textChanged.connect(self.change_Batterybar)
        self._widget.ListenButton.clicked.connect(self.Listen)

        # Setting other variables

        self._widget.IrisInputBox.insertItems(0,['iris1','iris2','iris3','iris4',"iris5"])
        self.batterysub = ''
        self.sub = ''
        

    def Listen(self):

        # Unsubscribes from any previous channels and subscribes to the monitored channels of the selected drone.

        if self.sub != '':
            self.sub.unregister()
        self.sub = rospy.Subscriber('/' + self._widget.IrisInputBox.currentText() + '/mavros/rc/override', OverrideRCIn, self.callback)    
        if self.batterysub != '':
            self.batterysub.unregister()
        self.batterysub = rospy.Subscriber('/' + self._widget.IrisInputBox.currentText() + '/mavros/battery', BatteryStatus, self.batterycallback)
#
    def callback(self,data):

        # Called each time data from /irisX/mavros/rc/override is recieved, 
        # updates the displayed outputs on the channels according to that data. 
        # self.batterycallback does the corresponding thing for the battery.

        self._widget.channel1display.setText(str(data.channels[0]))
        self._widget.channel2display.setText(str(data.channels[1]))
        self._widget.channel3display.setText(str(data.channels[2]))
        self._widget.channel4display.setText(str(data.channels[3]))
        self._widget.channel5display.setText(str(data.channels[4]))
        self._widget.channel6display.setText(str(data.channels[5]))
        self._widget.channel7display.setText(str(data.channels[6]))
        self._widget.channel8display.setText(str(data.channels[7]))

    def batterycallback(self,data):

        self._widget.Batterydisplay.setText(str(data.voltage))
    
    # The following functions update the bars when the text on the corresponding display is changed.
    def change_bar1(self,text):
        self._widget.channel1bar.setValue(int(text))
    def change_bar2(self,text):
        self._widget.channel2bar.setValue(int(text))
    def change_bar3(self,text):
        self._widget.channel3bar.setValue(int(text)) 
    def change_bar4(self,text):
        self._widget.channel4bar.setValue(int(text))
    def change_bar5(self,text):
        self._widget.channel5bar.setValue(int(text))
    def change_bar6(self,text):
        self._widget.channel6bar.setValue(int(text))
    def change_bar7(self,text):
        self._widget.channel7bar.setValue(int(text))
    def change_bar8(self,text):
        self._widget.channel8bar.setValue(int(text))

    def change_Batterybar(self,text):

        # The Batterybar's function setValue first rounds its argument to an integer,
        # therefore scaling of the input is needed to get a good resolution. 

        self._widget.Batterybar.setValue(float(text)*10000/12.5)
        self._widget.Batterybar.setFormat(text+'/12.5')



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
