# Erik Berglund 2015
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

import os
import subprocess

import trajectory_generator
from trajectory import Trajectory
from trajectory_generato import TrajectoryGenerator
from trajectory_node import TrajectoryNode
from mocap.msg import QuadPositionDerived
from controller.msg import Permission
from straight_line_class import StraightLineGen
from dxfwrite import DXFEngine as dxfE


import dxfgrabber
import time
import threading
import json



class pointInputPlugin(Plugin):

    # The variable launch is a signal specific to this class, emitted to make the plugin
    # send the next instruction on the current list.
    launch = pyqtSignal()
    
    def __init__(self, context):
        super(pointInputPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('pointInputPlugin')

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
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'pointInput.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('pointInputUi')
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)


        # Setting default values of the class variables

        self.ID = 0
        self.index = 0
        self.State = QuadPositionDerived()
        self.Target = QuadPositionDerived()
        self.sub = ''
        self.targetsub = ''
        self.name = ''
        self.pwd = os.environ['PWD']
        self.pointlist = []
        self.filelist = os.listdir(self.pwd + '/src/kampala/gui/src/gui/DXFFiles')
        self.filelist2 = os.listdir(self.pwd + '/src/kampala/gui/src/gui/ActionFiles') 
        self._widget.IrisInputBox.insertItems(0,['iris1','iris2','iris3','iris4',"iris5"])
        self._widget.DXFInputBox.insertItems(0,self.filelist)
        self._widget.ActionInputBox.insertItems(0,self.filelist2)

        # Connecting slots to signals. If a QWidget A has a signal X, then self._widget.A.X.connect(fun)
        # means that the function fun will be called with any output of the signal as its argument each time the signal
        # X is emitted.
        self._widget.bStart.clicked.connect(self.Start)
        self._widget.AddPointButton.clicked.connect(self.AddPoint)
        self._widget.RemovePointButton.clicked.connect(self.RemovePoint)
        self._widget.LoadButton.clicked.connect(self.LoadDXF)
        self._widget.SaveButton.clicked.connect(self.SaveDXF)
        self._widget.AddWaitButton.clicked.connect(self.AddWait)
        self._widget.SaveActionsButton.clicked.connect(self.SaveActions)
        self._widget.LoadActionsButton.clicked.connect(self.LoadActions)
        self.launch.connect(self.manage_task)

        self._widget.XBox.setMinimum(-10.0)
        self._widget.XBox.setMaximum(10.0)
        self._widget.XBox.setSingleStep(0.1)
        self._widget.YBox.setMinimum(-10.0)
        self._widget.YBox.setMaximum(10.0)
        self._widget.YBox.setSingleStep(0.1)
        self._widget.ZBox.setMinimum(-10.0)
        self._widget.ZBox.setMaximum(10.0)
        self._widget.ZBox.setSingleStep(0.1)
    
    class EmitThread (threading.Thread):

        # A thread class used to emit the launch signal. Instances of it are created with the outer class
        # as the third argument.  

        def __init__(self,delay,launcher):
            threading.Thread.__init__(self)
            self.delay = delay
            self.launcher = launcher
        def run(self):

            # Waits for a set delay time, then emits the launch signal. The reason for doing this in a separate thread
            # is to not block the rest of the GUI for the delay time.

            time.sleep(self.delay)
            self.launcher.launch.emit()


    def execute(self,cmd):

        # Writes the command cmd to the corresponding pipefile of the selected drone, so that the terminal reading
        # from that pipefile can execute it.

        subprocess.Popen(["bash","-c","cd "+self.pwd+"/src/kampala/gui/scripts; echo "+cmd+" > pipefile" + self.name]) 


    def manage_task(self):

        # Is called each time the launch signal is emitted. Checks what kind of instruction that shall be carried out
        # and calls the corresponding functions.

        currentpoint = self.pointlist[self.index]
        if currentpoint[0] == 'go to: ':
            self.publish_trajectory_segment()
        if currentpoint[0] == 'wait: ':
            if self.index >= len(self.pointlist):
                pass
            else:
                self.index += 1
                self.EmitThread(currentpoint[1],self).start()


    def publish_trajectory_segment(self):

        # Launches the file line_userinput.launch, publishing a linear trajectory between the drone's current position
        # and its next target position according to its instructions.

        endpoint = self.pointlist[self.index][1]
        inputstring = "roslaunch scenarios line_userinput.launch ns:=%s xstart:=%f ystart:=%f zstart:=%f xdest:=%f ydest:=%f zdest:=%f" % (self.name,self.State.x,self.State.y,self.State.z,endpoint[0],endpoint[1],endpoint[2])
        self.execute(inputstring)
        
    def Start(self):
        
        # Resets the index keeping track of which instruction to execute, subscribes to the position and rcoutput topics
        # of the selected drone and emits the launch signal in a separate thread after waiting for 1 second.

        self.name = self._widget.IrisInputBox.currentText()
        self.index = 0
        self.ID = rospy.get_param(self.name + '/body_id')
        if self.sub != '':
            self.sub.unregister()
        self.sub = rospy.Subscriber('/body_data/id_' + str(self.ID),QuadPositionDerived,self.UpdateState)

        if self.targetsub != '':
            self.targetsub.unregister()
        self.targetsub = rospy.Subscriber('/' + self.name + '/trajectory_gen/target',QuadPositionDerived,self.target_track)

        self.EmitThread(1.0,self).start()
        
        
    def AddPoint(self):

        # Adds an instruction to go to the currently selected point to both the internal and the displayed list.

        addindex = self._widget.Pointlist.currentIndex()+1
        self.pointlist.insert(addindex,['go to: ',[round(self._widget.XBox.value(),3),round(self._widget.YBox.value(),3),round(self._widget.ZBox.value(),3)]])
        self._widget.Pointlist.insertItem(addindex,'go to: ' + str(self._widget.XBox.value()) + ',' +  str(self._widget.YBox.value()) + ',' + str(self._widget.ZBox.value()))
        self._widget.Pointlist.setCurrentIndex(addindex)

    def AddWait(self):

        # Add an instruction to wait at the last point for the currently selected waiting time to both the
        # internal and the displayed list.

        addindex = self._widget.Pointlist.currentIndex()+1
        self.pointlist.insert(addindex,['wait: ',self._widget.WaitBox.value()])
        self._widget.Pointlist.insertItem(addindex,'wait: ' + str(self._widget.WaitBox.value()) + 's')
        self._widget.Pointlist.setCurrentIndex(addindex)


    def RemovePoint(self):

        # Removes the currently displayed instruction from the list.

        rmindex = self._widget.Pointlist.currentIndex()
        if self.pointlist != []:
            del self.pointlist[rmindex]
        self._widget.Pointlist.removeItem(rmindex)   
       
    
    def UpdateState(self,data):

        # Called each time data from /irisX/body_data/id_x is recieved. Updates the internal tracking of
        # the drone's position.

        self.State = data

    def target_track(self,target):

        # Called each time data from /irisX/trajectory_gen/target is received. Checks if the currently published
        # target point is the endpoint of the current trajectory segment and if so, emits the launch signal in a
        # separate thread after waiting for 1 second.

        if self.index >= len(self.pointlist) - 1:
            pass
        else:
            targetpoint_rounded = [round(target.x,3),round(target.y,3),round(target.z,3)]
            if self.pointlist[self.index][0] == 'go to: ':
                endpoint = self.pointlist[self.index][1]
                if targetpoint_rounded == endpoint:
                    self.index += 1
                    self.EmitThread(1.0,self).start()

    def LoadDXF(self):

        # Loads the dxf-file selected in the DXFInputBox, checks all entities in the file and appends those with the 
        # dxftype 'POLYLINE' (a set of points interpolated by lines) to a list. The points of the first polyline are
        # then loaded into the instruction list. NOTE: ALL POINTS WITH Z-COORDINATES LESS THAN 0,5m WILL HAVE THEIR
        # Z-COORDINATE SET TO 0,5 WHEN LOADED!

        dxf = dxfgrabber.readfile(self.pwd + '/src/kampala/gui/src/gui/DXFFiles/' + self._widget.DXFInputBox.currentText())
        allpolylines = [entity for entity in dxf.entities.__iter__() if entity.dxftype == 'POLYLINE']
        if allpolylines != []:
            linepoints = allpolylines[0].points
            self._widget.Pointlist.clear()
            self.pointlist = []
            for point in linepoints:
                if point[2] < 0.5:
                    safepoint = [round(point[0],3),round(point[1],3),0.5]
                else:
                    safepoint = [round(point[0],3),round(point[1],3),round(point[2])]
                self.pointlist.append(['go to: ',safepoint])
                self._widget.Pointlist.insertItem(len(self.pointlist),'go to: ' + str(safepoint[0]) + ',' + str(safepoint[1]) + ',' + str(safepoint[2]))
        else:
            rospy.logwarn("no points in the selected DXFFile")



    def SaveDXF(self):

        # Creates a dxf-file with the name written in the FileInput widget, adding the extension .dxf if it's not
        # present. Then creates a polyline entity in that file resembling the planned trajectory of the list, ignoring 
        # all instructions in the list other than those with the 'go to: '-tag. Finally, the list of the ActionInputBox
        # widget is refreshed. 

        filename = self._widget.FileInput.text()
        l = len(filename)
        if l > 3:
            if filename[(l-4):l] != '.dxf':
                filename = filename + '.dxf'
        else:
            filename = filename + '.dxf'
        drawing = dxfE.drawing(self.pwd + '/src/kampala/gui/src/gui/DXFFiles/' + filename)

        trajectory_points = []
        for point in self.pointlist:
            if point[0] == 'go to: ':
                trajectory_points.append(point[1])

        drawing.add(dxfE.polyline(trajectory_points))
        drawing.save()
        self.filelist = os.listdir(self.pwd + '/src/kampala/gui/src/gui/DXFFiles')
        self._widget.DXFInputBox.clear()
        self._widget.DXFInputBox.insertItems(0,self.filelist)

    def SaveActions(self):

        # Extracts all instructions from the current list that are not about going to a point and adds them to a new list
        # together with the index of their position in the original list. Then creates a file with the name entered in the
        # ActionFileInput widget, adding the extension .txt if it's not present. Finally, the json module is used to 
        # write the nested list to the new file. 

        actions = []
        for i in range(len(self.pointlist)):
            instruction = self.pointlist[i] 
            if instruction[0] != 'go to: ':
                actions.append([instruction,i])
        filename = self._widget.ActionFileInput.text()
        l = len(filename)
        if l > 3:
            if filename[(l-4):l] != '.txt':
                filename = filename + '.txt'
        else:
            filename = filename + '.txt'
        with open(self.pwd + '/src/kampala/gui/src/gui/ActionFiles/' + filename, 'w') as f:
            json.dump(actions,f)
        self.filelist2 = os.listdir(self.pwd + '/src/kampala/gui/src/gui/ActionFiles')
        self._widget.ActionInputBox.clear()
        self._widget.ActionInputBox.insertItems(0,self.filelist2)

    #NOTE: Both save functions will overwrite any existing files with the same name as the one entered if there is any.
        
    def LoadActions(self):

        # Loads the txt-file selected in the ActionInputBox and loads the list of actions from it using the json module.
        # Then inserts each action in the instruction list at the positions determined by its associated index in the
        # file, as long as there is not an instruction for an identical instruction already there. The helper function
        # actioninsert is used to insert the action in the displayed instruction list.    

        actions = []
        filename = self._widget.ActionInputBox.currentText()
        with open(self.pwd + '/src/kampala/gui/src/gui/ActionFiles/' + filename, 'r') as f:
            actions = json.load(f)
        for action in actions:
            index = action[1]
            instruction = action[0]
            if index < len(self.pointlist):
                if self.pointlist[index] != instruction:
                    self.pointlist.insert(index,instruction)
                    self.actioninsert(instruction,index)
            else:
                self.pointlist.insert(index,instruction)
                self.actioninsert(instruction,index)
                    
    def actioninsert(self,instruction,index):
        if instruction[0] == 'wait: ':
            self._widget.Pointlist.insertItem(index, instruction[0] + str(instruction[1]) + 's')

        

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

    
