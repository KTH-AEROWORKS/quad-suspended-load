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


# import services defined in quad_control
from quad_control.srv import *


import argparse



class positionPlotPlugin(Plugin):

    # for time vector
    Time = pyqtSignal(float)

    # for x,y,z vectors
    Xon = pyqtSignal(float)
    Yon = pyqtSignal(float)
    Zon = pyqtSignal(float)
    
    # for desired x,y,z vectors
    Xdon = pyqtSignal(float)
    Ydon = pyqtSignal(float)
    Zdon = pyqtSignal(float)

    # for vx,vy,vz vectors
    Xvelon = pyqtSignal(float)
    Yvelon = pyqtSignal(float)
    Zvelon = pyqtSignal(float)

    # for desired vx,vy,vz vectors
    Xdvelon = pyqtSignal(float)
    Ydvelon = pyqtSignal(float)
    Zdvelon = pyqtSignal(float)

    # for vx,vy,vz vectors
    Xvelon = pyqtSignal(float)
    Yvelon = pyqtSignal(float)
    Zvelon = pyqtSignal(float)

    # for desired vx,vy,vz vectors
    Xdvelon = pyqtSignal(float)
    Ydvelon = pyqtSignal(float)
    Zdvelon = pyqtSignal(float)  

    # for euler angles
    Roll_on = pyqtSignal(float)
    Pitch_on = pyqtSignal(float)
    Yaw_on = pyqtSignal(float)
    
    # for desired euler angles
    Rolld_on = pyqtSignal(float)
    Pitchd_on = pyqtSignal(float)
    Yawd_on = pyqtSignal(float)     

    # for channels vectors
    Ch1on = pyqtSignal(float)
    Ch2on = pyqtSignal(float)
    Ch3on = pyqtSignal(float)    
    Ch4on = pyqtSignal(float)      


    # ---------------------------------------------- #
    # ---------------------------------------------- #
    # Necessary constants

    # size of vectors: INTEGER
    Size_Vector = 100
    # Period of complete Time Window: SECONDS
    Period_Window = 10.0
    # Period for data saving: it depends of size of vectors and the time window period
    Period_Data_Saving = Period_Window/Size_Vector
    # Period for plotting: be wise, when plotting to many things
    Period_Plot = 1.0
    # Need to know frequency of messages we are subscribing to 
    Frequency_Subscription = 10




    def __init__(self, context,namespace = None):

        # it is either "" or the input given at creation of plugin
        self.namespace = self._parse_args(context.argv())

        super(positionPlotPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('positionPlotPlugin')

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
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'positionPlot.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('positionPlotUi')
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

        # window for positions
        plotwidget = pg.PlotWidget()
        plotwidget.getPlotItem().addLegend()
        plotwidget.setYRange(-2.5,2.5)      


        # window for velocities
        velplotwidget = pg.PlotWidget()
        velplotwidget.getPlotItem().addLegend()
        velplotwidget.setYRange(-2.5,2.5)   

        # window for velocities
        Anglesplotwidget = pg.PlotWidget()
        Anglesplotwidget.getPlotItem().addLegend()
        Anglesplotwidget.setYRange(-30,30)        

        # window for channels
        channelplotwidget = pg.PlotWidget()
        channelplotwidget.getPlotItem().addLegend()
        channelplotwidget.setYRange(1000,2000)


        # ---------------------------------------------- #
        # ---------------------------------------------- #        
        
        layout        = QtGui.QGridLayout()
        vellayout     = QtGui.QGridLayout()
        Angleslayout  = QtGui.QGridLayout()
        channellayout = QtGui.QGridLayout()

        # increase or reset counter plot data saving
        layout.addWidget(plotwidget)
        vellayout.addWidget(velplotwidget)
        Angleslayout.addWidget(Anglesplotwidget)
        channellayout.addWidget(channelplotwidget)

        # ---------------------------------------------- #
        # ---------------------------------------------- #  

        # labels for window with positions
        plotwidget.getPlotItem().setLabel('left','position','m')
        plotwidget.getPlotItem().setLabel('bottom','time','s')

        # labels for window with velocities
        velplotwidget.getPlotItem().setLabel('left','speed','m/s')
        velplotwidget.getPlotItem().setLabel('bottom','time','s')

        # labels for window with angles
        Anglesplotwidget.getPlotItem().setLabel('left','Angles','Degrees')
        Anglesplotwidget.getPlotItem().setLabel('bottom','time','s')

        # labels for window with channels
        channelplotwidget.getPlotItem().setLabel('left','cmd','PWM(?)')
        channelplotwidget.getPlotItem().setLabel('bottom','time','s')


        # complete putting labels
        self._widget.frame.setLayout(layout)
        self._widget.frame_2.setLayout(vellayout)
        self._widget.frame_3.setLayout(Angleslayout)
        self._widget.frame_4.setLayout(channellayout)


        # ---------------------------------------------- #
        # ---------------------------------------------- #          

        # time vector
        self.timevector = [0]*self.Size_Vector

        
        #Setting variables for each coordinate and channel

        # ---------------------------------------------- #  
        # positions x,y,z
        self.Xplotvector = [0]*self.Size_Vector
        self.Xcurve = plotwidget.getPlotItem().plot(self.timevector,self.Xplotvector, name='x')
        self.Xcurve.setPen(pg.mkPen('r'))
        
        self.Yplotvector = [0]*self.Size_Vector
        self.Ycurve = plotwidget.getPlotItem().plot(self.timevector,self.Yplotvector, name='y')
        self.Ycurve.setPen(pg.mkPen('g'))

        self.Zplotvector = [0]*self.Size_Vector
        self.Zcurve = plotwidget.getPlotItem().plot(self.timevector,self.Zplotvector, name='z')
        self.Zcurve.setPen(pg.mkPen('b'))


        # ---------------------------------------------- #  
        # desired positions x,y,z
        self.Xdplotvector = [0]*self.Size_Vector
        self.Xdcurve = plotwidget.getPlotItem().plot(self.timevector,self.Xdplotvector, name='x')
        self.Xdcurve.setPen(pg.mkPen('r', style=PyQt4.QtCore.Qt.DashLine))
        
        self.Ydplotvector = [0]*self.Size_Vector
        self.Ydcurve = plotwidget.getPlotItem().plot(self.timevector,self.Ydplotvector, name='y')
        self.Ydcurve.setPen(pg.mkPen('g', style=PyQt4.QtCore.Qt.DashLine))

        self.Zdplotvector = [0]*self.Size_Vector
        self.Zdcurve = plotwidget.getPlotItem().plot(self.timevector,self.Zdplotvector, name='z')
        self.Zdcurve.setPen(pg.mkPen('b', style=PyQt4.QtCore.Qt.DashLine))        


        # ---------------------------------------------- #  
        # velocities x,y,z
        self.Xvelplotvector = [0]*self.Size_Vector
        self.Xvelcurve = velplotwidget.getPlotItem().plot(self.timevector,self.Xvelplotvector, name='v<sub>x</sub>')
        self.Xvelcurve.setPen(pg.mkPen('r'))
        
        self.Yvelplotvector = [0]*self.Size_Vector
        self.Yvelcurve = velplotwidget.getPlotItem().plot(self.timevector,self.Yvelplotvector, name='v<sub>y</sub>')
        self.Yvelcurve.setPen(pg.mkPen('g'))

        self.Zvelplotvector = [0]*self.Size_Vector
        self.Zvelcurve = velplotwidget.getPlotItem().plot(self.timevector,self.Zvelplotvector, name='v<sub>z</sub>')
        self.Zvelcurve.setPen(pg.mkPen('b'))

        # ---------------------------------------------- #  
        # desired velocities x,y,z
        self.Xdvelplotvector = [0]*self.Size_Vector
        self.Xdvelcurve = velplotwidget.getPlotItem().plot(self.timevector,self.Xdvelplotvector, name='v<sub>x</sub>')
        self.Xdvelcurve.setPen(pg.mkPen('r', style=PyQt4.QtCore.Qt.DashLine))
        
        self.Ydvelplotvector = [0]*self.Size_Vector
        self.Ydvelcurve = velplotwidget.getPlotItem().plot(self.timevector,self.Ydvelplotvector, name='v<sub>y</sub>')
        self.Ydvelcurve.setPen(pg.mkPen('g', style=PyQt4.QtCore.Qt.DashLine))

        self.Zdvelplotvector = [0]*self.Size_Vector
        self.Zdvelcurve = velplotwidget.getPlotItem().plot(self.timevector,self.Zdvelplotvector, name='v<sub>z</sub>')
        self.Zdvelcurve.setPen(pg.mkPen('b', style=PyQt4.QtCore.Qt.DashLine))

        # ---------------------------------------------- #  
        #  IT ACCEPTS HTML CODE FOR WRITTING ANGLES
        # Euler angles
        self.Roll_plotvector = [0]*self.Size_Vector
        self.Roll_curve = Anglesplotwidget.getPlotItem().plot(self.timevector,self.Roll_plotvector, name='&#966;')
        self.Roll_curve.setPen(pg.mkPen('r'))
        
        self.Pitch_plotvector = [0]*self.Size_Vector
        self.Pitch_curve = Anglesplotwidget.getPlotItem().plot(self.timevector,self.Pitch_plotvector, name='&#952;')
        self.Pitch_curve.setPen(pg.mkPen('g'))

        self.Yaw_plotvector = [0]*self.Size_Vector
        self.Yaw_curve = Anglesplotwidget.getPlotItem().plot(self.timevector,self.Yaw_plotvector, name='&#968;')
        self.Yaw_curve.setPen(pg.mkPen('b'))


        # ---------------------------------------------- #  
        # desired Euler angles
        self.Rolld_plotvector = [0]*self.Size_Vector
        self.Rolld_curve = Anglesplotwidget.getPlotItem().plot(self.timevector,self.Rolld_plotvector, name='&#966;<sub>d</sub>')
        self.Rolld_curve.setPen(pg.mkPen('r', style=PyQt4.QtCore.Qt.DashLine))
        
        self.Pitchd_plotvector = [0]*self.Size_Vector
        self.Pitchd_curve = Anglesplotwidget.getPlotItem().plot(self.timevector,self.Pitchd_plotvector, name='&#952;<sub>d</sub>')
        self.Pitchd_curve.setPen(pg.mkPen('g', style=PyQt4.QtCore.Qt.DashLine))

        self.Yawd_plotvector = [0]*self.Size_Vector
        self.Yawd_curve = Anglesplotwidget.getPlotItem().plot(self.timevector,self.Yawd_plotvector, name='&#968;<sub>d</sub>')
        self.Yawd_curve.setPen(pg.mkPen('b', style=PyQt4.QtCore.Qt.DashLine))    

        # ---------------------------------------------- #  
        # channels
        self.Ch1plotvector = [0]*self.Size_Vector
        self.Ch1curve = channelplotwidget.getPlotItem().plot(self.timevector,self.Ch1plotvector, name='U<sub>1</sub>')
        self.Ch1curve.setPen(pg.mkPen('r'))

        self.Ch2plotvector = [0]*self.Size_Vector
        self.Ch2curve = channelplotwidget.getPlotItem().plot(self.timevector,self.Ch2plotvector, name='U<sub>2</sub>')
        self.Ch2curve.setPen(pg.mkPen('g'))

        self.Ch3plotvector = [0]*self.Size_Vector
        self.Ch3curve = channelplotwidget.getPlotItem().plot(self.timevector,self.Ch3plotvector, name='U<sub>3</sub>')
        self.Ch3curve.setPen(pg.mkPen('b'))

        self.Ch4plotvector = [0]*self.Size_Vector
        self.Ch4curve = channelplotwidget.getPlotItem().plot(self.timevector,self.Ch4plotvector, name='U<sub>4</sub>')
        self.Ch4curve.setPen(pg.mkPen('c'))



        # ---------------------------------------------- #
        # ---------------------------------------------- #   

        
        # THIS IS A NECESSARY MEASURE BECAUSE THE UPDATING OF DATA AND
        # THE SUBSCRIPTION RUN ON DIFFERENT THREADS

        # Connecting slots to signals
        self.Time.connect(self.TimeUpdate)

        # positions x,y,z
        self.Xon.connect(self.XonUpdate)
        self.Yon.connect(self.YonUpdate)
        self.Zon.connect(self.ZonUpdate)

        # desired positions x,y,z
        self.Xdon.connect(self.XdonUpdate)
        self.Ydon.connect(self.YdonUpdate)
        self.Zdon.connect(self.ZdonUpdate)

        # velocities x,y,z
        self.Xvelon.connect(self.XvelonUpdate)
        self.Yvelon.connect(self.YvelonUpdate)
        self.Zvelon.connect(self.ZvelonUpdate)

        # desired velocities x,y,z
        self.Xdvelon.connect(self.XdvelonUpdate)
        self.Ydvelon.connect(self.YdvelonUpdate)
        self.Zdvelon.connect(self.ZdvelonUpdate)

        # Euler angles
        self.Roll_on.connect(self.Roll_onUpdate)
        self.Pitch_on.connect(self.Pitch_onUpdate)
        self.Yaw_on.connect(self.Yaw_onUpdate)

        # Desired Euler angles
        self.Rolld_on.connect(self.Rolld_onUpdate)
        self.Pitchd_on.connect(self.Pitchd_onUpdate)
        self.Yawd_on.connect(self.Yawd_onUpdate)        

        # channels
        self.Ch1on.connect(self.Ch1onUpdate)
        self.Ch2on.connect(self.Ch2onUpdate)
        self.Ch3on.connect(self.Ch3onUpdate) 
        self.Ch4on.connect(self.Ch4onUpdate)          

        
        # ---------------------------------------------- #
        # ---------------------------------------------- #
        # counter for data saving
        self.counter = 1
        self.counterBound = numpy.int(self.Period_Data_Saving*self.Frequency_Subscription)


        # ---------------------------------------------- #
        # ---------------------------------------------- #
        # counter for plotting
        self.counterPlot = 1
        self.counterBoundPlot = numpy.int(self.Period_Plot*self.Frequency_Subscription)


        # ---------------------------------------------- #
        # ---------------------------------------------- #
        # initial time: this will be used to offset to time to 0 
        # instead of plotting with "real" time
        self.time0 = rospy.get_time()


        # ---------------------------------------------- #
        # ---------------------------------------------- #

        # BUTTON TO SUBSCRIBE AND UNSUBSCRIBE
        self._widget.ButtonSubscribe.stateChanged.connect(self.SetSubscription)
       

        # ---------------------------------------------- #
        # ---------------------------------------------- #

        # BUTTON TO ASK FOR SAVING AND STOPPING SAVING AS WELL
        self._widget.ButtonRequestSave.stateChanged.connect(self.SaveDataClient)


    def callback(self,data):

        
        # time vector new data        
        self.Time.emit(data.time)


        # ---------------------------------------------- #
        # ---------------------------------------------- #

        # increase or reset counter plot data saving
        if self.counterPlot <= self.counterBoundPlot:
            self.counterPlot = self.counterPlot + 1
        else:
            self.counterPlot = 1


        # ---------------------------------------------- #
        # ---------------------------------------------- #

        # increase or reset counter plot data saving
        if self.counter <= self.counterBound:
            self.counter = self.counter + 1
        else:
            self.counter = 1


        # ---------------------------------------------- #
        # ---------------------------------------------- #            

        # emit data for x,y,z vectors to be updated
        self.Xon.emit(data.x)
        self.Yon.emit(data.y)
        self.Zon.emit(data.z)
        # emit data for desired x,y,z vectors to be updated
        self.Xdon.emit(data.xd)
        self.Ydon.emit(data.yd)
        self.Zdon.emit(data.zd)
        # emit data for velocities x,y,z vectors to be updated
        self.Xvelon.emit(data.vx)
        self.Yvelon.emit(data.vy)
        self.Zvelon.emit(data.vz)
        # emit data for desired velocities x,y,z vectors to be updated
        self.Xdvelon.emit(data.vxd)
        self.Ydvelon.emit(data.vyd)
        self.Zdvelon.emit(data.vzd)
        # emit data for euler angles vector to be updated
        self.Roll_on.emit(data.roll)
        self.Pitch_on.emit(data.pitch)
        self.Yaw_on.emit(data.yaw)
        # emit data for desired euler angles vector to be updated
        self.Rolld_on.emit(data.roll_d)
        self.Pitchd_on.emit(data.pitch_d)
        self.Yawd_on.emit(data.yaw_d)        
        # emit data for channels z vectors to be updated
        self.Ch1on.emit(data.cmd_1)
        self.Ch2on.emit(data.cmd_2)
        self.Ch3on.emit(data.cmd_3)                           
        self.Ch4on.emit(data.cmd_4)                           

    # ------------------------------------------------------------------------------ #
    # ------------------------------------------------------------------------------ #
    # ------------------------------------------------------------------------------ #
    # ------------------------------------------------------------------------------ #

    def TimeUpdate(self,data):

        if self.counter == 1:
            self.timevector[:-1] = self.timevector[1:]
            self.timevector[-1]  = data - self.time0


    # def XonUpdate(self,data):

        
    #     # updata data vector for x component
    #     if self.counter == 1:
    #         self.Xplotvector[:-1] = self.Xplotvector[1:]
    #         self.Xplotvector[-1]  = data

    
    #     # plot only after certain time interval, according to counter
    #     if self.counterPlot == 1:

    #         # plot if box for x component is ticked
    #         if self._widget.Xcheck.isChecked():
                
    #             # uncomment when interested in cheching time between two data points
    #             # print(self.timevector[-1] - self.timevector[-2])
                
    #             self.Xcurve.setData(self.timevector,self.Xplotvector)

    #         else:
    #             # clear plot 
    #             self.Xcurve.setData([],[])
        

    def VariableOnUpdate(self,VariablePlotVector,data,PlotHandle,WidgetChecked):

        # updata data vector for "x" component
        if self.counter == 1:
            VariablePlotVector[:-1] = VariablePlotVector[1:]
            VariablePlotVector[-1]  = data

    
        # plot only after certain time interval, according to counter
        if self.counterPlot == 1:

            # plot if box for "x" component is ticked
            if WidgetChecked.isChecked():
                
                # uncomment when interested in cheching time between two data points
                # print(self.timevector[-1] - self.timevector[-2])
                
                PlotHandle.setData(self.timevector,VariablePlotVector)

            else:
                # clear plot 
                PlotHandle.setData([],[])


    # ---------------------------------------------- #
    # x, y, z         
    def XonUpdate(self,data):
        return self.VariableOnUpdate(self.Xplotvector,data,self.Xcurve,self._widget.Xcheck)

    def YonUpdate(self,data):
        return self.VariableOnUpdate(self.Yplotvector,data,self.Ycurve,self._widget.Ycheck)

    def ZonUpdate(self,data):
        return self.VariableOnUpdate(self.Zplotvector,data,self.Zcurve,self._widget.Zcheck)

    # ---------------------------------------------- #
    # desired x, y, z
    def XdonUpdate(self,data):
        return self.VariableOnUpdate(self.Xdplotvector,data,self.Xdcurve,self._widget.Xdcheck)

    def YdonUpdate(self,data):
        return self.VariableOnUpdate(self.Ydplotvector,data,self.Ydcurve,self._widget.Ydcheck)

    def ZdonUpdate(self,data):
        return self.VariableOnUpdate(self.Zdplotvector,data,self.Zdcurve,self._widget.Zdcheck)
    
    # ---------------------------------------------- #
    # velocities x, y, z
    def XvelonUpdate(self,data):
        return self.VariableOnUpdate(self.Xvelplotvector,data,self.Xvelcurve,self._widget.Xvelcheck)

    def YvelonUpdate(self,data):
        return self.VariableOnUpdate(self.Yvelplotvector,data,self.Yvelcurve,self._widget.Yvelcheck)

    def ZvelonUpdate(self,data):
        return self.VariableOnUpdate(self.Zvelplotvector,data,self.Zvelcurve,self._widget.Zvelcheck)       

    # ---------------------------------------------- #
    # desired velocities x, y, z
    def XdvelonUpdate(self,data):
        return self.VariableOnUpdate(self.Xdvelplotvector,data,self.Xdvelcurve,self._widget.Xdvelcheck)

    def YdvelonUpdate(self,data):
        return self.VariableOnUpdate(self.Ydvelplotvector,data,self.Ydvelcurve,self._widget.Ydvelcheck)

    def ZdvelonUpdate(self,data):
        return self.VariableOnUpdate(self.Zdvelplotvector,data,self.Zdvelcurve,self._widget.Zdvelcheck)
    
    # ---------------------------------------------- #
    # euler angles         
    def Roll_onUpdate(self,data):
        return self.VariableOnUpdate(self.Roll_plotvector,data,self.Roll_curve,self._widget.Roll_check)

    def Pitch_onUpdate(self,data):
        return self.VariableOnUpdate(self.Pitch_plotvector,data,self.Pitch_curve,self._widget.Pitch_check)

    def Yaw_onUpdate(self,data):
        return self.VariableOnUpdate(self.Yaw_plotvector,data,self.Yaw_curve,self._widget.Yaw_check)

    # ---------------------------------------------- #
    # desired euler angles
    def Rolld_onUpdate(self,data):
        return self.VariableOnUpdate(self.Rolld_plotvector,data,self.Rolld_curve,self._widget.Rolld_check)

    def Pitchd_onUpdate(self,data):
        return self.VariableOnUpdate(self.Pitchd_plotvector,data,self.Pitchd_curve,self._widget.Pitchd_check)

    def Yawd_onUpdate(self,data):
        return self.VariableOnUpdate(self.Yawd_plotvector,data,self.Yawd_curve,self._widget.Yawd_check)

    # ---------------------------------------------- #
    # channels 1, 2, 3 and 4
    def Ch1onUpdate(self,data):
        return self.VariableOnUpdate(self.Ch1plotvector,data,self.Ch1curve,self._widget.Ch1check)

    def Ch2onUpdate(self,data):
        return self.VariableOnUpdate(self.Ch2plotvector,data,self.Ch2curve,self._widget.Ch2check)

    def Ch3onUpdate(self,data):
        return self.VariableOnUpdate(self.Ch3plotvector,data,self.Ch3curve,self._widget.Ch3check)            
        
    def Ch4onUpdate(self,data):
        return self.VariableOnUpdate(self.Ch4plotvector,data,self.Ch4curve,self._widget.Ch4check)          
    # ------------------------------------------------------------------------------ #
    # ------------------------------------------------------------------------------ #
    # ------------------------------------------------------------------------------ #
    # ------------------------------------------------------------------------------ #  


    def SetSubscription(self):

        if self._widget.ButtonSubscribe.isChecked():
            self.sub = rospy.Subscriber(self.namespace+'quad_state_and_cmd', quad_state_and_cmd, self.callback)
        else:
            # unsubscribe to topic
            self.sub.unregister()

            # clear all plots 
            self.Xcurve.setData([],[])
            self.Ycurve.setData([],[])
            self.Zcurve.setData([],[])
            
            self.Xdcurve.setData([],[])
            self.Ydcurve.setData([],[])
            self.Zdcurve.setData([],[])
            
            self.Xvelcurve.setData([],[])
            self.Yvelcurve.setData([],[])
            self.Zvelcurve.setData([],[])            
            
            self.Xdvelcurve.setData([],[])
            self.Ydvelcurve.setData([],[])
            self.Zdvelcurve.setData([],[])

            self.Roll_curve.setData([],[])
            self.Pitch_curve.setData([],[])
            self.Yaw_curve.setData([],[])
            
            self.Rolld_curve.setData([],[])
            self.Pitchd_curve.setData([],[])
            self.Yawd_curve.setData([],[])            

            self.Ch1curve.setData([],[])
            self.Ch2curve.setData([],[])
            self.Ch3curve.setData([],[])            
            self.Ch4curve.setData([],[])   

    #@Slot(bool)
    def SaveDataClient(self):
        
        try: 
            # time out of one second for waiting for service
            rospy.wait_for_service(self.namespace+'SaveDataFromGui',1.0)
            
            try:
                AskForSavingOrNot = rospy.ServiceProxy(self.namespace+'SaveDataFromGui', SaveData)

                # if button is pressed save data
                if self._widget.ButtonRequestSave.isChecked():
                    # request controller to save data
                    reply = AskForSavingOrNot(True)
                    if reply.Saving == True:
                        # if controller receives message, we know it
                        print('Saving')
                else:
                    # request controller to STOP saving data
                    reply = AskForSavingOrNot(False)
                    if  reply.Saving == True:
                        # if controller receives message, we know it
                        print('Stopped Saving')

            except rospy.ServiceException, e:
                print "Service call failed: %s"%e 
            
        except: 
            print "Service not available ..."        
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

    

