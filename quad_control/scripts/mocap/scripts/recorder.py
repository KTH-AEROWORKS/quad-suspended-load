#!/usr/bin/env python

# A ros node that records the position, velocity and acceleration data published by 
# the ros_mocap node for a certain drone when when it receives a message to do so.
import rospy
import subprocess
import os
import signal
from mocap.msg import Record
from mocap.msg import QuadPositionDerived

class Recorder():

    def __init__(self):
        rospy.init_node('recorder')
        rospy.Subscriber('/recorder', Record, self.callback)
        self.irisdict = dict(zip(['iris1','iris2','iris3','iris4','iris5'],rospy.get_param('/body_array',[1,2,3,4,5])))
        self.p1 = None
        self.p2 = None
        self.p3 = None
        self.p4 = None
        self.p5 = None
        self.last_msg = Record()
        self.last_msg.record_iris1 = False
        self.last_msg.record_iris2 = False
        self.last_msg.record_iris3 = False
        self.last_msg.record_iris4 = False
        self.last_msg.record_iris5 = False
        self.pwd = os.environ['PWD']
        rospy.spin()
        


    def callback(self,data):

        # time is appended to the filenames to make them unique

        if data.record_iris1 != self.last_msg.record_iris1:
            if data.record_iris1:
                self.p1 = subprocess.Popen(['rosbag','record','-O', self.pwd + '/src/kampala/analysis/bagfiles/iris1_'+str(rospy.get_time()),'/body_data/id_'+str(self.irisdict['iris1'])])
            else:
                self.terminate_process_and_children(self.p1)

        if data.record_iris2 != self.last_msg.record_iris2:
            if data.record_iris2:
                self.p2 = subprocess.Popen(['rosbag','record','-O', self.pwd + '/src/kampala/analysis/bagfiles/iris2_'+str(rospy.get_time()),'/body_data/id_'+str(self.irisdict['iris2'])])
            else:
                self.terminate_process_and_children(self.p2)

        if data.record_iris3 != self.last_msg.record_iris3:
            if data.record_iris3:
                self.p3 = subprocess.Popen(['rosbag','record','-O', self.pwd + '/src/kampala/analysis/bagfiles/iris3_'+str(rospy.get_time()),'/body_data/id_'+str(self.irisdict['iris3'])])
            else:
                self.terminate_process_and_children(self.p3)

        if data.record_iris4 != self.last_msg.record_iris4:
            if data.record_iris4:
                self.p4 = subprocess.Popen(['rosbag','record','-O', self.pwd + '/src/kampala/analysis/bagfiles/iris4_'+str(rospy.get_time()),'/body_data/id_'+str(self.irisdict['iris4'])])
            else:
                self.terminate_process_and_children(self.p4)

        if data.record_iris5 != self.last_msg.record_iris5:
            if data.record_iris5:
                self.p5 = subprocess.Popen(['rosbag','record','-O', self.pwd + '/src/kampala/analysis/bagfiles/iris5_'+str(rospy.get_time()),'/body_data/id_'+str(self.irisdict['iris5'])])
            else:
                self.terminate_process_and_children(self.p5)

        self.last_msg = data

        # This function is copied from http://answers.ros.org/question/10714/start-and-stop-rosbag-within-a-python-script/. It is
        # used to stop recording with rosbag.
    def terminate_process_and_children(self,p):
        ps_command = subprocess.Popen("ps -o pid --ppid %d --noheaders" % p.pid, shell=True, stdout=subprocess.PIPE)
        ps_output = ps_command.stdout.read()
        retcode = ps_command.wait()
        assert retcode == 0, "ps command returned %d" % retcode
        for pid_str in ps_output.split("\n")[:-1]:
            os.kill(int(pid_str), signal.SIGINT)
        p.terminate()



        
        
if __name__ == "__main__":
    r = Recorder()

   


    
