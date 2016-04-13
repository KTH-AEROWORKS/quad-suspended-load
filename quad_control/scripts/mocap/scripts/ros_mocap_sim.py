#!/usr/bin/env python

import rospy
import sys
import ast
import sml_setup

from mocap.msg import QuadPosition
from mocap.msg import QuadPositionDerived

from mocap.msg import QuadPosition
from mocap.srv import Bodies
from mocap.srv import BodyData
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose

from rotmat import Vector3, Matrix3
from math import radians, degrees

import analysis
import utils

import copy
import control

import pid

from system import System
import math

import message_filters

import timeit

## Convert quaternion to DCM
# @param q1, q2, q3, q4: quaternion
# @return dcm matrix
def quat_to_dcm(q1, q2, q3, q4):
	q3q3 = q3 * q3
	q3q4 = q3 * q4
	q2q2 = q2 * q2
	q2q3 = q2 * q3
	q2q4 = q2 * q4
	q1q2 = q1 * q2
	q1q3 = q1 * q3
	q1q4 = q1 * q4
	q4q4 = q4 * q4

	m = Matrix3()
	m.a.x = 1.0-2.0*(q3q3 + q4q4)
	m.a.y =   2.0*(q2q3 - q1q4)
	m.a.z =   2.0*(q2q4 + q1q3)
	m.b.x =   2.0*(q2q3 + q1q4)
	m.b.y = 1.0-2.0*(q2q2 + q4q4)
	m.b.z =   2.0*(q3q4 - q1q2)
	m.c.x =   2.0*(q2q4 - q1q3)
	m.c.y =   2.0*(q3q4 + q1q2)
	m.c.z = 1.0-2.0*(q2q2 + q3q3)
	return m

class Signal():
	def __init__(self,fc_p,fc_v,fc_a,dt=1/30.0):
		s = control.tf([1.0,0.0],[1.0])
		self.pos = System( 1 ,dt=dt)
		self.vel = System( s/(1+s/(fc_v*2.0*math.pi)) ,dt=dt)
		self.acc = System( (s/(1+s/(fc_a*2.0*math.pi)) )**2 ,dt=dt)

	def append(self,s,t):
		self.p = self.pos.output(s,t)
		self.v = self.vel.output(s,t)
		self.a = self.acc.output(s,t)

		self.pos.next_state()
		self.vel.next_state()
		self.acc.next_state()

	def get(self):
		return [self.p, self.v, self.a]


class State():
	def __init__(self):
		fc_p = 10
		fc_v = 2
		fc_a = 1
		self.x = Signal(fc_p,fc_v,fc_a)
		self.y = Signal(fc_p,fc_v,fc_a)
		self.z = Signal(fc_p,fc_v,fc_a)
		self.roll = Signal(fc_p,fc_v,fc_a)
		self.pitch = Signal(fc_p,fc_v,fc_a)
		self.yaw = Signal(fc_p,fc_v,fc_a)
		self.data = False
		self.time = 0.0

	def update(self,data,t):
		data = copy.deepcopy(data)
		self.found_body = data.found_body
		self.x.append(data.x,t)
		self.y.append(data.y,t)
		self.z.append(data.z,t)
		self.roll.append(data.roll,t)
		self.pitch.append(data.pitch,t)
		self.yaw.append(data.yaw,t)

	def get_message(self):
		result = QuadPositionDerived()

		result.found_body=self.found_body

		result.x=self.x.p
		result.y=self.y.p
		result.z=self.z.p
		result.pitch=self.pitch.p
		result.roll=self.roll.p
		result.yaw=self.yaw.p

		result.x_vel=self.x.v
		result.y_vel=self.y.v
		result.z_vel=self.z.v
		result.pitch_vel=self.pitch.v
		result.roll_vel=self.roll.v
		result.yaw_vel=self.yaw.v

		result.x_acc=self.x.a
		result.y_acc=self.y.a
		result.z_acc=self.z.a
		result.pitch_acc=self.pitch.a
		result.roll_acc=self.roll.a
		result.yaw_acc=self.yaw.a
		
		# The time_diff is added to make the position plot on the
		# GUI work
		result.time_diff = 0.05

		return copy.deepcopy(result)

class Mocap:
	def __init__(self):
		utils.loginfo('Mocap starting')

		utils.loginfo('start publishing')
		
		rate=rospy.Rate(30)

		self.all_bodies = dict()

		#Get parameters (all the body ID's that are requested)
		self.body_names=utils.Get_Parameter('body_names',['iris1','iris2'])
		self.body_array=utils.Get_Parameter('body_array',[1,2])
		if type(self.body_array) is str:
			self.body_array=ast.literal_eval(self.body_array)

		#Get topic names for each requested body
		body_topic_array=self.Get_Topic_Names(self.body_array)

		#Establish one publisher topic for each requested body
		topics_publisher=self.Get_Publishers(body_topic_array)

		# Create the different body objects
		self.body_state = []
		for i in range(0,len(self.body_array)):
			self.body_state.append(State())

		# Suscrib to Gazebo
		self.start_subscribes()

		while not rospy.is_shutdown():
			for i in range(0,len(self.body_array)):
				if self.body_state[i].data:
					data = self.all_bodies[self.body_names[i]]

					self.body_state[i].update(data,self.body_state[i].time)
					topics_publisher[i].publish(self.body_state[i].get_message())
			 		self.body_state[i].data = False
			# for i in range(0,len(self.body_array)):
			# 	if self.body_state[i].data:
			# 		topics_publisher[i].publish(self.body_state[i].get_message())
			# 		self.body_state[i].data = False

			rate.sleep()

		utils.logwarn('Mocap stop publishing')

	def get_bodies(self,arg):
		num_bodies = len(self.bodies.name)
		return {'list':range(num_bodies)}

	def get_data(self,bodies,body_id,time):
		data = QuadPosition()

		data.found_body = False
		
		if bodies==[]:
			utils.loginfo("Gazebo not started")
		else:
			if hasattr(bodies,'name'):
				if body_id<len(bodies.name):
					data.found_body=True
					data.x=bodies.pose[body_id].position.x
					data.y=bodies.pose[body_id].position.y
					data.z=bodies.pose[body_id].position.z
					x = bodies.pose[body_id].orientation.x
					y = bodies.pose[body_id].orientation.y
					z = bodies.pose[body_id].orientation.z
					w = bodies.pose[body_id].orientation.w

					dcm = quat_to_dcm(w,x,y,z)
					(roll,pitch,yaw) = dcm.to_euler()

					data.pitch=degrees(pitch)
					data.roll=degrees(roll)
					data.yaw=degrees(yaw)
				else:
					print 'Body not found'
			
		return copy.deepcopy(data)



	def update_positions(self,msg):
		#utils.loginfo('receive data from Gazebo')
		bodies = copy.deepcopy(msg)

		t = rospy.Time.now()
		time = t.secs+(t.nsecs/1.0E9)

		for i in range(0,len(self.body_array)):
			if any(self.body_names[i] == s for s in bodies.name):
				indice_from_gazebo = bodies.name.index(self.body_names[i])

				data = self.get_data(bodies,indice_from_gazebo)
				self.body_state[i].update(data,time)
				self.body_state[i].data = True
		
		if not hasattr(self,"spend"):
			self.spend=0.0
			self.last = rospy.Time.now()
			
		self.spend += timeit.default_timer()-tic

		if (rospy.Time.now()-self.last).secs>0:
			self.last = rospy.Time.now()
			print self.spend
			self.spend = 0.0

	def update_positions_links(self,msg):
		#utils.loginfo('receive data from Gazebo')
		bodies = copy.deepcopy(msg)

		t = rospy.Time.now()
		time = t.secs+(t.nsecs/1.0E9)

		for i in range(0,len(self.body_array)):
			if any(self.body_names[i] == s for s in bodies.name):
				indice_from_gazebo = bodies.name.index(self.body_names[i])

				data = self.get_data(bodies,indice_from_gazebo)
				self.body_state[i].update(data,time)
				self.body_state[i].data = True
		print rospy.Time.now()-t

	def get_msg(self,msg):
		#utils.loginfo('receive data from Gazebo')
		bodies = copy.deepcopy(msg)

		t = rospy.Time.now()
		time = t.secs+(t.nsecs/1.0E9)

		for i in range(0,len(self.body_array)):
			if any(self.body_names[i] == s for s in bodies.name):
				indice_from_gazebo = bodies.name.index(self.body_names[i])
				data = self.get_data(bodies,indice_from_gazebo,time)
				self.all_bodies[self.body_names[i]] = data
				self.body_state[i].data = True
				self.body_state[i].time = time
				


	def start_subscribes(self):
		utils.loginfo('Suscrib')
		rospy.Subscriber("/gazebo/model_states",ModelStates,self.get_msg)
		rospy.Subscriber("/gazebo/link_states",ModelStates,self.get_msg)


	def Get_Topic_Names(self,bodies):
		a=len(bodies)
		result=[]
		for i in range(0,a):
			result.append('body_data/id_'+str(self.body_array[i]))

		return(result)


	def Get_Publishers(self,topic_array):
		a=len(topic_array)
		result=[]
		for i in range(0,a):
			result.append(rospy.Publisher(topic_array[i],QuadPositionDerived,queue_size=10))

		return(result)



if __name__=="__main__":
	rospy.init_node("ros_mocap")
	Mocap()

#EOF
