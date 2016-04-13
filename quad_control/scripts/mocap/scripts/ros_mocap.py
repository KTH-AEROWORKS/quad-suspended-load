#!/usr/bin/env python
"""Motion Capture script

Get the data coming from the Qualisys computer using the "mocap_source.py" script.
Perform the derivation of each signals (x, y, z, roll, pitch, yaw).
Send information in a QuadPositionDerived message.

Set the found_body attribute after 5 missing packets from Qualisys in order to support weird networking behaviours from some computers.
No filter is implemented.
"""

import rospy
import mocap_source
import sys
import ast
#import sml_setup

# from mocap.msg import QuadPosition
# from mocap.msg import QuadPositionDerived

# import analysis
# import utils

# import controller


# ## Class for storing the time information.
# ## Used in the computation of the derivative.
# class Time():
# 	## Constructor: init the time with the current one
# 	def __init__(self):
# 		self.current_time=rospy.Time.now()
# 		self.past_time=self.current_time
# 		self.time_diff=0

# 	## Compute the time difference and store the current time.	
# 	## @return difference of time since the last call.
# 	def get_time_diff(self):
# 		self.past_time=self.current_time
# 		self.current_time=rospy.Time.now()
# 		d_time=self.current_time-self.past_time
# 		self.time_diff=d_time.secs+(d_time.nsecs/1E9)

# 		return self.time_diff

## Get the body data from mocap_source.py.
## @param body_id: Qualisys id of the body (number of the body that appear in the list in the Qualisys software).
## @return QuadPosition message with the x, y, z, roll, pitch and yaw position of the body. 
def Get_Body_Data(body_id):

	
 	# Get the bodies from mocap_source.py
	bodies=Qs.get_updated_bodies()
	body_length=len(bodies)
	body_indice=-1

	# Get the corresponding id of the body
	if isinstance(bodies,list):
		for i in range(0,body_length):
			if(bodies[i]['id']==body_id):
				body_indice=i
				print('bbb')
				

	# Store data in a QuadPosition object
	#data=QuadPosition()

	# If the body does not appear in the list, label it as "not found"
	#if(body_indice==-1):
	#	#utils.logerr('Body %i not found'%(body_id))
	#	data.found_body=False
	#	return(data)
	
	
# 	data.found_body=True
# 	data.x=bodies[body_indice]["x"]
# 	data.y=bodies[body_indice]["y"]
# 	data.z=bodies[body_indice]["z"]
# 	data.pitch=bodies[body_indice]["pitch"]
# 	data.roll=bodies[body_indice]["roll"]
# 	data.yaw=bodies[body_indice]["yaw"]

# 	return(data)

# ## Get the topics names of the body we want to track. The topic will be "/body_data/id_'Qid'" where 'Qid' is the Qualisys identifier.
# ## @param bodies: list of body Qualisys id.
# ## @return list of topics where information will be send on.
# def Get_Topic_Names(bodies):
# 	a=len(bodies)
# 	result=[]
# 	for i in range(0,a):
# 		result.append('body_data/id_'+str(bodies[i]))

# 	return(result)

# ## Create every publisher from a list of topic names and store them in a list.
# ## @param topic_array: list of topic names.
# ## @return list of publisher objects.
# def Get_Publishers(topic_array):
# 	a=len(topic_array)
# 	result=[]
# 	for i in range(0,a):
# 		result.append(rospy.Publisher(topic_array[i],QuadPositionDerived,queue_size=10))

# 	return(result)


# ## Create the message of the body position initialised with data.
# ## @param current_data: initialising data.
# ## @return message with the current position of the quad (but not yet with the velocity and the acceleration).
# def Insert_Current_Data(current_data):

# 	result=QuadPositionDerived()
# 	result.found_body=current_data.found_body
# 	result.x=current_data.x
# 	result.y=current_data.y
# 	result.z=current_data.z
# 	result.pitch=current_data.pitch
# 	result.roll=current_data.roll
# 	result.yaw=current_data.yaw
# 	return(result)

# ## Compute the derivative of a signal.
# ## @param current: current value of the signal.
# ## @param past: past value of the signal.
# ## @param time: time difference between the two values.
# ## @return time derivative, return 0 if the derivative is infinite.
# def Compute_Derivative(current,past,time):
# 	if time:
# 		result=(current-past)/time
# 		return(result)
# 	else:
# 		utils.logwarn("Infinite derivative.")
# 		return 0



# ## Compute all velocity and acceleration of the quad.
# ## @param current_data: current position of the quad.
# ## @param past_data: past position of the quad.
# ## @param time: time difference between the two values.
# ## @return QuadPositionDerived message with the position, velocity and acceleration of the quad.
# def Get_Derived_Data(current_data,past_data,time):
# 	result=Insert_Current_Data(current_data)
# 	result.x_vel=Compute_Derivative(current_data.x,past_data.x,time)
# 	result.y_vel=Compute_Derivative(current_data.y,past_data.y,time)
# 	result.z_vel=Compute_Derivative(current_data.z,past_data.z,time)
# 	result.pitch_vel=Compute_Derivative(current_data.pitch,past_data.pitch,time)
# 	result.roll_vel=Compute_Derivative(current_data.roll,past_data.roll,time)
# 	result.yaw_vel=Compute_Derivative(current_data.yaw,past_data.yaw,time)
# 	result.x_acc=Compute_Derivative(result.x_vel,past_data.x_vel,time)
# 	result.y_acc=Compute_Derivative(result.y_vel,past_data.y_vel,time)
# 	result.z_acc=Compute_Derivative(result.z_vel,past_data.z_vel,time)
# 	result.pitch_acc=Compute_Derivative(result.pitch_vel,past_data.pitch_vel,time)
# 	result.roll_acc=Compute_Derivative(result.roll_vel,past_data.roll_vel,time)
# 	result.yaw_acc=Compute_Derivative(result.yaw_vel,past_data.yaw_vel,time)
	
# 	# CORRECT THIS: time of MOCAP IS NOT BEING USED!!!!!!!
# 	result.time_diff = time

# 	return(result)


## Main loop of the mocap script that get data from the nocap_source script (Qualisys) and send them to each topics.
# @ros_param frequency: Sending frequency of the quads data.
# @ros_param body_array: Array with each id that we want to track.
def start_publishing():

	# Set the frequency
	# frequency = rospy.get_param("frequency",30)
	# utils.loginfo("Mocap will work at " + str(frequency) + "Hz ")
	# rate=rospy.Rate(frequency)

	rate=rospy.Rate(10)

	# Init the Time object used for the derivative computation
	#timer=Time()

	# Get parameters (all the body ID's that are requested)
	#body_array=utils.Get_Parameter('body_array',[8,16,17,20])
	#if type(body_array) is str:
	#	body_array=ast.literal_eval(body_array)

	# Get topic names for each requested body
	#body_topic_array=Get_Topic_Names(body_array)

	# Establish one publisher topic for each requested body
	#topics_publisher=Get_Publishers(body_topic_array)

	# Initialize empty past data list
	#mocap_past_data=[]
	#empty_data=QuadPositionDerived()
	#for i in range(0,len(body_array)):
	#	mocap_past_data.append(empty_data)

	# Initialize error numbers
	#error = [0]*len(body_array)

	while not rospy.is_shutdown():

		# # Update the Time object and get the difference of time since the last call
		# delta_time=timer.get_time_diff()

		# for i in range(0,len(body_array)):
		# 	# Get the data for the body with the id body_array[i]
		# 	mocap_data=Get_Body_Data(body_array[i])

		# 	if mocap_data.found_body:
		# 		mocap_data_derived=Get_Derived_Data(mocap_data,mocap_past_data[i],delta_time)

		# 		# Update past mocap data
		# 		mocap_past_data[i]=mocap_data_derived

		# 		# Publish data on topic
		# 		topics_publisher[i].publish(mocap_data_derived)
		# 		error[i]=0
		# 	else:
		# 		# Support for bad computers that have trouble with the network
		# 		#  (for my computer, some of the packets are lost, and as the security guard land the quad as soon as the tracking of the quad is lost, we notice the tracking lost after several missing packets)
		# 		error[i]+=1

		# 		if error[i]<30:
		# 			if error[i]>5:
		# 				mocap_past_data[i].found_body = False
		# 				utils.logwarn("Send body %i not found"%(body_array[i]))
		# 			utils.logwarn("Body %i: %i errors"%(body_array[i],error[i]))
		# 		elif error[i]==30:
		# 			utils.logwarn("Body %i: %i errors. Stop printing Errors!"%(body_array[i],error[i]))

		# 		topics_publisher[i].publish(mocap_past_data[i])





			# Get the data for the body with the id body_array[i]
		mocap_data=Get_Body_Data(21)

		print(mocap_data)

			# if mocap_data.found_body:
			# 	# mocap_data_derived=Get_Derived_Data(mocap_data,mocap_past_data[i],delta_time)

			# 	# Update past mocap data
			# 	mocap_past_data[i]=mocap_data_derived

			# 	# Publish data on topic
			# 	topics_publisher[i].publish(mocap_data_derived)
			# 	error[i]=0
			# else:
			# 	# Support for bad computers that have trouble with the network
			# 	#  (for my computer, some of the packets are lost, and as the security guard land the quad as soon as the tracking of the quad is lost, we notice the tracking lost after several missing packets)
			# 	error[i]+=1

			# 	if error[i]<30:
			# 		if error[i]>5:
			# 			mocap_past_data[i].found_body = False
			# 			utils.logwarn("Send body %i not found"%(body_array[i]))
			# 		utils.logwarn("Body %i: %i errors"%(body_array[i],error[i]))
			# 	elif error[i]==30:
			# 		utils.logwarn("Body %i: %i errors. Stop printing Errors!"%(body_array[i],error[i]))

			# 	topics_publisher[i].publish(mocap_past_data[i])

		print('aaaa')
		rate.sleep()


if __name__=="__main__":

	# rospy.init_node('my_node_name')
	rospy.init_node("ros_mocap_message")


	Qs     = mocap_source.Mocap(info=0)
	bodies = Qs.get_updated_bodies()

	if(bodies=="off"):
		rospy.logerr("No connection to the Qualisys Motion Capture System")
		sys.exit()
	else:
		start_publishing()

#EOF
