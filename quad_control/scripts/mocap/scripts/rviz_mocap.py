#!/usr/bin/env python
#Vinzenz Minnig, 2015

#Periodically requests data from the ros_mocap node (Qualysis tracking information)
#Publishes two topics read by RVIZ: one for the position and orientation of the quad (rviz/quad_position) and for for the path (rviz/quad_trajectory)
#This node is called with an argument (ID of the body to draw)
#If there is no available body with that ID, the node is shut down

import sys
import rospy
import math
from visualization_msgs.msg import Marker
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from mocap.srv import BodyData
from mocap .msg import QuadPosition


def draw_body_found(body):
	#compute quaternion from euler angles (rviz marker needs quaternions...)
	c1=math.cos(math.radians(body.pos.yaw)/2)
	c2=math.cos(math.radians(body.pos.pitch)/2)
	c3=math.cos(math.radians(-body.pos.roll)/2)
	s1=math.sin(math.radians(body.pos.yaw)/2)
	s2=math.sin(math.radians(body.pos.pitch)/2)
	s3=math.sin(math.radians(-body.pos.roll)/2)

	x=(c1*c2*c3)+(s1*s2*s3)
	y=(s1*c2*c3)-(c1*s2*s3)
	z=(c1*s2*c3)+(s1*c2*s3)
	w=(c1*c2*s3)-(s1*s2*c3)

	#if an available body has been found, draw it at the right location, with the right orientation	
	data=Marker()
	data.header.frame_id='map'
	data.header.stamp=rospy.get_rostime()
	data.type=1
	data.action=0
	data.pose.position.x=body.pos.x
	data.pose.position.y=body.pos.y
	data.pose.position.z=body.pos.z
	data.pose.orientation.x=x
	data.pose.orientation.y=y
	data.pose.orientation.z=z
	data.pose.orientation.w=w
	data.scale.x=0.4
	data.scale.y=0.2
	data.scale.z=0.05
	data.color.a=1.0
	data.color.r=0.1
	data.color.g=0.1
	data.color.b=1.0
	return data


def draw_body_error(body):
	#if there is no available body with that id, the quad is drawn at the origin, and in red
	#this can happen for example when the quad leaves the range of the Motion Capture System
	data=Marker()
	data.header.frame_id='map'
	data.header.stamp=rospy.get_rostime()
	data.type=1
	data.action=0
	data.pose.position.x=0
	data.pose.position.y=0
	data.pose.position.z=0.025
	data.pose.orientation.x=0.0
	data.pose.orientation.y=0.0
	data.pose.orientation.z=0.0
	data.pose.orientation.w=1.0
	data.scale.x=0.4
	data.scale.y=0.2
	data.scale.z=0.05
	data.color.a=1.0
	data.color.r=1.0
	data.color.g=0.0
	data.color.b=0.0
	return data


def talk(body_id):
	loop_rate=rospy.Rate(30)
	position=rospy.Publisher('rviz/quad_position',Marker,queue_size=10)
	trajectory=rospy.Publisher('rviz/quad_trajectory',Path,queue_size=10)

	#wait for the mocap services to be up and running
	try:
		rospy.loginfo('Connecting to the mocap system...')
		rospy.wait_for_service('mocap_get_data',10)
	except:
		rospy.logerr('[RVIZ] No connection to the mocap system')
		sys.exit()
	rospy.loginfo('[RVIZ] Connection established')

	path=Path()
	path.header.stamp=rospy.get_rostime()
	path.header.frame_id='map'

	posestamped=PoseStamped()
	posestamped.header.stamp=rospy.get_rostime()
	posestamped.header.frame_id='map'
	posestamped.pose.position.x=0.0
	posestamped.pose.position.y=0.0
	posestamped.pose.position.z=0.0

	list_posestamped=[posestamped]
	del list_posestamped[0]

	while not rospy.is_shutdown():
		#try to contact the mocap service, exit the program if there is no connection
		try:
			body_info=rospy.ServiceProxy('mocap_get_data',BodyData)
			body=body_info(body_id)
		except:
			rospy.logerr('[RVIZ] No connection to the mocap system')
			sys.exit()

		#distinguish between the case where a body has been found, and when it hasn't
		if body.pos.found_body:
			data_to_rviz=draw_body_found(body)
			#add the current point to the trajectory line
			current_point=PoseStamped()
			current_point.pose.position.x=data_to_rviz.pose.position.x
			current_point.pose.position.y=data_to_rviz.pose.position.y
			current_point.pose.position.z=data_to_rviz.pose.position.z
			list_posestamped.append(current_point)
		else:
			rospy.logerr('[RVIZ] The requested body is not available')
			data_to_rviz=draw_body_error(body)


		#publish the data to rviz
		path.poses=list_posestamped

		#position and orientation of the quad
		position.publish(data_to_rviz)
		#trajectory of the quad
		trajectory.publish(path)
		
		loop_rate.sleep()


if __name__=='__main__':
	rospy.init_node('rviz_mocap')
	#get the body id parameter from the parameters space
	body_id=int(rospy.get_param('/rviz_mocap/qualysis_id',7))
	if rospy.has_param('/rviz_mocap/qualysis_id'):
		rospy.loginfo('Found body id: '+str(body_id))
	else:
		rospy.logwarn('No body id found - trying with default: '+str(body_id))

	talk(body_id)
	
#EOF
