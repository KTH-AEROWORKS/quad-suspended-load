#!/usr/bin/env python

import rospy
import mavros
import numpy
from mavros_msgs.msg import OverrideRCIn
from mavros_msgs.srv import CommandBool,SetMode,ParamSet
import os


## Initialize the quadcopter flight mode, change the system ID and arm
##@return true if the quad was successfully armed, false otherwise
def Prepare_For_Flight():

    #Set the flight mode to stabilize (default)
    # Modes: 'LAND', 'STABILIZE', 'ACRO'
    mode_success   = Set_Flight_Mode('STABILIZE')
    # ID_success     = Set_System_ID(1)
    arming_success = Arming_Quad()

    # if mode_success and ID_success and arming_success:
    if mode_success and arming_success:
    # if arming_success:
        return True
    else:   
        return False



def Set_Flight_Mode(MODE):
    
    #Change the flight mode on the Pixhawk flight controller
    try:
        # it waits for service for 10 seconds
        rospy.wait_for_service('mavros/set_mode',10)

        try:
            change_param = rospy.ServiceProxy('mavros/set_mode',SetMode)
            param=change_param(0,MODE)

            if param.success:
                rospy.logwarn('Flight mode changed to '+MODE)
                return True
            else:
                rospy.logwarn('Mavros is not available')
                return False
        except:
            rospy.logwarn('Mavros is not available')
            return False

    except:
        rospy.logwarn('Mavros is not available')
        return False




## Set the system ID.
##
## This is necessary to set the SYSID_MYGCS parameter before flying.
##
##@param id_int: an integer to set the system ID to
##@return true if the system ID was set successfully and a connection to Mavros was establishe, false otherwise
def Set_System_ID(id_int):
    # """This function sets the system ID and checks if it is possible to connect to mavros. The system ID
    # should be 1 to be able to use the rc/override topic."""
    
    rospy.logwarn('Connecting to Mavros ...')
    try:
        rospy.wait_for_service('mavros/param/set',10)

        try:
            change_param=rospy.ServiceProxy('mavros/param/set',ParamSet)
            param=change_param('SYSID_MYGCS',id_int,0.0)

            print(param.success)
            if param.success:
                rospy.logwarn('System ID changed')
                return True
            else:
                rospy.logwarn('Cannot change system ID')
                return False
        except:
            rospy.logwarn('Cannot change system ID')
            return False

    except:
        rospy.logwarn('Mavros is not available (fail to contact service mavros/param/set).')
        return False




## Arm the quad: This function ask for the quad arming.
##
## @param basename: name of the iris group (iris1, iris2, ...)
## @return returns true if the quad was armed successfully and false otherwise
def Arming_Quad(base_name=""):
    """This function is used to arm the quad."""

    #Arming the Quad
    srv_path = 'mavros/cmd/arming'
    if base_name!="":
        srv_path = "/%s/%s"%(base_name,srv_path)

    try:
        rospy.logwarn('Arming Quad ...')
        rospy.wait_for_service(srv_path,10)

        try:
            arming = rospy.ServiceProxy(srv_path,CommandBool)
            arming_result=arming(True)
            if arming_result.success:
                rospy.logwarn('Quad is Armed!!!!')
                return True
            else:
                rospy.logwarn('Cannot arm quad')
                return False

        except:
            rospy.logwarn('Cannot arm quad')
            return False

    except:
        rospy.logwarn('No connection to Mavros')
        return False


## Arm the quad: This function ask for the quad arming.
##
## @param basename: name of the iris group (iris1, iris2, ...)
## @return returns true if the quad was armed successfully and false otherwise
def UNArming_Quad(base_name=""):
    """This function is used to arm the quad."""

    #Arming the Quad
    srv_path = 'mavros/cmd/arming'
    if base_name!="":
        srv_path = "/%s/%s"%(base_name,srv_path)

    try:
        rospy.logwarn('UNArming Quad ...')
        rospy.wait_for_service(srv_path,10)

        try:
            arming = rospy.ServiceProxy(srv_path,CommandBool)
            arming_result=arming(False)
            if arming_result.success:
                rospy.logwarn('Quad is UNArmed!!!!')
                return True
            else:
                rospy.logwarn('Cannot UNarm quad')
                return False

        except:
            rospy.logwarn('Cannot UNarm quad')
            return False

    except:
        rospy.logwarn('No connection to Mavros')
        return False


def shutdown():
    rospy.loginfo("Shutting down Sample Node...")    
    flag = UNArming_Quad()

    print(flag)
    while not flag:
        flag = UNArming_Quad()
        print(flag)



def alternating_publisher():
    pub = rospy.Publisher('mavros/rc/override', OverrideRCIn, queue_size=100)
    rospy.init_node('Alternating_publisher', anonymous=True)
    alt = True
    r = 30
    #starttime = rospy.get_time()
    #pwd = os.environ['PWD']
    #FF = file(str(pwd)+'/data_'+str(int(starttime))+'.txt','w')
    #print r
    rate = rospy.Rate(r) # r hz


    # Execute this function when shutting down
    # rospy.on_shutdown(shutdown)   


    # output = [int(1500),int(1500),int(1000),int(1500),int(1000),int(1000),int(1000),int(1000)]
    # pub.publish(output)

    # Arming_Quad()
    # Set_System_ID(1)
    # Set_Flight_Mode('LAND')
    # Arming_Quad()
    # Arming_Quad()
    # Arming_Quad()
    # Arming_Quad()
    # Arming_Quad()

    # output = [int(1500),int(1500),int(1000),int(1500),int(1500),int(1500),int(1500),int(1500)]
    # # print(1500 + 500*numpy.sin(6.3/10*t))
    # pub.publish(output)
    # flag = Prepare_For_Flight()
    # print(flag)
    # counter = 0
    # while not flag:
    #     output = [int(1500),int(1500),int(1000),int(1500),int(1500),int(1500),int(1500),int(1500)]
    #     # print(1500 + 500*numpy.sin(6.3/10*t))
    #     pub.publish(output)
    #     flag = Prepare_For_Flight()
    #     print(flag)
    #     counter = counter +1
    #     if counter==10:
    #         break

    t0 = rospy.get_time() 


    while not rospy.is_shutdown():

        # output = [int(1500),int(1500),int(1000),int(1500),int(1500),int(1500),int(1500),int(1500)]
        # print(1500 + 500*numpy.sin(6.3/10*t))
        # pub.publish(output)
        
        if rospy.get_time() - t0 < 10:
            t = rospy.get_time() - t0
            # output = [int(1500),int(1500),int(1500 + 500*numpy.sin(6.3/10*t)),int(1500),int(1500),int(1500),int(1500),int(1500)]
            # print(1500 + 500*numpy.sin(6.3/10*t))
            output = [int(1500),int(1500),int(1000),int(1500),int(1500),int(1500),int(1500),int(1500)]
            pub.publish(output)
            print(t)
        else:
            if rospy.get_time() - t0 < 20:
                t = rospy.get_time() - t0
                output = [int(1500),int(1500),int(1500 + 500*numpy.sin(6.3/5*t)),int(1500),int(1500),int(1500),int(1500),int(1500)]
                # output = [int(1500),int(1500),int(1500),int(1500),int(1500),int(1500),int(1500),int(1500)]
                pub.publish(output)
                print(2)
            else:
                print(3)
                output = [int(1500),int(1500),int(1000),int(1500),int(1500),int(1500),int(1500),int(1500)]
                pub.publish(output)
                # # print('a')
                # # Set_Flight_Mode('LAND')
                # # UNArming_Quad()
                # # UNArming_Quad()
                # output = [int(1500),int(1500),int(1000),int(1000),int(1500),int(1500),int(1500),int(1500)]
                # pub.publish(output)                
                # flag2 = Set_Flight_Mode('LAND')
                # flag1 = UNArming_Quad()
                # counter = 0
                # while not (flag1 and flag2):
                #     print('not yet')
                #     output = [int(1500),int(1500),int(1000),int(1000),int(1500),int(1500),int(1500),int(1500)]
                #     pub.publish(output)                    
                #     flag2 = Set_Flight_Mode('LAND')
                #     flag1 = UNArming_Quad()
                #     counter = counter +1
                #     if counter==10:
                #         break

        # numpy.savetxt(FF,[numpy.concatenate([[rospy.get_time()-starttime],output])])
        rate.sleep()

if __name__ == '__main__':
    try:
        alternating_publisher()
    except rospy.ROSInterruptException:
        pass
