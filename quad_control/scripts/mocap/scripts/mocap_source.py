# Author: Matteo Vanin
#parser of response message for a command type of message
import socket   #for sockets
import sys      #for exit
import struct
import math
import time

class Mocap(object):

    def __init__(self, host=None, port=None, info=0):
        #set IP and PORT of the Qtm PC
        if host is None:
            #host='130.237.50.87'
            host = '130.237.50.84'
        if port is None:
            port = 22224
        self.host = host
        self.port = port
        #create socket connection
        self.socket = self._create_connection(host,port,info)
        if self.socket is not None:
            self._start_measurement()
        
    def _create_connection(self,host,port,printinfo):
        #create socket
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        except socket.error:
            print ('Failed to create socket')
            sys.exit()    
        if printinfo:
            print ('\nSocket Created \n')

        #create a socket connection
        s.settimeout(0.1)
        try:
            s.connect((host , port))  
        except:
            return None

        if printinfo:
            print ('Socket Connected on host ' + host + ', port ' + str(port) + '\n')

        #Parse the WELCOME MESSAGE (always 35 Bytes)
        msg = _parser_comm(s)
        if printinfo:
            print('---Qualysis message:---')
            print(msg['message'] + '\n')

        # set the communication protocol version to 1.11
        str_to_send = 'Version 1.11'
        msg = self._build_packet(str_to_send,1);
        s.sendall(msg)
        # Parse the VERSION MESSAGE 
        msg = _parser_comm(s)
        if printinfo:
            print('---Qualysis message:---')
            print(msg['message'] + '\n')
        return s
    
    def _build_packet(self,data,type):
        if sys.version_info > (2, 8):
            data_bytes = bytes(data,'UTF-8')
        else:
            data_bytes = bytearray(data,'UTF-8')

        data_len = len(data_bytes)
        packet_size = data_len + 9 #message size plus 8B of header and 1B of \x00 trailer
        header_size = struct.pack('>l',packet_size)
        header_type = struct.pack('>l',type)
        msg_to_send = header_size + header_type + data_bytes + b'\x00'
        return msg_to_send

    def _send_command(self,command):
        msg = self._build_packet(command,1)
        self.socket.sendall(msg)
        # if command == 'Close':
        #     #self.socket.close()
        #     return 
        # else:
        return _parser_comm(self.socket)

    def _start_measurement(self):
        reply = self._send_command('New')
        if reply['message'] == 'You must be master to issue this command\x00':
            reply = self._send_command('TakeControl sml')
            if reply['message'] == 'You are now master\x00':
                reply = self._send_command('New')
 
    def _stop_measurement(self):
        self._send_command('Close')
        #self._send_command('ReleaseControl')
        #print reply

    def ask_for_6DOFinfo(self):
        str_to_send = 'GetCurrentFrame 6DEuler'
        msg = self._build_packet(str_to_send,1)
        try:
            self.socket.sendall(msg)
            return True
        except:
            None
        
    def find_available_bodies(self, printinfo=True):
        if self.ask_for_6DOFinfo() == None:
            return None
        msg = _parser_comm(self.socket)
        if msg == None:
            return None
        valid = []
        #print msg['type']
        if msg['type']=='No more data' or msg['type']=='Event':
            return valid
            # raise Exception('No more data available. Check if the QTM server is running.')
        b = msg['bodies']
        
        if printinfo:
            print ('---Valid bodies in the workspace:---')
        for ii in range(len(b)): #foreach body in the configuration file, check if the body is in the workspace
            if not math.isnan(sum(b[ii])):
                valid.append(ii+1)
                if printinfo:
                    print('body nr. '  + str(ii+1) +  ' is valid\nx= ' + str(b[ii][0]) + '\ny= ' + str(b[ii][1]) + '\n')
        if len(valid) == 0 and printinfo:
            print "There are no valid bodies in the workspace."
        return [valid,msg]

    def get_updated_bodies(self):
        try:
            [valid_bodies,bodies_info] = self.find_available_bodies(printinfo=False)
        except:
            return 'off'
        if valid_bodies == None:
            return 'off'

        bodies_list = []
        for body in valid_bodies:
            # new_pose = Body(self,body,'g').getPose(bodies_info)
            new_pose = Body(self,body,bodytype='a').getPose(bodies_info)
            if new_pose == None or new_pose == 'off':
                return new_pose 
            bodies_list.append(new_pose)
        return bodies_list

class Body(object):

    def __init__(self, mocap, bodynr, bodytype=None):
        self.mocap = mocap
        self.bodynr = bodynr
        self.bodytype = bodytype
        self.ready = True

        if bodytype in ["ground","g"]:
            self.dtype = "xya"
        elif bodytype in ["air","a"] or (bodytype is None):
            self.dtype = "xyza"
        else:
            raise Exception("Invalid body type: must be either 'ground' (or 'g') or 'air' (or 'a')")

    def __repr__(self):
        dof = self.getPose()
        if self.dtype == "xya":
            return "Body nr {0} (ground vehicle) is at x = {1} mm, y = {2} mm, with yaw = {3} degrees".format(self.bodynr,dof['x'],dof['y'],dof['yaw'])
        elif self.dtype == "xyza":
            return "Body nr {0} (air vehicle) is at x = {1} mm, y = {2} mm, z = {3} mm, with angles of {4}, {5} and {6} degrees".format(self.bodynr,dof['x'],dof['y'],dof['z'],dof['a1'],dof['a2'],dof['a3'])

    def getPose(self,msg=None):
        socket = self.mocap.socket
        datatype = self.dtype
        if msg == None:
            self.mocap.ask_for_6DOFinfo()
            msg = _parser_comm(socket)
            if msg == None:
                return 'off'
        bodies=msg['bodies']
        timestamp=msg['timestamp']
        try:
            mybody = bodies[self.bodynr-1]
        except:
            return None
        x = mybody[0]/1000.
        y = mybody[1]/1000.
        z = mybody[2]/1000.
        roll = mybody[3]
        pitch = mybody[4]
        yaw = mybody[5]
        if datatype == 'xy':
            dof = {'x':x,'y':y}
        elif datatype == 'xya':
            dof = {'x':x,'y':y,'yaw':yaw}
        elif datatype == 'xyz':
            dof = {'x':x,'y':y,'z':z}
        elif datatype == 'xyza':
            dof = {'x':x,'y':y,'z':z,'roll':roll,'pitch':pitch,'yaw':yaw}
        else:
            raise Exception('Invalid data type request')
        dof['ts']=timestamp
        dof['id']=self.bodynr
        return dof


def _parser_comm(socket):
    msg = {'size':None, 'type':None, 'message':None, 'bodies':None, 'timestamp':None}
    try:
        msg_size_bytes = socket.recv(4) #receive the size of the package
    except:
        return None

    try:
        msg_size = struct.unpack('>l', msg_size_bytes)[0] #get the decimal message size
    except:
        return None

    msg['size']=msg_size
    msg_type_bytes = socket.recv(4) #receive the type of message
    msg_type_code = struct.unpack('>l', msg_type_bytes)[0] #get the message type code
    msg_types = ['Error','Command','XML','Data','No more data','C3D file','Event']

    try:
        msg_type = msg_types[msg_type_code]
    except:
        print "msg_type_code: " + str(msg_type_code)
        raise Exception('unexpexted type of message, see protocol documentation')
    msg['type']=msg_type
    if msg_type_code == 3:
        # parse the rest of the header
        header_timestamp = socket.recv(8)
        timestamp = struct.unpack('>q', header_timestamp)[0]
        header_framenumber = socket.recv(4)
        #print(struct.unpack('>l', header_framenumber)[0] )
        header_componentcount = socket.recv(4)
        nr_componentcount = struct.unpack('>l', header_componentcount)[0]
        for ii in range(nr_componentcount): #only one iteration if you request the current frame
            component_size = socket.recv(4)
            nr_comp_size = struct.unpack('>l', component_size)[0]
            component_type = socket.recv(4)
            nr_comp_type = struct.unpack('>l', component_type)[0]
            if nr_comp_type != 6:
                raise Exception('requested data type not manageable by the parser')
            body_count = struct.unpack('>l',socket.recv(4))[0]
            useless_info = socket.recv(4)
            bodies = [0]*body_count
            for jj in range(body_count):
                x = struct.unpack('>f',socket.recv(4))[0]
                y = struct.unpack('>f',socket.recv(4))[0]
                z = struct.unpack('>f',socket.recv(4))[0]
                a1 = struct.unpack('>f',socket.recv(4))[0]
                a2 = struct.unpack('>f',socket.recv(4))[0]
                a3 = struct.unpack('>f',socket.recv(4))[0]
                bodies[jj] = [x, y, z, a1, a2, a3]
            msg['bodies']=bodies
            msg['timestamp']=timestamp
    elif msg_type_code!=4:
        qtm_message_bytes = socket.recv(msg_size-8) #receive the message (size+type are 8B)
        qtm_message = qtm_message_bytes.decode("UTF-8")
        msg['message']=qtm_message
    return msg

#if __name__ == "__main__":
#    Qs = Mocap(info=1)
#    bodies = Qs.find_available_bodies(printinfo=1)
#    #pick the first valid body
#    while 1:
#        bodies = Qs.get_updated_bodies()
