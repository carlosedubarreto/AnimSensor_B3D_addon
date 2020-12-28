# # C:/Users/Pichau/AppData/Local/Programs/Python/Python37/python.exe
# import sys,time
# # insert at 1, 0 is the script path (or '' in REPL)
# sys.path.insert(1, '/Downloads/0_Projetos/2020/0_addon/AnimSensor')


import argparse
import zmq
import time
import numpy as np
from HIMUServer import HIMUServer
import json
from datetime import datetime, timedelta 
from skinematics.sensors.manual import MyOwnSensor
# from . HIMUServer import HIMUServer

class SimplePrintListener:
    # global time_in,time_in_neg,time_in_ant,time_in_neg_ant,z_total
    j=1
    # array = np.empty([3, 3])
    rate = 20 #50ms = 20Hz
    # acc = np.empty([1, 3])
    # omega = np.empty([1, 3])
    # mag = np.empty([1, 3])
    # print('ini: ',array)

    def __init__(self, serverInstance):
        self.__server = serverInstance
    

    def notify (self, sensorData):
       
        try:	
            for acquisition in sensorData:
                i = 1;
                list_of_lists = []
                for sensorAcq in acquisition :
                    # print('Sensor' + str(i) +  ": " + str(sensorAcq))
                    # list_of_lists.append(str(sensorAcq))
                    list_of_lists.append(sensorAcq)

                    i+=1
        except Exception as ex:
            print(str(ex))

        if len(list_of_lists) == 3:
            if self.j==1 :
                # self.array = np.asarray(list_of_lists)
                self.acc = np.asarray(list_of_lists)[0].astype(float)
                self.omega = np.asarray(list_of_lists)[1].astype(float)
                self.mag = np.asarray(list_of_lists)[2].astype(float)

                # print('shape: ',len(list_of_lists))
                # print('1if acc: ',self.acc)
                # print('1if omega: ',self.omega)
                # print('1if mag: ',self.mag)

                self.j = self.j +1 
            else: 
                # print('shape: ',len(list_of_lists))
                # self.array = np.vstack([self.array,np.asarray(list_of_lists)])
                self.acc = np.vstack([self.acc,np.asarray(list_of_lists)[0]]).astype(float)
                self.omega = np.vstack([self.omega,np.asarray(list_of_lists)[1]]).astype(float)
                self.mag = np.vstack([self.mag,np.asarray(list_of_lists)[2]]).astype(float)

                # print('2elif acc: ',self.acc)
                # print('2elif omega: ',self.omega)
                # print('2elif mag: ',self.mag)
                initial_orientation = np.array([[1,0,0],
                                     [0,0,-1],
                                     [0,1,0]])

                # print('elif: ',self.array)
                # array_list = array.tolist()
                # json_string = json.dumps(array_list)
                if self.j < 3 :
                    self.j = self.j +1
                else:
                    in_data = {'rate':self.rate,
                            'acc': self.acc,
                            'omega':self.omega,
                            'mag':self.mag}
                    my_sensor = MyOwnSensor(in_file='My own 123 sensor.', in_data=in_data, q_type = 'mahony', R_init=initial_orientation)

                    # q_types = 'analytical'
                    # q_types = 'kalman'
                    # q_types = 'mahony'
                    # q_types = 'madgwick'
       
                    # print('pos: ',my_sensor.pos)
                    position = my_sensor.pos[len(my_sensor.pos)-1]#.astype(str)
                    # print('pos ult:',position) 
                    # print('vel: ',my_sensor.vel)
                    # position = position *70                
                    array_list = position.tolist()
                    json_string = json.dumps(array_list)
                    msg = str(json_string).encode('utf-8')
                    # publish data
                    socket.send_multipart([topic, msg])  # 'test'.format(i)
                    print("On topic {}, send data: {}".format(topic, msg))

                    self.j = 1

        
            # array_list = array.tolist()
            # json_string = json.dumps(array_list)

            # msg = str(json_string).encode('utf-8')
            # # publish data
            # socket.send_multipart([topic, msg])  # 'test'.format(i)
            # print("On topic {}, send data: {}".format(topic, msg))


# def main(ip="127.0.0.1", port="5550"):
ip="127.0.0.1"
port="5550"
# ZMQ connection
url = "tcp://{}:{}".format(ip, port)
ctx = zmq.Context()
socket = ctx.socket(zmq.PUB)
socket.connect(url)  # publisher connects to subscriber
print("Pub connected to: {}\nSending data...".format(url))

i = 0
topic = 'foo'.encode('ascii')

#HIMUServer instance:
myHIMUServer = HIMUServer()

#Creating listener and adding it to the server instance:
myListener = SimplePrintListener(myHIMUServer)
myHIMUServer.addListener(myListener)

#Change the timeout (in seconds) :
myHIMUServer.timeout = 10

#Launch acquisition via TCP on port 2055:
myHIMUServer.start("TCP", 2055)
