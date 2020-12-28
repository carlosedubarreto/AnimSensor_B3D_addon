# C:/Users/Pichau/AppData/Local/Programs/Python/Python37/python.exe

import argparse
import zmq
import time
import numpy as np
from HIMUServer import HIMUServer
import json
from datetime import datetime, timedelta 
# from . HIMUServer import HIMUServer
# global time_in,time_in_neg,time_in_ant,time_in_neg_ant,z_total

time_in =datetime.now()
time_in_neg =datetime.now()
#An example of listener implementation.




class SimplePrintListener:
    # global time_in,time_in_neg,time_in_ant,time_in_neg_ant,z_total
    j=1
    # array = np.empty([3, 3])
    sample_rate = 20 #50ms = 20Hz
    acc = np.empty([1, 3])
    omega = np.empty([1, 3])
    mag = np.empty([1, 3])
    # print('ini: ',array)

    def __init__(self, serverInstance):
        self.__server = serverInstance
    

    def notify (self, sensorData):
        # global time_in,time_in_neg,time_in_ant,time_in_neg_ant,z_total

        #Customize the notify method in order to elaborate data
		# sensorData contains String values (see HIMUServer.__extractSensorData())
        
        # HIMUServer.printSensorsData(sensorData)
		#for a string-to-float conversion, try HIMUServer.strings2Floats()
        
        # sensor=3
        # x_value = sensorData[0][sensor][0]
        # i = x_value
        # i = sensorData
        
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

        # json_string = json.dumps(list_of_lists)
        # print('json',json_string)
        # print(type(json_string))
        # print(type(json_string[0]))
        print('antes count')
        
        

        if self.j==1 :
            # self.array = np.asarray(list_of_lists)
            self.acc = np.asarray(list_of_lists)[0]
            self.omega = np.asarray(list_of_lists)[1]
            self.mag = np.asarray(list_of_lists)[2]

            # print('shape: ',self.array.shape,' if: ',self.array)
            print('if acc: ',self.acc)
            print('if omega: ',self.omega)
            print('if mag: ',self.mag)
            self.j = self.j +1 
        elif self.j < 10 :
            # self.array = np.vstack([self.array,np.asarray(list_of_lists)])
            self.acc = np.vstack([self.acc,np.asarray(list_of_lists)[0]])
            self.omega = np.vstack([self.omega,np.asarray(list_of_lists)[1]])
            self.mag = np.vstack([self.mag,np.asarray(list_of_lists)[2]])

            print('elif acc: ',self.acc)
            print('elif omega: ',self.omega)
            print('elif mag: ',self.mag)


            # print('elif: ',self.array)
            # array_list = array.tolist()
            # json_string = json.dumps(array_list)
            self.j = self.j +1
        else:
            self.array = np.vstack([self.array,np.asarray(list_of_lists)])
            j = 1

            print('else: ',self.array)

            # array_list = array.tolist()
            # json_string = json.dumps(array_list)

        # np.vstack([a,b])

        # print('valor i :',i)
        # print('tipo list  :',type(list_of_lists))
        # print('valor list_of_lists :',list_of_lists)
        # user_msg = input("Please type a message to send: ")
        # msg = str(i).encode('utf-8')
        # send_data =[]
        # ini_time = datetime.now()


        # # print('ini: ',ini_time) 
        # json_string = json.dumps(list_of_lists)
        # round_x = round(float(list_of_lists[0][0]),1)
        # round_y = round(float(list_of_lists[0][1]),1)
        # round_z = round(float(list_of_lists[0][2]),1)
        # # print('list x: ',round_x,' y: ',round_y,'z: ',round_z)
        # send_data.append(round_x)
        # send_data.append(round_y)
        # send_data.append(round_z)

        # if abs(round_z) > 0.0:
        #     z_total = z_total + round_z
        #     z_total = round(z_total,1)

        # if round_z >= 0 :
        #     pstv_z = 1
        # else:
        #     pstv_z = 0
        # print('pstv_z: ',pstv_z,'abs z',abs(round_z),'r_z',round_z, 'z_total: ',z_total)

        # if pstv_z ==1 and abs(round_z) >= 0.5 :
        #     time_in = datetime.now() 
        #     delta = time_in-time_in_neg_ant
        #     if delta.seconds > 0 :
        #         sent = 1
        #     else:
        #         sent = 0
        #     print('time_in: ',time_in,'delta: ',delta.seconds,' ',delta.microseconds,'sent: ',sent)

        #     if delta.seconds > 0 :
        #         msg = str(json_string).encode('utf-8')
        #         # publish data
        #         socket.send_multipart([topic, msg])  # 'test'.format(i)
        #         # print("On topic {}, send data: {}".format(topic, msg))
        #     time_in_ant = datetime.now()

        # if pstv_z ==0  and abs(round_z) >= 0.5 :
        #     time_in_neg = datetime.now()
        #     delta_neg = time_in_neg-time_in_ant
        #     if delta_neg.seconds > 0 :
        #         sent = 1
        #     else:
        #         sent = 0
        #     print('time_in_neg: ',time_in_neg,'delta: ',delta_neg.seconds,' ',delta_neg.microseconds,'sent: ',sent)

        #     if delta_neg.seconds > 0 :
        #         msg = str(json_string).encode('utf-8')
        #         # publish data
        #         socket.send_multipart([topic, msg])  # 'test'.format(i)
        #         # print("On topic {}, send data: {}".format(topic, msg))
        #     time_in_neg_ant = datetime.now()


        







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

# if 'time_in' not in globals():
#     time_in = datetime.now()

# if 'time_in_neg' not in globals():
#     time_in_neg = datetime.now()

# if 'time_in_ant' not in globals():
#     time_in_ant = datetime.now()

# if 'time_in_neg_ant' not in globals():
#     time_in_neg_ant = datetime.now()

# if 'z_total' not in globals():
#     z_total = 0



if 'j' not in globals():
    j = 1
print('j: ',j)

#HIMUServer instance:
myHIMUServer = HIMUServer()

#Creating listener and adding it to the server instance:
myListener = SimplePrintListener(myHIMUServer)
myHIMUServer.addListener(myListener)

#Change the timeout (in seconds) :
myHIMUServer.timeout = 10

#Launch acquisition via TCP on port 2055:
myHIMUServer.start("TCP", 2055)

# myHIMUServer.start("FILE", "HIMU-filetest.csv")

# keep sending messages until program interruption
# while True:
#     # user_msg = input("Please type a message to send: ")
#     msg = str(i).encode('utf-8')
#     # publish data
#     socket.send_multipart([topic, msg])  # 'test'.format(i)
#     print("On topic {}, send data: {}".format(topic, msg))
#     # 30 fps
#     time.sleep(1/30)

#     # i += 2
#     # if i > 100:
#         # i = 0



