import argparse
import zmq
import time
from HIMUServer import HIMUServer
import json
# from . HIMUServer import HIMUServer

#An example of listener implementation.
class SimplePrintListener:
    def __init__(self, serverInstance):
        self.__server = serverInstance
    

    def notify (self, sensorData):
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


        # print('valor i :',i)
        # print('tipo list  :',type(list_of_lists))
        # print('valor list_of_lists :',list_of_lists)
        # user_msg = input("Please type a message to send: ")
        # msg = str(i).encode('utf-8')
        json_string = json.dumps(list_of_lists)
        msg = str(json_string).encode('utf-8')
        # publish data
        socket.send_multipart([topic, msg])  # 'test'.format(i)
        print("On topic {}, send data: {}".format(topic, msg))
        # 30 fps
        # time.sleep(1/30)

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



