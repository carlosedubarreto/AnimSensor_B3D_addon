import argparse
import zmq
import time

from . HIMUServer import HIMUServer


# def main(ip="127.0.0.1", port="5550"):
#     # ZMQ connection
#     url = "tcp://{}:{}".format(ip, port)
#     ctx = zmq.Context()
#     socket = ctx.socket(zmq.PUB)
#     socket.connect(url)  # publisher connects to subscriber
#     print("Pub connected to: {}\nSending data...".format(url))

#     i = 0
#     topic = 'foo'.encode('ascii')
    
    
    
#     #HIMUServer instance:
#     myHIMUServer = HIMUServer()

#     #Creating listener and adding it to the server instance:
#     myListener = SimplePrintListener(myHIMUServer)
#     myHIMUServer.addListener(myListener)

#     #Change the timeout (in seconds) :
#     myHIMUServer.timeout = 10

#     #Launch acquisition via TCP on port 2055:
#     myHIMUServer.start("TCP", 2055)

#     # keep sending messages until program interruption
#     # while True:
#     #     # user_msg = input("Please type a message to send: ")
#     #     msg = str(i).encode('utf-8')
#     #     # publish data
#     #     socket.send_multipart([topic, msg])  # 'test'.format(i)
#     #     print("On topic {}, send data: {}".format(topic, msg))
#     #     # 30 fps
#     #     time.sleep(1/30)

#     #     # i += 2
#     #     # if i > 100:
#     #         # i = 0


#HIMUServer instance:
myHIMUServer = HIMUServer()

#Creating listener and adding it to the server instance:
myListener = SimplePrintListener(myHIMUServer)
myHIMUServer.addListener(myListener)

#Change the timeout (in seconds) :
myHIMUServer.timeout = 10

#Launch acquisition via TCP on port 2055:
myHIMUServer.start("TCP", 2055)

#An example of listener implementation.
class SimplePrintListener:
    def __init__(self, serverInstance):
        self.__server = serverInstance
    		
    def notify (self, sensorData):
		#Customize the notify method in order to elaborate data
		# sensorData contains String values (see HIMUServer.__extractSensorData())
        
        # HIMUServer.printSensorsData(sensorData)
		#for a string-to-float conversion, try HIMUServer.strings2Floats()
        
        sensor=9
        x_value = sensorData[0][sensor][0]
        i = x_value
        print('valor i :',i)
        # # user_msg = input("Please type a message to send: ")
        # msg = str(i).encode('utf-8')
        # # publish data
        # socket.send_multipart([topic, msg])  # 'test'.format(i)
        # print("On topic {}, send data: {}".format(topic, msg))
        # # 30 fps
        # time.sleep(1/30)

        # # i += 2
        # # if i > 100:
        # #     i = 0




# if __name__ == "__main__":
#     # command line arguments
#     parser = argparse.ArgumentParser()

#     # publisher setup commandline arguments
#     parser.add_argument("--ip", default="127.0.0.1",
#                         help="IP (e.g. 192.168.x.x) of where to pub to; Default: 127.0.0.1 (local)")
#     parser.add_argument("--port", default="5550",
#                         help="Port of where to pub to; Default: 5550")

#     args, leftovers = parser.parse_known_args()
#     print("The following arguments are used: {}".format(args))
#     print("The following arguments are ignored: {}\n".format(leftovers))

#     main(**vars(args))


