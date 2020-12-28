# import datetime
# import os
import bpy
from bpy import context
# from bpy.app.handlers import persistent
# from . keyframe_pro_client import KeyframeProClient
from . HIMUServer import HIMUServer
# from . panel import Panel_kfp

global x_value
x_value =0
class SimplePrintListener:
    global x_value
    def __init__(self, serverInstance):
        self.__server = serverInstance
    
    def notify (self, sensorData):
        global x_value
		#Customize the notify method in order to elaborate data
		# sensorData contains String values (see HIMUServer.__extractSensorData())
        # HIMUServer.printSensorsData(sensorData)
		#for a string-to-float conversion, try HIMUServer.strings2Floats()
        # print('data :',sensorData)
        # print('tamanho sensordata :',len(sensorData), 'tam sensor[0] :', len(sensorData[0]),'tam sensor[0][0] :', len(sensorData[0][0]))
        # print('x[0][0]', HIMUServer.strings2Floats(sensorData[0][0][0]))
        sensor=9
        x_value = sensorData[0][sensor][0]
        print ('valor:',x_value)
        # print('x[0][0]',sensorData[0][sensor][0], 'y :',sensorData[0][sensor][1],'z :',sensorData[0][sensor][2])
        #HIMUServer.strings2Floats(sensorData)


    



from bpy.props import IntProperty, FloatProperty
class get_sensor(bpy.types.Operator):
    global x_value
    bl_idname = "view3d.get_sensor"
    bl_label = "Get Sensor"
    bl_description = "Get Sensor Information"

    first_mouse_x: IntProperty()
    first_value: FloatProperty()

    def execute(self, context):
        global x_value
        myHIMUServer = HIMUServer(bufferSize = 1024, timeout = 10, separatorIndex = 0)
        
        myListener = SimplePrintListener(myHIMUServer)
        myHIMUServer.addListener(myListener)
        
        myHIMUServer.start("TCP", 2055)
        # print ('valor:',x_value)
        return{'FINISHED'}

    def modal(self, context, event):
    
        global x_value
        myHIMUServer = HIMUServer(bufferSize = 1024, timeout = 10, separatorIndex = 0)
        
        myListener = SimplePrintListener(myHIMUServer)
        myHIMUServer.addListener(myListener)
        
        myHIMUServer.start("TCP", 2055)

        if event.type == 'MOUSEMOVE':
            delta = self.first_mouse_x - event.mouse_x
            context.object.location.x = self.first_value + delta * 0.01

        elif event.type == 'LEFTMOUSE':
            return {'FINISHED'}

        elif event.type in {'RIGHTMOUSE', 'ESC'}:
            context.object.location.x = self.first_value
            return {'CANCELLED'}

        return {'RUNNING_MODAL'}

    def invoke(self, context, event):
        if context.object:
            self.first_mouse_x = event.mouse_x
            self.first_value = context.object.location.x

            context.window_manager.modal_handler_add(self)
            return {'RUNNING_MODAL'}
        else:
            self.report({'WARNING'}, "No active object, could not finish")
            return {'CANCELLED'}