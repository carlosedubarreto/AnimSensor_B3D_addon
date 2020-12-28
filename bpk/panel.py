import bpy
from . keyframe_connect import get_sensor #,check_sync #para pegar o numero do offset e usar no label

class Panel_kfp(bpy.types.Panel):
    bl_idname = "ANIMSENSOR_PT_Panel"
    bl_label = "AnumSensor"
    bl_category = "AnimSensor"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"


    def draw(self, context):
        
        layout = self.layout
        layout.label(text=" Sensor:")
        row = layout.row()
        row.operator('view3d.get_sensor', text="Get Sensor Data")
        # row.operator('import_test.some_data', text='Open File')
        