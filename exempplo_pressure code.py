import bpy
from bpy.props import BoolProperty, IntProperty

class WM_OT_pressure_simulate(bpy.types.Operator):

    bl_description = "simulate pressure"
    bl_idname = "wm.pressure_simulate"
    bl_label = "pressure"
    numerator : IntProperty(default=156)
    def modal (self, context, event):

        scene = context.scene
        obj = context.object
        wm = context.window_manager

        if not wm.modal_pressure or obj.mode != 'SCULPT':
            return self.cancel(context)

        if event.type == 'TIMER':
            ps = scene.tool_settings.unified_paint_settings
            ps.size = event.mouse_x / self.numerator
            print(event.mouse_x / 156)

        return {"PASS_THROUGH"}


    def execute(self, context):
        wm = context.window_manager
        self._timer = wm.event_timer_add(0.01, window=context.window)
        wm.modal_handler_add(self)
        return {"RUNNING_MODAL"}

    def cancel (self, context):
        wm = context.window_manager
        wm.event_timer_remove(self._timer)
        wm.modal_pressure = False
        return {'CANCELLED'}

def toggle_pressure(self, context):
    print("Pressure Sim Mode ", self.modal_pressure)
    if self.modal_pressure:
        bpy.ops.wm.pressure_simulate()
        #bpy.ops.wm.pressure_simulate(numerator=20)

def add_button(self, context):
    layout = self.layout
    wm = context.window_manager
    if context.object.mode == 'SCULPT':
        layout.prop(wm, "modal_pressure", toggle=True)

def register():
    bpy.utils.register_class(WM_OT_pressure_simulate)

    bpy.types.WindowManager.modal_pressure = BoolProperty(
                                        name="Pressure",
                                        description="Toggle Pressure",
                                        default=False,
                                        update=toggle_pressure)

    bpy.types.VIEW3D_HT_header.prepend(add_button)

if __name__ == "__main__":
    register()