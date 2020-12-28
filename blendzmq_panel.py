# ##### BEGIN GPL LICENSE BLOCK #####
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software Foundation,
#  Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
#
# ##### END GPL LICENSE BLOCK #####

# Copyright (c) Stef van der Struijk <stefstruijk@protonmail.ch>

import bpy
from bpy.types import Panel
from bpy.props import (#StringProperty,
                       BoolProperty,
                      IntProperty,
                      FloatProperty,
#                       FloatVectorProperty,
#                       EnumProperty,
                    #    PointerProperty,
                       )
from bpy.types import (Panel,
                       Operator,
                       AddonPreferences,
                       PropertyGroup,
                       )

global percent_render_var
# Draw Socket panel in Toolbar
class BLENDZMQ_PT_zmqConnector(Panel):
    """Interface to set and (dis)connect the ZeroMQ socket; Found in side panel of the 3D view
     (open by pressing `n` or dragging `<`)"""

    bl_label = "AnimSensor"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_context = "objectmode"
    bl_category = "AnimSnsr"
    # bl_options = {'DEFAULT_CLOSED'}

    def draw(self, context):
        layout = self.layout
        preferences = context.preferences.addons[__package__].preferences
        socket_settings = context.window_manager.socket_settings

        scene = context.scene
        # mytool = scene.my_tool
        percentage = scene.percentage
        percent_render_var = percentage.percentage

        #   check if pyzmq is installed; will fail with an ImportError if not
        # if installed, will show interaction options: (dis)connect socket and whether to use dynamic object selection
        try:
            import zmq

            # connection information
            row = layout.row()
            #   per Blender session ip and port number
            # row.prop(socket_settings, "socket_ip", text="ip")
            # row.prop(socket_settings, "socket_port", text="port")
            #   Add-on preference based ip and port number
            row.prop(preferences, "socket_ip", text="ip")
            row.prop(preferences, "socket_port", text="port")

            # whether if previous selection is remembered or always use current selected objects
            layout.prop(socket_settings, "dynamic_object")
            layout.prop(percentage, "percentage", text="Multiplier")
            # if our socket hasn't connected yet
            if not socket_settings.socket_connected:
                layout.operator("socket.connect_subscriber")  # , text="Connect Socket"
            else:
                layout.operator("socket.connect_subscriber", text="Disconnect Sensor")
                layout.prop(socket_settings, "msg_received")
            
            # row = layout.row()
            # row.operator("socket.cancelaTimer", text='cancela timer')
            

        # if not installed, show button that enables & updates pip, and pip installs pyzmq
        except ImportError:
            # keep track of how our installation is going
            install_props = context.window_manager.install_props

            # button: enable pip and install pyzmq if not available
            layout.operator("pipzmq.pip_pyzmq")
            # show status messages (kinda cramped)
            layout.prop(install_props, "install_status")

from bpy.props import (#StringProperty,
                       BoolProperty,
                      IntProperty,
                      FloatProperty,
#                       FloatVectorProperty,
#                       EnumProperty,
                    #    PointerProperty,
                       )
from bpy.types import (Panel,
                       Operator,
                       AddonPreferences,
                       PropertyGroup,
                       )

# class MySettings(PropertyGroup):
#     bool_sound : BoolProperty(
#         name="Enable or Disable",
#         description="Enable or disable sound",
#         default = False
#         )
class MySettingsPerc(PropertyGroup):
    percentage: IntProperty(name="percent", description="Sensor Number", default=4)

def register():
    bpy.utils.register_class(BLENDZMQ_PT_zmqConnector,MySettingsPerc)


def unregister():
    bpy.utils.unregister_class(BLENDZMQ_PT_zmqConnector,MySettingsPerc)


if __name__ == "__main__":
    register()