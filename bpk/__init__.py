# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTIBILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
# General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <http://www.gnu.org/licenses/>.

# Beta 0.5
# - toggle frame_preview range
# - add possibility to render frame preview range
# - fix to load bookmark from timeline with name different from 'My Timeline'
# - auto-clean previous render (for each View)

bl_info = {
    "name" : "AnimSensor",
    "author" : "Carlos Barreto",
    "description" : "",
    "blender" : (2, 80, 0),
    "version" : (0, 0, 1),
    "location" : "View3D",
    "warning" : "",
    "category" : "Generic"
}
import bpy
# from . keyframe_pro_client import KeyframeProClient
from . keyframe_connect import get_sensor
from . panel import Panel_kfp


classes = (Panel_kfp,get_sensor)

def register():
    from bpy.utils import register_class
    for cls in classes:
        register_class(cls)

def unregister():
    from bpy.utils import unregister_class
    for cls in reversed(classes):
        unregister_class(cls) 
