'''
Copyright (C) 2024 Ian Sloat

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
'''

import bpy
from .. import MotionEngine as me
from .. import global_vars


class TriangulatePointsOperator(bpy.types.Operator):
    """Triangulate tracked points and place them in the current scene"""
    bl_idname = "motionengine.triangulate_points_operator"
    bl_label = "Triangulate points"
    bl_options = {'REGISTER', 'UNDO'}

    @classmethod
    def poll(cls, context):
        scene = context.scene
        properties = scene.motion_engine_ui_properties
        active_clip = properties.anchor_cam_selection
        return not global_vars.ui_lock_state and active_clip is not None

    def execute(self, context):
        scene = context.scene
        properties = scene.motion_engine_ui_properties
        active_clip = properties.anchor_cam_selection
        me.blender.OP_TriangulatePoints(self, active_clip)
        return {"FINISHED"}


CLASSES = [
    TriangulatePointsOperator
]
