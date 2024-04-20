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


class FilterTracksOperator(bpy.types.Operator):
    """Refine track points with a kalman filter"""
    bl_idname = "motionengine.filter_tracks_operator"
    bl_label = "Filter tracks"

    def execute(self, context):
        scene = context.scene
        properties = scene.motion_engine_ui_properties
        g1, s1 = properties.gen_objs.new()
        g2, s2 = properties.gen_objs.new()
        g3, s3 = properties.gen_objs.new()

        g_keys = properties.gen_objs.keys()

        print(g_keys)

        if not global_vars.ui_lock_state:
            current_clip = context.edit_movieclip

            #me.filter_2D(current_clip, 1e-2, 5e-4)

            me.z_thresh_2D(current_clip)

        return {"FINISHED"}


CLASSES = [
    FilterTracksOperator
]
