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
import mathutils

from .. import MotionEngine as me
from .. import global_vars
from ..utils import get_clip_tracking_data, get_clip_Kk, prepare_camera_for_clip


class SolveCamerasOperator(bpy.types.Operator):
    """Solve cameras"""
    bl_idname = "motionengine.solve_cameras_operator"
    bl_label = "Solve cameras"

    def execute(self, context):
        if global_vars.ui_lock_state:
            return {'FINISHED'}

        anchor_cam = None

        space_type = context.area.type
        if space_type == 'CLIP_EDITOR':
            current_clip = context.edit_movieclip
        elif space_type == 'VIEW_3D':
            active_obj = context.active_object
            if active_obj.type != 'CAMERA' or bpy.data.movieclips.get(active_obj.data.name) is None:
                return {'FINISHED'}
            current_clip = bpy.data.movieclips.get(active_obj.data.name)
            anchor_cam = active_obj
        else:
            return {'FINISHED'}

        scene = context.scene

        clips = [current_clip] + [clip for clip in bpy.data.movieclips if clip is not current_clip]

        if len(clips) < 2:
            return {'FINISHED'}

        t_data = []
        cam_Kk = []

        for clip in clips:
            t_data.append(get_clip_tracking_data(clip))
            cam_Kk.append(get_clip_Kk(clip))

        cam_transforms = me.tracking.solve_static_set(t_data, cam_Kk)

        flip_mtx = [
            [1, -1, -1, 1],
            [-1, 1, 1, -1],
            [-1, 1, 1, -1],
            [1, 1, 1, 1]
        ]

        for i in range(len(cam_transforms)):
            tf = cam_transforms[i]
            clip = clips[i + 1]
            if tf.is_identity():
                self.debug(f"TF IS IDENTITY, {str(tf.to4x4())}")
                continue
            if anchor_cam is None:
                anchor_cam = prepare_camera_for_clip(current_clip, context)
            clip_cam = prepare_camera_for_clip(clip, context)
            tf.invert()
            tf = tf.to4x4()
            blend_mtx = mathutils.Matrix()
            for r in range(4):
                for c in range(4):
                    blend_mtx[r][c] = tf[r, c] * flip_mtx[r][c]
            clip_cam.matrix_world = anchor_cam.matrix_world @ blend_mtx

        self.debug("FINISHED")

        return {'FINISHED'}

    def debug(self, msg):
        self.report({'INFO'}, str(msg))


CLASSES = [
    SolveCamerasOperator
]
