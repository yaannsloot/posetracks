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

from ..MotionEngine import MEPython
from .. import global_vars
from .. import utils

class SolveCamerasOperator(bpy.types.Operator):
    """Solve cameras"""
    bl_idname = "motionengine.solve_cameras_operator"
    bl_label = "Solve cameras"

    def execute(self, context):
        if not global_vars.ui_lock_state:
            scene = context.scene
            properties = scene.motion_engine_ui_properties

            clips = bpy.data.movieclips

            current_clip = context.edit_movieclip

            pose_tracks_a = properties.get_clip_pose_tracks(current_clip)["Pose 0"]

            other_clip = None

            for clip in clips:
                if clip != current_clip:
                    other_clip = clip
                    break

            pose_tracks_b = properties.get_clip_pose_tracks(other_clip)["Pose 0"]

            if pose_tracks_a is None or pose_tracks_b is None:
                return {'FINISHED'}

            cam_mtx_a, dist_coeffs_a = utils.get_cv_camera_intrinsics(current_clip)
            cam_mtx_b, dist_coeffs_b = utils.get_cv_camera_intrinsics(other_clip)

            scene_cam_a = scene.objects[current_clip.name]
            scene_cam_b = scene.objects[other_clip.name]

            if scene_cam_a is None or scene_cam_b is None or scene_cam_a.type != 'CAMERA' or scene_cam_b.type != 'CAMERA':
                return {'FINISHED'}

            local_transform_b = MEPython.solve_cameras(
                current_clip,
                other_clip,
                pose_tracks_a,
                pose_tracks_b,
                cam_mtx_a,
                cam_mtx_b,
                dist_coeffs_a,
                dist_coeffs_b
            )

            scene_cam_b.matrix_world = mathutils.Matrix(local_transform_b) @ scene_cam_a.matrix_world

        return {'FINISHED'}


CLASSES = [
    SolveCamerasOperator
]
