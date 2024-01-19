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

from ..MotionEngine import MEPython
from .. import global_vars
from .. import utils


class TriangulatePointsOperator(bpy.types.Operator):
    """Triangulate tracked points and place them in the current scene"""
    bl_idname = "motionengine.triangulate_points_operator"
    bl_label = "Triangulate points"

    def execute(self, context):
        if not global_vars.ui_lock_state:
            scene = context.scene
            properties = scene.motion_engine_ui_properties

            clips = bpy.data.movieclips

            pose_tracks_list = properties.me_ui_prop_pose_clip_collection

            cam_matrices = []

            dist_vectors = []

            for clip in clips:
                cam_mtx, dist_vector = utils.get_cv_camera_intrinsics(clip)
                cam_matrices.append(cam_mtx)
                dist_vectors.append(dist_vector)

            MEPython.triangulate_points(
                clips,
                pose_tracks_list,
                cam_matrices,
                dist_vectors,
                scene,
                bpy.data
            )

        return {"FINISHED"}


CLASSES = [
    TriangulatePointsOperator
]
