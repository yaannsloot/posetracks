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
from .. import utils


class SolveCamerasOperator(bpy.types.Operator):
    """Solve cameras"""
    bl_idname = "motionengine.solve_cameras_operator"
    bl_label = "Solve cameras"

    def execute(self, context):
        scene = context.scene
        properties = scene.motion_engine_ui_properties
        solution_id = me.crypto.random_sha1()

        if global_vars.ui_lock_state:
            return {'FINISHED'}

        active_clip = properties.me_ui_prop_anchor_cam_selection

        if active_clip is None:
            return {'FINISHED'}

        anchor_cam = utils.prepare_camera_for_clip(active_clip, context)

        clips = [active_clip] + [clip for clip in bpy.data.movieclips if clip is not active_clip]

        if len(clips) < 2:
            return {'FINISHED'}

        t_data = []
        cam_Kk = []

        for clip in clips:
            t_data.append(utils.get_clip_tracking_data(clip, filter_locked=True))
            cam_Kk.append(utils.get_clip_Kk(clip))

        cam_transforms = me.tracking.solve_static_set(t_data, cam_Kk)

        flip_mtx = [
            [1, -1, -1, 1],
            [-1, 1, 1, -1],
            [-1, 1, 1, -1],
            [1, 1, 1, 1]
        ]

        if all(tf.is_identity() for tf in cam_transforms):
            self.report({'ERROR'}, 'Failed to find solution for camera transforms')
            return {'FINISHED'}

        for child in anchor_cam.children:
            child_tf = child.matrix_world
            child.parent = None
            child.matrix_world = child_tf

        anchor_tf = anchor_cam.matrix_world
        anchor_cam.parent = None
        anchor_cam.matrix_world = anchor_tf

        for i in range(len(cam_transforms)):
            tf = cam_transforms[i]
            clip = clips[i + 1]
            if tf.is_identity():
                print(f"SKIPPED '{clip.name}': CALCULATED TF IS IDENTITY!")
                continue
            clip_cam = utils.prepare_camera_for_clip(clip, context)
            clip_cam.parent = None
            clip_cam.parent = anchor_cam
            clip_cam.data['solution_id'] = solution_id
            tf.invert()
            tf = tf.to4x4()
            blend_mtx = mathutils.Matrix()
            for r in range(4):
                for c in range(4):
                    blend_mtx[r][c] = tf[r, c] * flip_mtx[r][c]
            blend_mtx.translation *= properties.me_ui_prop_solution_scale
            clip_cam.matrix_world = anchor_cam.matrix_world @ blend_mtx

        anchor_cam.data['solution_id'] = solution_id

        if any(tf.is_identity() for tf in cam_transforms):
            self.report({'WARNING'}, 'Some views failed to solve. Check the console for more information.')
        else:
            self.report({'INFO'}, 'Solver finished.')

        return {'FINISHED'}


def get_xy_plane_pos(origin, proj_point):
    """
    Not currently in use but may help get a scaling distance against another
    coplanar tag next to the origin
    origin = camera position
    proj_point = position of coplanar bundle somewhere in 3d space
    """
    xy_bundle_x = origin[2] * (proj_point[0] - origin[0])
    xy_bundle_x /= proj_point[2] - origin[2]
    xy_bundle_x = origin[0] - xy_bundle_x
    xy_bundle_y = origin[2] * (proj_point[1] - origin[1])
    xy_bundle_y /= proj_point[2] - origin[2]
    xy_bundle_y = origin[1] - xy_bundle_y
    return mathutils.Vector((xy_bundle_x, xy_bundle_y, 0))


class SolveCameraFromTagOperator(bpy.types.Operator):
    """Solve view for current clip using selected tag as the origin"""
    bl_idname = "motionengine.solve_camera_from_tag_operator"
    bl_label = "Solve camera"

    def execute(self, context):
        scene = context.scene
        properties = scene.motion_engine_ui_properties
        current_clip = context.edit_movieclip
        
        if current_clip is not None:
            active_track = current_clip.tracking.tracks.active
        else:
            active_track = None

        if active_track is None or not utils.is_valid_tag_name(active_track.name):
            return {'FINISHED'}

        scene_cam = utils.prepare_camera_for_clip(current_clip, context)
        clip_info = utils.ClipInfo(current_clip)
        marker = active_track.markers.find_frame(clip_info.scene_to_clip(scene.frame_current), exact=False)
        tag = utils.marker_to_tag(marker, clip_info.clip_size, False)
        cam_Kk = utils.get_clip_Kk(current_clip)
        cam_T = me.tracking.solve_camera_with_tag(tag, cam_Kk)

        if cam_T.is_identity():
            self.report({'ERROR'}, 'Failed to solve pose for selected tag')
            return {'FINISHED'}

        flip_mtx = [
            [1, -1, -1, 1],
            [-1, 1, 1, -1],
            [-1, 1, 1, -1],
            [1, 1, 1, 1]
        ]

        cam_T = cam_T.to4x4()
        blend_mtx = mathutils.Matrix()

        for r in range(4):
            for c in range(4):
                blend_mtx[r][c] = cam_T[r, c] * flip_mtx[r][c]

        scene_cam.delta_location = (0, 0, 0)
        scene_cam.delta_rotation_euler = (0, 0, 0)
        scene_cam.delta_scale = (1, 1, 1)

        scene_cam.matrix_world = mathutils.Matrix.Identity(4)

        context.view_layer.update()

        orig_matrix = scene_cam.matrix_world.copy()

        scene_cam.matrix_world = blend_mtx @ orig_matrix.inverted()

        if active_track.has_bundle:
            # Doesn't scale perfectly but should work for special cases like this
            bundle_pos = active_track.bundle
            new_scale = abs(blend_mtx.translation.length / bundle_pos.length)
            scale_mtx = mathutils.Matrix.Scale(new_scale, 4)
            scene_cam.matrix_world = (blend_mtx @ scale_mtx) @ orig_matrix.inverted()
            bpy.ops.clip.select_all(action='DESELECT')
            active_track.select = True
            bpy.ops.clip.set_origin(use_median=False)

        properties.me_ui_prop_anchor_cam_selection = current_clip
        scene_cam.data['solution_id'] = active_track.name

        return {'FINISHED'}


CLASSES = [
    SolveCamerasOperator,
    SolveCameraFromTagOperator
]
