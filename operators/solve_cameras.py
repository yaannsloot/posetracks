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


def clip_list(self, context):
    return [(clip.name, clip.name, f'Set anchor view to {clip.name}') for clip in bpy.data.movieclips]


class SolveCamerasOperator(bpy.types.Operator):
    """Solve cameras"""
    bl_idname = "motionengine.solve_cameras_operator"
    bl_label = "Solve cameras"
    bl_options = {'REGISTER', 'UNDO'}

    views: bpy.props.EnumProperty(
        items=clip_list,
        name="Anchor View",
        description="Clip to use as anchor view"
    )

    solution_scale: bpy.props.FloatProperty(
        name="Solution scale",
        description="Sets solution scale for cameras",
        default=1,
        min=0.01,
    )

    @classmethod
    def poll(cls, context):
        return (not global_vars.ui_lock_state
                and context.area.type == 'VIEW_3D'
                and context.mode == 'OBJECT'
                and len(bpy.data.movieclips) > 1)

    def execute(self, context):
        me.blender.OP_SolveCameras_Execute(self, self.views, self.solution_scale)
        return {'FINISHED'}

    def invoke(self, context, event):
        active = context.active_object
        cameras = [item[0] for item in clip_list(self, context)]
        if active is not None and active.type == 'CAMERA' and active.data.name in cameras:
            self.views = active.data.name
        me.blender.OP_SolveCameras_Invoke(self.views)
        return self.execute(context)


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
    bl_options = {'REGISTER', 'UNDO'}

    @classmethod
    def poll(cls, context):
        return (not global_vars.ui_lock_state and
                context.area.type == 'CLIP_EDITOR' and
                context.edit_movieclip is not None and
                context.edit_movieclip.tracking.tracks.active is not None and
                utils.is_valid_tag_name(context.edit_movieclip.tracking.tracks.active.name))

    def execute(self, context):
        scene = context.scene
        properties = scene.motion_engine_ui_properties
        current_clip = context.edit_movieclip
        active_track = current_clip.tracking.tracks.active

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

        properties.anchor_cam_selection = current_clip
        scene_cam.data['solution_id'] = active_track.name

        return {'FINISHED'}


class TrackCameraFromTagOperator(bpy.types.Operator):
    """Track view for current clip using selected tag as the origin"""
    bl_idname = "motionengine.track_camera_from_tag_operator"
    bl_label = "Track camera"
    bl_options = {'REGISTER', 'UNDO'}

    @classmethod
    def poll(cls, context):
        return (not global_vars.ui_lock_state and
                context.area.type == 'CLIP_EDITOR' and
                context.edit_movieclip is not None and
                context.edit_movieclip.tracking.tracks.active is not None and
                utils.is_valid_tag_name(context.edit_movieclip.tracking.tracks.active.name))

    def execute(self, context):
        scene = context.scene
        properties = scene.motion_engine_ui_properties
        current_clip = context.edit_movieclip
        active_track = current_clip.tracking.tracks.active

        scene_cam = utils.prepare_camera_for_clip(current_clip, context)
        clip_info = utils.ClipInfo(current_clip)
        cam_Kk = utils.get_clip_Kk(current_clip)

        flip_mtx = [
            [1, -1, -1, 1],
            [-1, 1, 1, -1],
            [-1, 1, 1, -1],
            [1, 1, 1, 1]
        ]

        was_keyed = False
        for marker in active_track.markers:
            tag = utils.marker_to_tag(marker, clip_info.clip_size, False)
            cam_T = me.tracking.solve_camera_with_tag(tag, cam_Kk)
            if cam_T.is_identity():
                continue
            if not was_keyed:
                for constraint in scene_cam.constraints:
                    if constraint.type == 'CAMERA_SOLVER':
                        scene_cam.constraints.remove(constraint)
                scene_cam.animation_data_clear()
                scene_cam.animation_data_create()
                scene_cam.rotation_mode = 'QUATERNION'
                scene_cam.delta_location = (0, 0, 0)
                scene_cam.delta_rotation_euler = (0, 0, 0)
                scene_cam.delta_scale = (1, 1, 1)
                scene_cam.matrix_world = mathutils.Matrix.Identity(4)
                context.view_layer.update()
            was_keyed = True
            cam_T = cam_T.to4x4()
            blend_mtx = mathutils.Matrix()
            for r in range(4):
                for c in range(4):
                    blend_mtx[r][c] = cam_T[r, c] * flip_mtx[r][c]
            scene_cam.matrix_world = blend_mtx
            frame_to_key = clip_info.clip_to_scene(marker.frame)
            self.report({'INFO'}, str(frame_to_key))
            scene_cam.keyframe_insert(data_path='location', frame=frame_to_key)
            scene_cam.keyframe_insert(data_path='rotation_quaternion', frame=frame_to_key)

        if not was_keyed:
            self.report({'ERROR'}, 'Failed to solve pose for selected tag')
            return {'FINISHED'}

        properties.anchor_cam_selection = current_clip
        scene_cam.data['solution_id'] = active_track.name

        return {'FINISHED'}


CLASSES = [
    SolveCamerasOperator,
    SolveCameraFromTagOperator,
    TrackCameraFromTagOperator
]
