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
from .. import global_vars
from .. import utils


class TrackingSelectPoseOperator(bpy.types.Operator):
    """Select all tracks associated with this pose"""
    bl_idname = "motionengine.select_pose_operator"
    bl_label = "Select All"
    bl_options = {'REGISTER', 'UNDO'}

    @classmethod
    def poll(cls, context):
        return not global_vars.ui_lock_state

    def execute(self, context):
        active_clip = context.edit_movieclip
        active_track = active_clip.tracking.tracks.active

        if active_track is None or not utils.is_valid_joint_name(active_track.name):
            return {'FINISHED'}

        split_name = active_track.name.split('.')

        source_id = split_name[-2]

        pose_name = ''

        for i in range(len(split_name) - 2):
            if pose_name != '':
                pose_name += '.' + split_name[i]
            else:
                pose_name = split_name[i]

        all_poses, _ = utils.get_joint_tracks(active_clip)

        if pose_name not in all_poses:
            return {'FINISHED'}

        pose = all_poses[pose_name]

        for source in pose:
            if source != source_id:
                continue
            for joint_id in pose[source]:
                pose[source][joint_id].select = True

        return {'FINISHED'}


def set_lock(clip, track, value):
    if utils.is_valid_joint_name(track.name):
        split_name = track.name.split('.')
        source_id = split_name[-2]
        pose_name = ''
        for i in range(len(split_name) - 2):
            if pose_name != '':
                pose_name += '.' + split_name[i]
            else:
                pose_name = split_name[i]
        all_poses, _ = utils.get_joint_tracks(clip)
        if pose_name not in all_poses:
            return
        pose = all_poses[pose_name]
        for source in pose:
            if source != source_id:
                continue
            for joint_id in pose[source]:
                pose[source][joint_id].lock = value
        return
    track.lock = value


class TrackingLockAllOperator(bpy.types.Operator):
    """Locked tracks are used for camera calibration and triangulation"""
    bl_idname = "motionengine.lock_tracks_operator"
    bl_label = "Lock this track and all associated tracks."
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        if global_vars.ui_lock_state:
            return {'FINISHED'}

        active_clip = context.edit_movieclip
        active_track = active_clip.tracking.tracks.active

        if active_track is None:
            return {'FINISHED'}

        set_lock(active_clip, active_track, True)

        return {'FINISHED'}


class TrackingUnlockAllOperator(bpy.types.Operator):
    """Locked tracks are used for camera calibration and triangulation"""
    bl_idname = "motionengine.unlock_tracks_operator"
    bl_label = "Unlock this track and all associated tracks."
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        if global_vars.ui_lock_state:
            return {'FINISHED'}

        active_clip = context.edit_movieclip
        active_track = active_clip.tracking.tracks.active

        if active_track is None:
            return {'FINISHED'}

        set_lock(active_clip, active_track, False)

        return {'FINISHED'}


CLASSES = [
    TrackingSelectPoseOperator,
    TrackingLockAllOperator,
    TrackingUnlockAllOperator
]
