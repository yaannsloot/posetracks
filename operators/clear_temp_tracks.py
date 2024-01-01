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
from ..property_groups import me_data

class ClearTempTracksOperator(bpy.types.Operator):
    """Delete non-persistent tracks"""
    bl_idname = "motionengine.clear_temp_tracks_operator"
    bl_label = "Clear temporary"

    def execute(self, context):
        bpy.ops.motionengine.prune_data_operator()
        if not global_vars.ui_lock_state:
            scene = context.scene
            properties = scene.motion_engine_ui_properties
            current_clip = context.edit_movieclip
            me_scene_data = scene.motion_engine_data

            clip_data = None
            for entry in me_scene_data.items:
                if entry.clip == current_clip:
                    clip_data = entry
                    break

            if clip_data is not None:
                pose_clips_list = properties.me_ui_prop_pose_clip_collection

                pose_tracks_list = None
                for entry in pose_clips_list:
                    if entry.clip == current_clip:
                        pose_tracks_list = entry.pose_tracks_list
                        break

                if pose_tracks_list is not None:
                    me_data.clear_tracks(current_clip, pose_tracks_list)

        return {'FINISHED'}


CLASSES = [
    ClearTempTracksOperator
]
