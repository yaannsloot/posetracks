'''
Copyright (C) 2023 Ian Sloat

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
from ..property_groups import me_data


class PruneDataOperator(bpy.types.Operator):
    bl_idname = "motionengine.prune_data_operator"
    bl_label = "Prune expired data"

    def execute(self, context):
        scene = context.scene
        properties = scene.motion_engine_ui_properties
        me_scene_data = scene.motion_engine_data
        all_clips = bpy.data.movieclips
        pose_clips = properties.me_ui_prop_pose_clip_collection
        index = 0
        pruned_data = 0
        pruned_poses = 0
        while index < len(me_scene_data.items):
            increment = True
            entry = me_scene_data.items[index]
            if not utils.check_list(entry.clip, all_clips):
                me_scene_data.items.remove(index)
                pruned_data = pruned_data + 1
                increment = False
            if increment:
                index = index + 1
        index = 0
        while index < len(pose_clips):
            increment = True
            entry = pose_clips[index]
            if not utils.check_list(entry.clip, all_clips):
                pose_tracks_list = entry.pose_tracks_list
                pruned_poses = pruned_poses + len(pose_tracks_list)
                pose_tracks_list.clear()
                pose_clips.remove(index)
                increment = False
            if increment:
                index = index + 1

        if pruned_data > 0 or pruned_poses > 0:
            print("[MotionEngine] Pruned " + str(pruned_data) + " expired data entries and " + str(
                pruned_poses) + " expired pose entries.")

        return {'FINISHED'}


CLASSES = [
    PruneDataOperator
]
