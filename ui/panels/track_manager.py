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
from ... import global_vars

class TrackManagerUIPanel(bpy.types.Panel):
    bl_label = "Track Manager"
    bl_idname = "MOTIONENGINE_TRACK_MANAGER_PT_panel"
    bl_space_type = "CLIP_EDITOR"
    bl_region_type = "UI"
    bl_category = "MotionEngine"

    def draw(self, context):
        layout = self.layout
        scene = context.scene
        properties = scene.motion_engine_ui_properties
        current_clip = context.edit_movieclip
        me_data = scene.motion_engine_data
        stats_data = scene.motion_engine_ui_properties.me_ui_prop_stats_collection
        warmup = global_vars.warmup_state
        ui_lock = global_vars.ui_lock_state

        pose_properties = None

        if current_clip is not None:
            pose_properties = get_pose_tracks_clip(current_clip, properties)

        column = layout.column()

        if pose_properties is not None:
            row = column.row()

            row.template_list("ME_UL_PoseListUIPanel", "", pose_properties, "pose_tracks_list", properties,
                              "me_ui_prop_active_pose_index", rows=2)

        column.operator("motionengine.create_tracks_operator")


CLASSES = [
    TrackManagerUIPanel
]
