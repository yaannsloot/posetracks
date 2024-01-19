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
from ... import global_vars


def get_pose_tracks_clip(clip, properties):
    pose_tracks_clip = None
    clip_collection = properties.me_ui_prop_pose_clip_collection
    for entry in clip_collection:
        if entry.clip == clip:
            pose_tracks_clip = entry
    return pose_tracks_clip


class TrackManagerUIPanel(bpy.types.Panel):
    bl_label = "Pose Manager"
    bl_idname = "MOTIONENGINE_TRACK_MANAGER_PT_panel"
    bl_space_type = "CLIP_EDITOR"
    bl_region_type = "UI"
    bl_category = "MotionEngine"

    def draw(self, context):
        layout = self.layout
        scene = context.scene
        properties = scene.motion_engine_ui_properties
        current_clip = context.edit_movieclip
        me_scene_data = scene.motion_engine_data
        stats_data = scene.motion_engine_ui_properties.me_ui_prop_stats_collection
        warmup = global_vars.warmup_state
        ui_lock = global_vars.ui_lock_state

        layout.enabled = not ui_lock

        pose_properties = None

        if current_clip is not None:
            pose_properties = get_pose_tracks_clip(current_clip, properties)
            if pose_properties is None:
                pose_properties = properties.me_ui_prop_pose_empty_clip

        column = layout.column()

        row = column.row()

        row.template_list("ME_UL_PoseListUIPanel", "", pose_properties, "pose_tracks_list", properties,
                          "me_ui_prop_active_pose_index", rows=2, maxrows=4)

        column = layout.column(align=True)

        column.operator("motionengine.create_tracks_operator")

        row = column.row(align=True)

        row.operator("motionengine.clear_temp_tracks_operator")

        row.operator("motionengine.clear_tracks_operator")

        row = layout.row()

        row.operator("motionengine.solve_cameras_operator")

        row = layout.row()

        row.operator("motionengine.filter_tracks_operator")

        row = layout.row()

        row.operator("motionengine.filter_fcurves_operator")


CLASSES = [
    TrackManagerUIPanel
]
