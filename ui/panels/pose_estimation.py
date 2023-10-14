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
from ...property_groups import me_stats
from ...property_groups import me_data


class PoseEstimationUIPanel(bpy.types.Panel):
    bl_label = "Pose Estimation"
    bl_idname = "MOTIONENGINE_POSE_ESTIMATION_PT_panel"
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
        model_ready = global_vars.me_detectpose_model.is_ready()

        column = layout.column()

        box = column.box()
        column = box.column()
        column.label(text="Image cache size", icon="RENDERLAYERS")
        column.enabled = not ui_lock
        row = column.row()
        row = row.grid_flow(columns=2, align=True)
        r = row.row(align=True)
        r.prop(properties, "me_ui_prop_cache_method_enum", text='')
        r.enabled = not ui_lock
        r = row.row(align=True)
        r.enabled = properties.me_ui_prop_cache_method_enum == "MANUAL" and not ui_lock
        r.prop(properties, "me_ui_prop_cache_size", text='')

        row = layout.column(align=True)
        row.enabled = current_clip is not None and not ui_lock
        row.operator("motionengine.pose_estimation_task_operator")
        split = row.split(align=True)
        row = split.row(align=True)
        clip_data = me_data.check_data_for_clip(me_data, current_clip)
        row.enabled = not ui_lock and clip_data is not None and len(clip_data.frames) > 0
        row.operator("motionengine.clear_current_operator")
        row = split.row(align=True)
        data_exists = False
        for entry in me_data.items:
            if len(entry.frames) > 0:
                data_exists = True
                break
        row.enabled = not ui_lock and data_exists
        row.operator("motionengine.clear_all_operator")

        run_stats = me_stats.get_run_statistics(me_data, stats_data, current_clip)

        stats_ui_string_frames = "Valid frames: "
        if run_stats is not None and run_stats.valid_frames > 0:
            stats_ui_string_frames += str(run_stats.valid_frames) + " / " + str(context.edit_movieclip.frame_duration)
        else:
            stats_ui_string_frames += "n/a"

        stats_ui_string_poses = "Total poses: "
        if run_stats is not None and run_stats.valid_frames > 0:
            stats_ui_string_poses += str(run_stats.total_poses)
        else:
            stats_ui_string_poses += "n/a"

        stats_ui_string_conf = "Mean confidence: "
        if run_stats is not None and run_stats.valid_frames > 0:
            stats_ui_string_conf += f"{run_stats.mean_confidence:.0%}"
        else:
            stats_ui_string_conf += "n/a"

        stats_ui_string_conf50 = "Mean conf. > 50: "
        if run_stats is not None and run_stats.valid_frames > 0 and run_stats.mean50_conf >= 0.5:
            stats_ui_string_conf50 += f"{run_stats.mean50_conf:.0%}"
        else:
            stats_ui_string_conf50 += "n/a"

        column = layout.column()
        row = column.row()
        row.enabled = not ui_lock
        row.label(text=stats_ui_string_frames)
        row = column.row()
        row.enabled = not ui_lock
        row.label(text=stats_ui_string_poses)
        row = column.row()
        row.enabled = not ui_lock
        row.label(text=stats_ui_string_conf)
        row = column.row()
        row.enabled = not ui_lock
        row.label(text=stats_ui_string_conf50)


CLASSES = [
    PoseEstimationUIPanel
]
