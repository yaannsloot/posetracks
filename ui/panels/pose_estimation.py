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
from ... import utils


class PoseEstimationPanelSpace:
    bl_space_type = "CLIP_EDITOR"
    bl_region_type = "UI"
    bl_category = "PoseTracks"


class PoseEstimationUIPanel(bpy.types.Panel, PoseEstimationPanelSpace):
    bl_label = "Pose Estimation"
    bl_idname = "POSETRACKS_POSE_ESTIMATION_PT_panel"

    display_priority = 2

    @classmethod
    def poll(cls, context):
        return context.edit_movieclip is not None

    def draw(self, context):
        layout = self.layout
        scene = context.scene
        properties = scene.pt_ui_properties
        ui_lock = global_vars.ui_lock_state

        layout.enabled = not ui_lock

        row = layout.row()

        row.scale_y = 1.6

        row.operator("posetracks.detect_poses_operator")

        row.enabled = context.edit_movieclip is not None and utils.get_active_track_count(context.edit_movieclip) > 0

        grid = layout.grid_flow(columns=2)
        row = grid.row()
        row.label(text='Target Type')
        row.alignment = 'RIGHT'
        row = grid.row()
        row.prop(properties, 'pose_target_enum', text='')

        grid = layout.grid_flow(columns=2)
        row = grid.row()
        row.label(text='Keypoints')
        row.alignment = 'RIGHT'
        row = grid.row()
        row.prop(properties, 'pose_keypoints_enum', text='')


class ModelSettingsPanel(bpy.types.Panel, PoseEstimationPanelSpace):
    bl_parent_id = "POSETRACKS_POSE_ESTIMATION_PT_panel"
    bl_idname = "POSETRACKS_POSE_MODEL_SETTINGS_PT_panel"
    bl_label = 'Model Settings'
    bl_options = {'DEFAULT_CLOSED'}

    display_priority = 3

    def draw(self, context):
        layout = self.layout
        scene = context.scene
        properties = scene.pt_ui_properties
        ui_lock = global_vars.ui_lock_state

        layout.enabled = not ui_lock

        column = layout.column()
        row = column.row()
        grid = row.grid_flow(even_columns=True, columns=2)
        grid_ele = grid.row()
        grid_ele.label(text="Model Selection")
        grid_ele.alignment = 'RIGHT'
        grid_ele = grid.row()
        grid_ele.prop(properties, "pose_model_sel_enum", text='')
        row = column.row()
        grid = row.grid_flow(even_columns=True, columns=2)
        grid_ele = grid.row()
        grid_ele.label(text="Device")
        grid_ele.alignment = 'RIGHT'
        grid_ele = grid.row()
        grid_ele.prop(properties, "exe_pose_enum", text='')
        row = column.row()
        grid = row.grid_flow(even_columns=True, columns=2)
        grid_ele = grid.row()
        grid_ele.label(text="Thresholding")
        grid_ele.alignment = 'RIGHT'
        grid_ele = grid.row()
        grid_ele.prop(properties, "pose_thresholding_enum", text='')
        if properties.pose_thresholding_enum == "MANUAL":
            row = column.row()
            grid = row.grid_flow(columns=2)
            grid_ele = grid.row()
            grid_ele.label(text="Confidence")
            grid_ele.alignment = 'RIGHT'
            grid_ele.prop(properties, "joint_conf", text="")


CLASSES = [
    PoseEstimationUIPanel,
    ModelSettingsPanel
]
