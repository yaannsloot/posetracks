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


class TagDetectionPanelSpace:
    bl_space_type = "CLIP_EDITOR"
    bl_region_type = "UI"
    bl_category = "PoseTracks"


class TagDetectionUIPanel(bpy.types.Panel, TagDetectionPanelSpace):
    bl_label = "Tag Detection"
    bl_idname = "POSETRACKS_TAG_DETECTION_PT_panel"

    display_priority = 0

    @classmethod
    def poll(cls, context):
        return context.edit_movieclip is not None

    def draw(self, context):
        layout = self.layout
        scene = context.scene
        ui_lock = global_vars.ui_lock_state

        layout.enabled = not ui_lock

        row = layout.row()

        row.scale_y = 1.6

        row.operator("posetracks.detect_tags_operator")

        row.enabled = context.edit_movieclip is not None


class DetectorSettingsUIPanel(bpy.types.Panel, TagDetectionPanelSpace):
    bl_parent_id = "POSETRACKS_TAG_DETECTION_PT_panel"
    bl_idname = "POSETRACKS_DETECTOR_TAG_SETTINGS_PT_panel"
    bl_label = 'Detector Settings'
    bl_options = {'DEFAULT_CLOSED'}

    display_priority = 1

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
        grid_ele.prop(properties, "det_tag_simple_sel_enum", text='')
        row = column.row()
        grid = row.grid_flow(even_columns=True, columns=2)
        grid_ele = grid.row()
        grid_ele.label(text="Device")
        grid_ele.alignment = 'RIGHT'
        grid_ele = grid.row()
        grid_ele.prop(properties, "exe_det_tag_enum", text='')
        row = column.row()
        grid = row.grid_flow(even_columns=True, columns=2)
        grid_ele = grid.row()
        grid_ele.label(text="Thresholding")
        grid_ele.alignment = 'RIGHT'
        grid_ele = grid.row()
        grid_ele.prop(properties, "det_tag_thresholding_enum", text='')
        if properties.det_tag_thresholding_enum == "MANUAL":
            row = column.row()
            grid = row.grid_flow(columns=2)
            grid_ele = grid.row()
            grid_ele.label(text="Confidence")
            grid_ele.alignment = 'RIGHT'
            grid_ele.prop(properties, "det_tag_conf", text="")
            row = column.row()
            grid = row.grid_flow(columns=2)
            grid_ele = grid.row()
            grid_ele.label(text="IoU")
            grid_ele.alignment = 'RIGHT'
            grid_ele.prop(properties, "det_tag_iou", text="")


class TagSettingsUIPanel(bpy.types.Panel, TagDetectionPanelSpace):
    bl_parent_id = "POSETRACKS_TAG_DETECTION_PT_panel"
    bl_idname = "POSETRACKS_TAG_SETTINGS_PT_panel"
    bl_label = 'Tag Settings'
    bl_options = {'DEFAULT_CLOSED'}

    display_priority = 2

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
        grid_ele.label(text="Detector Type")
        grid_ele.alignment = 'RIGHT'
        grid_ele = grid.row()
        grid_ele.prop(properties, "tag_detector_type_enum", text='')
        if properties.tag_detector_type_enum == 'CV':
            row = column.row()
            grid = row.grid_flow(even_columns=True, columns=2)
            grid_ele = grid.row()
            grid_ele.label(text="Dictionary")
            grid_ele.alignment = 'RIGHT'
            grid_ele = grid.row()
            grid_ele.prop(properties, "tag_detector_cv_dict_list_enum", text='')
            row = column.row()
            grid = row.grid_flow(even_columns=True, columns=2)
            grid_ele = grid.row()
            grid_ele.label(text="Resample")
            grid_ele.alignment = 'RIGHT'
            grid_ele = grid.row()
            grid_ele.prop(properties, "tag_detector_cv_resample_toggle", text='')
        else:
            row = column.row()
            grid = row.grid_flow(even_columns=True, columns=2)
            grid_ele = grid.row()
            grid_ele.label(text="Model Selection")
            grid_ele.alignment = 'RIGHT'
            grid_ele = grid.row()
            grid_ele.prop(properties, "tag_detector_ml_model_sel_enum", text='')
            row = column.row()
            grid = row.grid_flow(even_columns=True, columns=2)
            grid_ele = grid.row()
            grid_ele.label(text="Device")
            grid_ele.alignment = 'RIGHT'
            grid_ele = grid.row()
            grid_ele.prop(properties, "exe_tag_detector_ml_enum", text='')


CLASSES = [
    TagDetectionUIPanel,
    DetectorSettingsUIPanel,
    TagSettingsUIPanel
]
