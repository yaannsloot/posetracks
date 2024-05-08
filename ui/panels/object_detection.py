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


class ObjectDetectionPanelSpace:
    bl_space_type = "CLIP_EDITOR"
    bl_region_type = "UI"
    bl_category = "MotionEngine"


class ObjectDetectionUIPanel(bpy.types.Panel, ObjectDetectionPanelSpace):
    bl_label = "Object Detection"
    bl_idname = "MOTIONENGINE_OBJECT_DETECTION_PT_panel"

    display_priority = 1

    @classmethod
    def poll(cls, context):
        return context.edit_movieclip is not None

    def draw(self, context):
        layout = self.layout
        scene = context.scene
        properties = scene.motion_engine_ui_properties
        ui_lock = global_vars.ui_lock_state

        layout.enabled = not ui_lock

        row = layout.row()

        row.scale_y = 1.6

        row.operator("motionengine.detect_objects_operator")

        row.enabled = context.edit_movieclip is not None

        grid = layout.grid_flow(columns=2)
        row = grid.row()
        row.label(text='Target')
        row.alignment = 'RIGHT'
        row = grid.row()
        row.prop(properties, 'me_ui_prop_det_class_enum', text='')


class DetectorSettingsUIPanel(bpy.types.Panel, ObjectDetectionPanelSpace):
    bl_parent_id = 'MOTIONENGINE_OBJECT_DETECTION_PT_panel'
    bl_idname = "MOTIONENGINE_DETECTOR_SETTINGS_PT_panel"
    bl_label = 'Detector Settings'
    bl_options = {'DEFAULT_CLOSED'}

    display_priority = 2

    def draw(self, context):
        layout = self.layout
        scene = context.scene
        properties = scene.motion_engine_ui_properties
        ui_lock = global_vars.ui_lock_state

        layout.enabled = not ui_lock

        column = layout.column()
        row = column.row()
        grid = row.grid_flow(even_columns=True, columns=2)
        grid_ele = grid.row()
        grid_ele.label(text="Model Selection")
        grid_ele.alignment = 'RIGHT'
        grid_ele = grid.row()
        grid_ele.prop(properties, "me_ui_prop_det_simple_sel_enum", text='')
        row = column.row()
        grid = row.grid_flow(even_columns=True, columns=2)
        grid_ele = grid.row()
        grid_ele.label(text="Device")
        grid_ele.alignment = 'RIGHT'
        grid_ele = grid.row()
        grid_ele.prop(properties, "me_ui_prop_exe_det_enum", text='')
        row = column.row()
        grid = row.grid_flow(even_columns=True, columns=2)
        grid_ele = grid.row()
        grid_ele.label(text="Thresholding")
        grid_ele.alignment = 'RIGHT'
        grid_ele = grid.row()
        grid_ele.prop(properties, "me_ui_prop_det_thresholding_enum", text='')
        if properties.me_ui_prop_det_thresholding_enum == "MANUAL":
            row = column.row()
            grid = row.grid_flow(columns=2)
            grid_ele = grid.row()
            grid_ele.label(text="Confidence")
            grid_ele.alignment = 'RIGHT'
            grid_ele.prop(properties, "me_ui_prop_det_conf", text="")
            row = column.row()
            grid = row.grid_flow(columns=2)
            grid_ele = grid.row()
            grid_ele.label(text="IoU")
            grid_ele.alignment = 'RIGHT'
            grid_ele.prop(properties, "me_ui_prop_det_iou", text="")


class TrackerSettingsUIPanel(bpy.types.Panel, ObjectDetectionPanelSpace):
    bl_parent_id = 'MOTIONENGINE_OBJECT_DETECTION_PT_panel'
    bl_idname = "MOTIONENGINE_TRACKER_SETTINGS_PT_panel"
    bl_label = 'Tracker Settings'
    bl_options = {'DEFAULT_CLOSED'}

    display_priority = 3

    def draw(self, context):
        layout = self.layout
        scene = context.scene
        properties = scene.motion_engine_ui_properties
        ui_lock = global_vars.ui_lock_state

        layout.enabled = not ui_lock

        column = layout.column()
        row = column.row()
        grid = row.grid_flow(even_columns=True, columns=2)
        grid_ele = grid.row()
        grid_ele.label(text="Device")
        grid_ele.alignment = 'RIGHT'
        grid_ele = grid.row()
        grid_ele.prop(properties, "me_ui_prop_exe_track_enum", text='')
        row = column.row()
        grid = row.grid_flow(even_columns=True, columns=2)
        grid_ele = grid.row()
        grid_ele.label(text="Thresholding")
        grid_ele.alignment = 'RIGHT'
        grid_ele = grid.row()
        grid_ele.prop(properties, "me_ui_prop_track_thresholding_enum", text='')
        if properties.me_ui_prop_track_thresholding_enum == "MANUAL":
            row = column.row()
            grid = row.grid_flow(columns=2)
            grid_ele = grid.row()
            grid_ele.label(text="Distance Type")
            grid_ele.alignment = 'RIGHT'
            grid_ele.prop(properties, "me_ui_prop_track_dist_type", text="")
            row = column.row()
            grid = row.grid_flow(columns=2)
            grid_ele = grid.row()
            grid_ele.label(text="Score")
            grid_ele.alignment = 'RIGHT'
            grid_ele.prop(properties, "me_ui_prop_track_score", text="")
            row = column.row()
            grid = row.grid_flow(columns=2)
            grid_ele = grid.row()
            grid_ele.label(text="ReID Score")
            grid_ele.alignment = 'RIGHT'
            grid_ele.prop(properties, "me_ui_prop_track_reid_score", text="")


CLASSES = [
    ObjectDetectionUIPanel,
    DetectorSettingsUIPanel,
    TrackerSettingsUIPanel
]
