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


class ModelManagerUIPanel(bpy.types.Panel):
    bl_label = "Model Manager"
    bl_idname = "MOTIONENGINE_MODEL_MANAGER_PT_panel"
    bl_space_type = "CLIP_EDITOR"
    bl_region_type = "UI"
    bl_category = "MotionEngine"

    def draw(self, context):
        layout = self.layout
        scene = context.scene
        properties = scene.motion_engine_ui_properties
        warmup = global_vars.warmup_state
        ui_lock = global_vars.ui_lock_state
        model_ready = global_vars.me_detectpose_model.is_ready()

        column = layout.column()

        box = column.box()
        column = box.column()
        column.label(text="Detection Model", icon="CON_OBJECTSOLVER")
        column.enabled = not ui_lock
        row = column.row()
        row.label(text="Executor:")
        row.prop(properties, "me_ui_prop_exe_det_enum", text='')
        row.enabled = not ui_lock
        row = column.row()
        row.label(text="Batch size:")
        row.prop(properties, "me_ui_prop_det_batch_size", text='')
        row.enabled = not ui_lock
        row = column.row()
        row.label(text="Input size:")
        row.prop(properties, "me_ui_prop_det_size_enum", text='')
        row.enabled = not ui_lock
        row = column.row()
        row.label(text="Thresholding:")
        row.prop(properties, "me_ui_prop_det_thresholding_enum", text='')
        row.enabled = not ui_lock
        if properties.me_ui_prop_det_thresholding_enum == "MANUAL":
            row = column.row()
            row.alignment = 'RIGHT'
            row.label(text="Confidence:")
            row.prop(properties, "me_ui_prop_det_conf", text="")
            row.enabled = not ui_lock
            row = column.row()
            row.alignment = 'RIGHT'
            row.label(text="IoU:")
            row.prop(properties, "me_ui_prop_det_iou", text="")
            row.enabled = not ui_lock

        column = layout.column()

        box = column.box()
        column = box.column()
        column.label(text="Pose Model", icon="OUTLINER_DATA_ARMATURE")
        column.enabled = not ui_lock
        row = column.row()
        row.label(text="Executor:")
        row.prop(properties, "me_ui_prop_exe_pose_enum", text='')
        row.enabled = not ui_lock
        row = column.row()
        row.label(text="Batch size:")
        row.prop(properties, "me_ui_prop_pose_batch_size", text='')
        row.enabled = not ui_lock
        row = column.row()
        row.label(text="Keypoints:")
        row.prop(properties, "me_ui_prop_pose_keypoints_enum", text='')
        row.enabled = not ui_lock
        if properties.me_ui_prop_pose_keypoints_enum == "17":
            row = column.row()
            row.label(text="Depth:")
            row.prop(properties, "me_ui_prop_pose_depth17_enum", text='')
            row.enabled = not ui_lock
            if properties.me_ui_prop_pose_depth17_enum == "m":
                row = column.row()
                row.label(text="Precision:")
                row.prop(properties, "me_ui_prop_pose_precision17m_enum", text='')
                row.enabled = not ui_lock
            elif properties.me_ui_prop_pose_depth17_enum == "l":
                row = column.row()
                row.label(text="Precision:")
                row.prop(properties, "me_ui_prop_pose_precision17l_enum", text='')
                row.enabled = not ui_lock
        elif properties.me_ui_prop_pose_keypoints_enum == "26":
            row = column.row()
            row.label(text="Depth:")
            row.prop(properties, "me_ui_prop_pose_depth26_enum", text='')
            row.enabled = not ui_lock
            if properties.me_ui_prop_pose_depth26_enum == "m":
                row = column.row()
                row.label(text="Precision:")
                row.prop(properties, "me_ui_prop_pose_precision26m_enum", text='')
                row.enabled = not ui_lock
            elif properties.me_ui_prop_pose_depth26_enum == "l":
                row = column.row()
                row.label(text="Precision:")
                row.prop(properties, "me_ui_prop_pose_precision26l_enum", text='')
                row.enabled = not ui_lock
        elif properties.me_ui_prop_pose_keypoints_enum == "133":
            row = column.row()
            row.label(text="Depth:")
            row.prop(properties, "me_ui_prop_pose_depth133_enum", text='')
            row.enabled = not ui_lock
            if properties.me_ui_prop_pose_depth133_enum == "l":
                row = column.row()
                row.label(text="Precision:")
                row.prop(properties, "me_ui_prop_pose_precision133l_enum", text='')
                row.enabled = not ui_lock

        row = layout.row()
        b_text = "Load Models"

        if model_ready and not warmup:
            b_text = "Reload Models"
        elif not model_ready and warmup:
            b_text = "Loading Models..."
        elif model_ready and warmup:
            b_text = "Warming Up..."
        row.enabled = not ui_lock
        row.operator("motionengine.model_load_operator", text=b_text)

        row = layout.row()
        row.enabled = model_ready and not ui_lock
        row.operator("motionengine.model_unload_operator")


CLASSES = [
    ModelManagerUIPanel
]
