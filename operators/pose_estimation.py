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
from .. import MotionEngine as me
from .. import global_vars
from .. import callbacks
from ..property_groups import me_stats
from ..property_groups import me_data

info_str = {'INFO'}


class PoseEstimationTaskOperator(bpy.types.Operator):
    """Start pose estimation analysis"""
    bl_idname = "motionengine.pose_estimation_task_operator"
    bl_label = "Detect poses"

    def execute(self, context):
        bpy.ops.motionengine.prune_data_operator()
        if not global_vars.ui_lock_state:
            scene = context.scene
            properties = scene.motion_engine_ui_properties

            selected_exec_det = me.dnn.Executor.CPU
            if properties.me_ui_prop_exe_det_enum == "CUDA":
                selected_exec_det = me.dnn.Executor.CUDA
            elif properties.me_ui_prop_exe_det_enum == "TENSORRT":
                selected_exec_det = me.dnn.Executor.TENSORRT

            selected_exec_pose = me.dnn.Executor.CPU
            if properties.me_ui_prop_exe_pose_enum == "CUDA":
                selected_exec_pose = me.dnn.Executor.CUDA
            elif properties.me_ui_prop_exe_pose_enum == "TENSORRT":
                selected_exec_pose = me.dnn.Executor.TENSORRT

            kp_str = properties.me_ui_prop_pose_keypoints_enum
            depth_str = ""
            precision_str = ""

            if properties.me_ui_prop_pose_keypoints_enum == "17":
                depth_str = properties.me_ui_prop_pose_depth17_enum
                if depth_str == "m" and properties.me_ui_prop_pose_precision17m_enum == "DOUBLE":
                    precision_str = "_double"
                elif depth_str == "l" and properties.me_ui_prop_pose_precision17l_enum == "DOUBLE":
                    precision_str = "_double"
            elif properties.me_ui_prop_pose_keypoints_enum == "26":
                depth_str = properties.me_ui_prop_pose_depth26_enum
                if depth_str == "m" and properties.me_ui_prop_pose_precision26m_enum == "DOUBLE":
                    precision_str = "_double"
                elif depth_str == "l" and properties.me_ui_prop_pose_precision26l_enum == "DOUBLE":
                    precision_str = "_double"
            elif properties.me_ui_prop_pose_keypoints_enum == "133":
                depth_str = properties.me_ui_prop_pose_depth133_enum
                if depth_str == "l" and properties.me_ui_prop_pose_precision133l_enum == "DOUBLE":
                    precision_str = "_double"

            det_path = me.model_path("rtmdet/fullbody_" + properties.me_ui_prop_det_size_enum + ".onnx")
            pose_path = me.model_path("rtmpose/fullbody" + kp_str + "-" + depth_str + precision_str + ".onnx")

            global_vars.context_tracker = context

            global_vars.clip_tracker = context.edit_movieclip

            global_vars.properties_tracker = scene.motion_engine_data

            global_vars.stats_tracker = scene.motion_engine_ui_properties.me_ui_prop_stats_collection

            cache_size = properties.me_ui_prop_cache_size

            det_batch_size = properties.me_ui_prop_det_batch_size

            pose_batch_size = properties.me_ui_prop_pose_batch_size

            det_conf_thresh = properties.me_ui_prop_det_conf

            det_iou_thresh = properties.me_ui_prop_det_iou

            callbacks.ui_lock_callback(True)
            callbacks.display_warmup_state_callback(False)
            callbacks.ui_draw_callback()

            global_vars.info_message.msg = "Detecting poses: 0%"

            bpy.ops.motionengine.report_status_operator()  # Start the modal info reporter

            me.threading.infer_async(
                global_vars.me_detectpose_model,
                det_path,
                selected_exec_det,
                pose_path,
                selected_exec_pose,
                cache_size,
                det_batch_size,
                pose_batch_size,
                det_conf_thresh,
                det_iou_thresh,
                global_vars.clip_tracker,
                global_vars.info_message,
                bpy.path.abspath,
                callbacks.ui_draw_callback,
                callbacks.ui_lock_callback,
                callbacks.display_warmup_state_callback,
                callbacks.write_data_callback
            )

        return {'FINISHED'}


class PoseEstimationClearCurrentOperator(bpy.types.Operator):
    """Clear analysis data from current clip"""
    bl_idname = "motionengine.clear_current_operator"
    bl_label = "Clear current"

    def execute(self, context):
        bpy.ops.motionengine.prune_data_operator()
        if not global_vars.ui_lock_state:
            scene = context.scene
            current_clip = context.edit_movieclip
            me_scene_data = scene.motion_engine_data
            stats_data = scene.motion_engine_ui_properties.me_ui_prop_stats_collection

            stat_prop = None
            for entry in stats_data:
                if entry.clip == current_clip:
                    stat_prop = entry

            clip_data = None
            for entry in me_scene_data.items:
                if entry.clip == current_clip:
                    clip_data = entry

            me_data.clear_blend_clip_data(current_clip)
            if stat_prop is not None and clip_data is not None:
                global_vars.clip_tracker = context.edit_movieclip
                bpy.ops.motionengine.calculate_statistics_operator()

        return {'FINISHED'}


class PoseEstimationClearAllOperator(bpy.types.Operator):
    """Clear analysis data from all clips"""
    bl_idname = "motionengine.clear_all_operator"
    bl_label = "Clear all"

    def execute(self, context):
        bpy.ops.motionengine.prune_data_operator()
        if not global_vars.ui_lock_state:
            scene = context.scene
            me_scene_data = scene.motion_engine_data
            stats_data = scene.motion_engine_ui_properties.me_ui_prop_stats_collection
            for entry in me_scene_data.items:
                current_clip = entry.clip
                stat_prop = me_stats.check_stats_for_clip(stats_data, current_clip)
                clip_data = me_data.check_data_for_clip(me_scene_data, current_clip)
                me_data.clear_blend_clip_data(current_clip)
                if stat_prop is not None and clip_data is not None:
                    global_vars.clip_tracker = context.edit_movieclip
                    bpy.ops.motionengine.calculate_statistics_operator()

        return {'FINISHED'}


CLASSES = [
    PoseEstimationTaskOperator,
    PoseEstimationClearCurrentOperator,
    PoseEstimationClearAllOperator
]
