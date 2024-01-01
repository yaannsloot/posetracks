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
from .. import MotionEngine
from ..MotionEngine import MEPython
from .. import global_vars
from .. import callbacks


class ModelLoadOperator(bpy.types.Operator):
    """Load or reload models"""
    bl_idname = "motionengine.model_load_operator"
    bl_label = "Load Models"

    def execute(self, context):
        bpy.ops.motionengine.prune_data_operator()
        if not global_vars.ui_lock_state:
            scene = context.scene
            properties = scene.motion_engine_ui_properties

            selected_exec_det = MEPython.dnn.Executor.CPU
            if properties.me_ui_prop_exe_det_enum == "CUDA":
                selected_exec_det = MEPython.dnn.Executor.CUDA
            elif properties.me_ui_prop_exe_det_enum == "TENSORRT":
                selected_exec_det = MEPython.dnn.Executor.TENSORRT

            selected_exec_pose = MEPython.dnn.Executor.CPU
            if properties.me_ui_prop_exe_pose_enum == "CUDA":
                selected_exec_pose = MEPython.dnn.Executor.CUDA
            elif properties.me_ui_prop_exe_pose_enum == "TENSORRT":
                selected_exec_pose = MEPython.dnn.Executor.TENSORRT

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

            det_path = MotionEngine.model_path("rtmdet/fullbody_" + properties.me_ui_prop_det_size_enum + ".onnx")
            pose_path = MotionEngine.model_path("rtmpose/fullbody" + kp_str + "-" + depth_str + precision_str + ".onnx")

            global_vars.me_detectpose_model.unload_all()

            global_vars.context_tracker = context

            callbacks.ui_lock_callback(True)
            callbacks.display_warmup_state_callback(True)
            callbacks.ui_draw_callback()

            MEPython.mt.rtm_load_async(
                global_vars.me_detectpose_model,
                det_path,
                selected_exec_det,
                pose_path,
                selected_exec_pose,
                callbacks.ui_draw_callback,
                callbacks.ui_lock_callback,
                callbacks.display_warmup_state_callback
            )

        return {'FINISHED'}


class ModelUnloadOperator(bpy.types.Operator):
    """Unload models"""
    bl_idname = "motionengine.model_unload_operator"
    bl_label = "Unload Models"

    def execute(self, context):
        bpy.ops.motionengine.prune_data_operator()
        if not global_vars.ui_lock_state:
            global_vars.me_detectpose_model.unload_all()
            print("[MotionEngine] Unloaded models.")
            return {'FINISHED'}


CLASSES = [
    ModelLoadOperator,
    ModelUnloadOperator
]
