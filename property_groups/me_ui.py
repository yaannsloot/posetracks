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
from .. import utils

object_detection_classes = set()

for model in me.get_models('object_detection'):
    if 'classes' not in model[1]:
        continue
    classes = model[1]['classes']
    object_detection_classes.update(classes)

object_detection_classes = list(object_detection_classes)

object_detection_classes.sort()

obj_det_class_enum_items = []
for i in range(len(object_detection_classes)):
    c = object_detection_classes[i]
    obj_det_class_enum_items.append((c, c.capitalize(), f'Set detection target to {c.lower()}', i + 1))

object_detection_classes = [c.lower() for c in object_detection_classes]

try:
    obj_det_class_default = object_detection_classes.index('person') + 1
except ValueError:
    obj_det_class_default = 1

tag_dictionary_members = [e for e in me.TagDictionary.__members__]

tag_detector_cv_dict_list_items = []
for m in tag_dictionary_members:
    tag_detector_cv_dict_list_items.append((m, m, f'Set dictionary to {m}'))

pose_estimation_targets = set()

for model in me.get_models('pose_estimation'):
    if 'target_class' not in model[1]:
        continue
    pose_estimation_targets.add(model[1]['target_class'])

pose_estimation_targets = list(pose_estimation_targets)

pose_estimation_targets.sort()

pose_estimation_targets_enum_items = []
for i in range(len(pose_estimation_targets)):
    t = pose_estimation_targets[i]
    pose_estimation_targets_enum_items.append((t, t.capitalize(), f'Set pose target type to {t}', i + 1))

pose_estimation_targets = [t.lower() for t in pose_estimation_targets]

try:
    pose_estimation_targets_default = pose_estimation_targets.index('person') + 1
except ValueError:
    pose_estimation_targets_default = 1

pose_estimation_target = 'person'
pose_estimation_kp_items = []


def update_pose_keypoints_items():
    global pose_estimation_kp_items
    models = me.get_models('pose_estimation', 'keypoints',
                           {'target_class': pose_estimation_target})
    choices = {m_def['keypoints'] for (m_name, m_def) in models}
    choices = list(choices)
    choices.sort()
    pose_estimation_kp_items = [(str(choices[i]), str(choices[i]), f'Set keypoints to {choices[i]}', i + 1) for i in
                                range(len(choices))]


update_pose_keypoints_items()


def get_keypoint_items(self, context):
    return pose_estimation_kp_items


def update_pose_keypoints(self, context):
    global pose_estimation_target
    scene = context.scene
    properties = scene.motion_engine_ui_properties
    pose_estimation_target = properties.me_ui_prop_pose_target_enum
    update_pose_keypoints_items()
    properties.me_ui_prop_pose_keypoints_enum = pose_estimation_kp_items[0][0]


def force_redraw(self, context):
    if context.area is not None:
        for region in context.area.regions:
            if region.type == "UI":
                region.tag_redraw()
    return None


def det_thresh_mode_update(self, context):
    scene = context.scene
    properties = scene.motion_engine_ui_properties
    if properties.me_ui_prop_det_thresholding_enum == "AUTO":
        properties.me_ui_prop_det_conf = 0.5
        properties.me_ui_prop_det_iou = 0.5
    return None


def pose_thresh_mode_update(self, context):
    scene = context.scene
    properties = scene.motion_engine_ui_properties
    if properties.me_ui_prop_pose_thresholding_enum == "AUTO":
        properties.me_ui_prop_joint_conf = 0.5
    return None


def det_tag_thresh_mode_update(self, context):
    scene = context.scene
    properties = scene.motion_engine_ui_properties
    if properties.me_ui_prop_det_tag_thresholding_enum == "AUTO":
        properties.me_ui_prop_det_tag_conf = 0.5
        properties.me_ui_prop_det_tag_iou = 0.5
    return None


def track_thresh_mode_update(self, context):
    scene = context.scene
    properties = scene.motion_engine_ui_properties
    if properties.me_ui_prop_track_thresholding_enum == "AUTO":
        properties.me_ui_prop_track_score = 1.4
        properties.me_ui_prop_track_reid_score = 1.0
        properties.me_ui_prop_track_dist_type = "NORM_EUCLIDEAN"
    return None


def get_track_name(self):
    current_clip = bpy.context.edit_movieclip
    active_track = current_clip.tracking.tracks.active
    if active_track is None:
        return ''
    if utils.is_valid_joint_name(active_track.name):
        split_name = active_track.name.split('.')
        pose_name = ''
        for i in range(len(split_name) - 2):
            if pose_name != '':
                pose_name += '.' + split_name[i]
            else:
                pose_name = split_name[i]
        return pose_name
    return active_track.name


def set_track_name(self, value):
    current_clip = bpy.context.edit_movieclip
    active_track = current_clip.tracking.tracks.active
    if active_track is None or utils.is_valid_tag_name(active_track.name):
        return
    if utils.is_valid_joint_name(active_track.name):
        split_name = active_track.name.split('.')
        source_id = split_name[-2]
        pose_name = ''
        for i in range(len(split_name) - 2):
            if pose_name != '':
                pose_name += '.' + split_name[i]
            else:
                pose_name = split_name[i]
        all_poses, _ = utils.get_joint_tracks(current_clip)
        if value in all_poses and source_id in all_poses[value]:
            return
        pose = all_poses[pose_name]
        for source in pose:
            if source != source_id:
                continue
            for joint_id in pose[source]:
                track = pose[source][joint_id]
                new_name = f'{value}.{source}.{joint_id}'
                track.name = new_name
        return

    active_track.name = value


class MotionEngineUIProperties(bpy.types.PropertyGroup):
    # Executor list text
    executor_list_name = "Target execution device"
    executor_list_desc = "Model execution device"
    cpu_tooltip = "Run model on the CPU"
    cuda_tooltip = "Run model on GPU using CUDA (Recommended)"
    tensorrt_tooltip = "Run model on GPU using TensorRT. Faster than CUDA but prone to loading errors"
    id_cpu = "CPU"
    id_cuda = "CUDA"
    id_tensorrt = "TENSORRT"
    desc_cpu = "CPU"
    desc_cuda = "CUDA"
    desc_tensorrt = "TensorRT"
    exec_list_no_trt = [
        (id_cpu, desc_cpu, cpu_tooltip, 1),
        (id_cuda, desc_cuda, cuda_tooltip, 2),
    ]
    exec_list_trt = [
        (id_cpu, desc_cpu, cpu_tooltip, 1),
        (id_cuda, desc_cuda, cuda_tooltip, 2),
        (id_tensorrt, desc_tensorrt, tensorrt_tooltip, 3)
    ]

    # Model selection text
    simple_selection_det_list_name = "Detection model preference"
    simple_selection_pose_list_name = "Pose model preference"
    tag_detector_ml_model_sel_list_name = "Tag model preference"
    pose_model_sel_list_name = "Pose model preference"
    simple_selection_list_desc = "Automatically choose the best model based on certain criteria"
    simp_sel_fast_tooltip = "Choose the fastest model available"
    simp_sel_bal_tooltip = "Choose a model that is both fast and somewhat accurate (Recommended)"
    simp_sel_bal_mem_tooltip = "Choose a model that is fast, accurate, and has a small memory footprint"
    simp_sel_accurate_tooltip = "Choose the model with the highest accuracy"
    id_simp_sel_fast = "FAST"
    id_simp_sel_bal = "BALANCED"
    id_simp_sel_bal_mem = "BAL_MEM"
    id_simp_sel_accurate = "ACCURATE"
    desc_simp_sel_fast = "Fast"
    desc_simp_sel_bal = "Balanced"
    desc_simp_sel_bal_mem = "Balanced Memory"
    desc_simp_sel_accurate = "Accurate"
    simp_model_sel_items = [
        (id_simp_sel_fast, desc_simp_sel_fast, simp_sel_fast_tooltip, 1),
        (id_simp_sel_bal, desc_simp_sel_bal, simp_sel_bal_tooltip, 2),
        (id_simp_sel_bal_mem, desc_simp_sel_bal_mem, simp_sel_bal_mem_tooltip, 3),
        (id_simp_sel_accurate, desc_simp_sel_accurate, simp_sel_accurate_tooltip, 4),
    ]

    # Tag model selection text
    tag_detector_type_list_name = "Tag detector type"
    tag_detector_type_list_desc = "Set the desired detection method"
    tag_detector_type_cv_tooltip = "Standard CV detector. Must specify target marker dictionary (Recommended)."
    tag_detector_type_ml_tooltip = "Machine learning based detector"
    id_tag_detector_type_cv = "CV"
    id_tag_detector_type_ml = "ML"
    desc_tag_detector_type_cv = id_tag_detector_type_cv
    desc_tag_detector_type_ml = id_tag_detector_type_ml
    tag_detector_cv_dict_list_name = "Target marker dictionary"
    tag_detector_cv_dict_list_desc = "Set the target marker dictionary"
    tag_detector_cv_resample_toggle_name = "Resample input"
    tag_detector_cv_resample_toggle_desc = ("Resample detection to improve subpixel accuracy. "
                                            "Disable if having issues with tracking")
    tag_detector_type_items = [
        (id_tag_detector_type_cv, desc_tag_detector_type_cv, tag_detector_type_cv_tooltip, 1),
        (id_tag_detector_type_ml, desc_tag_detector_type_ml, tag_detector_type_ml_tooltip, 2),
    ]

    pose_estimation_target_list_name = "Target type"
    pose_estimation_target_list_desc = "Set the pose target type"

    me_ui_active_track_name: bpy.props.StringProperty(
        name='Track name',
        description='Name of the active track',
        default='',
        get=get_track_name,
        set=set_track_name
    )
    me_ui_redraw_prop: bpy.props.BoolProperty(
        name="Update property",
        description="Hidden prop",
        default=False,
        update=force_redraw
    )
    me_ui_prop_overwrite_poses: bpy.props.BoolProperty(
        name="Overwrite existing poses",
        description="If unchecked, will prioritize whichever estimate has the higher confidence score",
        default=True
    )
    me_ui_prop_mute_results: bpy.props.BoolProperty(
        name="Mute tracking results",
        description="If enabled, markers will be muted once the detector has finished",
        default=True
    )
    me_ui_tag_detector_cv_resample_toggle_prop: bpy.props.BoolProperty(
        name=tag_detector_cv_resample_toggle_name,
        description=tag_detector_cv_resample_toggle_desc,
        default=False
    )
    me_ui_prop_pose_target_enum: bpy.props.EnumProperty(
        name=pose_estimation_target_list_name,
        description=pose_estimation_target_list_desc,
        items=pose_estimation_targets_enum_items,
        default=pose_estimation_targets_default,
        update=update_pose_keypoints,
    )
    me_ui_prop_pose_keypoints_enum: bpy.props.EnumProperty(
        name="Pose keypoints",
        description="Sets the number of keypoints to detect per pose",
        items=get_keypoint_items,
        default=1
    )
    me_ui_tag_detector_cv_dict_list_enum: bpy.props.EnumProperty(
        name=tag_detector_cv_dict_list_name,
        description=tag_detector_cv_dict_list_desc,
        items=tag_detector_cv_dict_list_items
    )
    me_ui_prop_tag_detector_type_enum: bpy.props.EnumProperty(
        name=tag_detector_type_list_name,
        description=tag_detector_type_list_desc,
        items=tag_detector_type_items,
    )
    me_ui_prop_tag_detector_ml_model_sel_enum: bpy.props.EnumProperty(
        name=tag_detector_ml_model_sel_list_name,
        description=simple_selection_list_desc,
        items=simp_model_sel_items,
        default=2
    )
    me_ui_prop_pose_model_sel_enum: bpy.props.EnumProperty(
        name=pose_model_sel_list_name,
        description=simple_selection_list_desc,
        items=simp_model_sel_items,
        default=2
    )
    me_ui_prop_exe_tag_detector_ml_enum: bpy.props.EnumProperty(
        name=executor_list_name,
        description=executor_list_desc,
        items=exec_list_no_trt,
        default=2
    )
    me_ui_prop_exe_det_enum: bpy.props.EnumProperty(
        name=executor_list_name,
        description=executor_list_desc,
        items=exec_list_no_trt,
        default=2
    )
    me_ui_prop_exe_pose_enum: bpy.props.EnumProperty(
        name=executor_list_name,
        description=executor_list_desc,
        items=exec_list_no_trt,
        default=2
    )
    me_ui_prop_exe_track_enum: bpy.props.EnumProperty(
        name=executor_list_name,
        description=executor_list_desc,
        items=exec_list_no_trt,
        default=2
    )
    me_ui_prop_exe_det_tag_enum: bpy.props.EnumProperty(
        name=executor_list_name,
        description=executor_list_desc,
        items=exec_list_no_trt,
        default=2
    )
    me_ui_prop_det_batch_size: bpy.props.IntProperty(
        name="Batch size for detection model",
        description="Sets the amount of frames to be processed by the detection model at any given time",
        default=32,
        min=1,
        max=256
    )
    me_ui_prop_pose_batch_size: bpy.props.IntProperty(
        name="Batch size for pose model",
        description="Sets the amount of detections to be processed by the pose model at any given time",
        default=32,
        min=1,
        max=256
    )
    me_ui_prop_solution_scale: bpy.props.FloatProperty(
        name="Solution scale",
        description="Sets solution scale for cameras",
        default=1,
        min=0,
    )
    me_ui_prop_det_conf: bpy.props.FloatProperty(
        name="Confidence threshold",
        description="Sets the minimum acceptable confidence score for detections",
        default=0.5,
        min=0,
        max=1
    )
    me_ui_prop_joint_conf: bpy.props.FloatProperty(
        name="Confidence threshold",
        description="Sets the minimum acceptable confidence score for pose joints",
        default=0.5,
        min=0,
        max=1
    )
    me_ui_prop_det_iou: bpy.props.FloatProperty(
        name="IoU threshold",
        description="Sets the maximum acceptable overlap between detections. Non-maximum suppression will be used on "
                    "boxes with overlap values higher than this",
        default=0.5,
        min=0,
        max=1
    )
    me_ui_prop_det_tag_conf: bpy.props.FloatProperty(
        name="Confidence threshold",
        description="Sets the minimum acceptable confidence score for detections",
        default=0.5,
        min=0,
        max=1
    )
    me_ui_prop_det_tag_iou: bpy.props.FloatProperty(
        name="IoU threshold",
        description="Sets the maximum acceptable overlap between detections. Non-maximum suppression will be used on "
                    "boxes with overlap values higher than this",
        default=0.5,
        min=0,
        max=1
    )
    me_ui_prop_track_score: bpy.props.FloatProperty(
        name="Distance threshold",
        description="Sets the maximum feature distance for tracking candidates",
        default=1.4,
        min=0
    )
    me_ui_prop_track_reid_score: bpy.props.FloatProperty(
        name="ReID threshold",
        description="Sets the maximum feature distance for reID candidates",
        default=1.0,
        min=0
    )
    me_ui_prop_track_dist_type: bpy.props.EnumProperty(
        name="Distance type",
        description="Equation to use for feature distance calculations",
        items=[
            ("EUCLIDEAN", "L2 norm", "Euclidean L2 norm", 1),
            ("NORM_EUCLIDEAN", "Norm L2 norm", "Normalized Euclidean L2 norm (Recommended)", 2),
        ],
        default=2
    )
    me_ui_prop_det_simple_sel_enum: bpy.props.EnumProperty(
        name=simple_selection_det_list_name,
        description=simple_selection_list_desc,
        items=simp_model_sel_items,
        default=2,
    )
    me_ui_prop_det_tag_simple_sel_enum: bpy.props.EnumProperty(
        name=simple_selection_det_list_name,
        description=simple_selection_list_desc,
        items=simp_model_sel_items,
        default=2,
    )
    me_ui_prop_det_class_enum: bpy.props.EnumProperty(
        name="Target",
        description="Set the target object",
        items=obj_det_class_enum_items,
        default=obj_det_class_default,
    )
    me_ui_prop_det_thresholding_enum: bpy.props.EnumProperty(
        name="Thresholding mode",
        description="Set threshold values automatically or manually",
        items=[
            ("AUTO", "Auto", "Set threshold values automatically (Recommended)"),
            ("MANUAL", "Manual", "Set threshold values manually"),
        ],
        default="AUTO",
        update=det_thresh_mode_update
    )
    me_ui_prop_track_thresholding_enum: bpy.props.EnumProperty(
        name="Thresholding mode",
        description="Set threshold values automatically or manually",
        items=[
            ("AUTO", "Auto", "Set threshold values automatically (Recommended)"),
            ("MANUAL", "Manual", "Set threshold values manually"),
        ],
        default="AUTO",
        update=track_thresh_mode_update
    )
    me_ui_prop_det_tag_thresholding_enum: bpy.props.EnumProperty(
        name="Thresholding mode",
        description="Set threshold values automatically or manually",
        items=[
            ("AUTO", "Auto", "Set threshold values automatically (Recommended)"),
            ("MANUAL", "Manual", "Set threshold values manually"),
        ],
        default="AUTO",
        update=det_tag_thresh_mode_update
    )
    me_ui_prop_pose_thresholding_enum: bpy.props.EnumProperty(
        name="Thresholding mode",
        description="Set threshold values automatically or manually",
        items=[
            ("AUTO", "Auto", "Set threshold values automatically (Recommended)"),
            ("MANUAL", "Manual", "Set threshold values manually"),
        ],
        default="AUTO",
        update=pose_thresh_mode_update
    )
    me_ui_prop_rigging_avg_locked_axis: bpy.props.IntProperty(
        min=0,
        max=2,
        default=0
    )
    me_ui_prop_anchor_cam_selection: bpy.props.PointerProperty(
        type=bpy.types.MovieClip,
        name="Clip to use as anchor view",
    )


CLASSES = [
    MotionEngineUIProperties
]
