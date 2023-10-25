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
from .me_data import MEPoseTracksClip
from .me_data import MEPoseTracks
from .me_stats import MotionEngineRunStatistics
from ..utils import set_select_tracks


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


def cache_method_update(self, context):
    scene = context.scene
    properties = scene.motion_engine_ui_properties
    if properties.me_ui_prop_cache_method_enum == "AUTO":
        properties.me_ui_prop_cache_size = 256
    return None


def active_pose_update(self, context):
    scene = context.scene
    properties = scene.motion_engine_ui_properties
    current_clip = context.edit_movieclip
    clip_tracks = current_clip.tracking.tracks

    target_list = None

    for item in properties.me_ui_prop_pose_clip_collection:
        if item.clip == current_clip:
            target_list = item.pose_tracks_list
            break

    if target_list is not None:
        target = target_list[properties.me_ui_prop_active_pose_index]

        if target is not None:

            all_pose_tracks = []
            target_pose_tracks = []

            for other in target_list:
                for i in range(other.tracks):
                    track = clip_tracks.get(other.track_prefix + '.' + str(i))
                    if track is not None:
                        all_pose_tracks.append(track)

            for i in range(target.tracks):
                track = clip_tracks.get(target.track_prefix + '.' + str(i))
                if track is not None:
                    target_pose_tracks.append(track)

            set_select_tracks(all_pose_tracks, False)
            set_select_tracks(target_pose_tracks, True)

    return None


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

    # Pose model depth text
    depth_list_name = "Pose model depth"
    depth_list_desc = "Sets the depth of the pose model"
    tiny_tooltip = "Minimum depth. Has the least prediction accuracy"
    small_tooltip = "Small depth. Decent accuracy and execution speed (Recommended)"
    medium_tooltip = "Medium depth. Slower, but predictions are more stable"
    medium_tooltip_133 = "Medium depth. Decent accuracy and execution speed (Recommended)"
    large_tooltip = "Large depth. Slowest, but has the highest accuracy"
    id_tiny = "t"
    id_small = "s"
    id_medium = "m"
    id_large = "l"
    desc_tiny = "Tiny"
    desc_small = "Small"
    desc_medium = "Medium"
    desc_large = "Large"

    # Pose model precision text
    precision_list_name = "Pose model precision"
    precision_list_desc = "Sets the coordinate precision of the pose model"
    single_tooltip = "Standard precision (Recommended)"
    double_tooltip = "Double precision. Slower, but has higher accuracy"
    id_single = "SINGLE"
    id_double = "DOUBLE"
    desc_single = "Single"
    desc_double = "Double"

    me_ui_redraw_prop: bpy.props.BoolProperty(
        name="Update property",
        description="Hidden prop",
        default=False,
        update=force_redraw
    )
    me_ui_prop_remove_poses: bpy.props.BoolProperty(
        name="Remove existing poses",
        description="Delete existing poses on new analysis",
        default=False
    )
    me_ui_prop_exe_det_enum: bpy.props.EnumProperty(
        name=executor_list_name,
        description=executor_list_desc,
        items=[
            (id_cpu, desc_cpu, cpu_tooltip, 1),
            (id_cuda, desc_cuda, cuda_tooltip, 2),
        ],
        default=2
    )
    me_ui_prop_exe_pose_enum: bpy.props.EnumProperty(
        name=executor_list_name,
        description=executor_list_desc,
        items=[
            (id_cpu, desc_cpu, cpu_tooltip, 1),
            (id_cuda, desc_cuda, cuda_tooltip, 2),
            (id_tensorrt, desc_tensorrt, tensorrt_tooltip, 3),
        ],
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
    me_ui_prop_det_conf: bpy.props.FloatProperty(
        name="Confidence threshold",
        description="Sets the minimum acceptable confidence score for detections",
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
    me_ui_prop_det_size_enum: bpy.props.EnumProperty(
        name="Input size",
        description="Sets the desired input size. A higher input size can help if there aren't any detections on "
                    "certain frames",
        items=[
            ("320", "320", "Set input size to 320x320 (Recommended)"),
            ("640", "640", "Set input size to 640x640"),
        ],
        default="320",
        update=det_thresh_mode_update
    )
    me_ui_prop_pose_keypoints_enum: bpy.props.EnumProperty(
        name="Pose keypoints",
        description="Sets the number of keypoints to detect per pose",
        items=[
            ("17", "17", "Standard set"),
            ("26", "26", "Extended set. Includes points for feet, neck, pelvis, and head axis (Recommended)"),
            ("133", "133", "Detailed set. Same as standard but includes additional points for face, hands, and feet"),
        ],
        default="26"
    )
    me_ui_prop_pose_depth17_enum: bpy.props.EnumProperty(
        name=depth_list_name,
        description=depth_list_desc,
        items=[
            (id_tiny, desc_tiny, tiny_tooltip, 1),
            (id_small, desc_small, small_tooltip, 2),
            (id_medium, desc_medium, medium_tooltip, 3),
            (id_large, desc_large, large_tooltip, 4),
        ],
        default=2
    )
    me_ui_prop_pose_depth26_enum: bpy.props.EnumProperty(
        name=depth_list_name,
        description=depth_list_desc,
        items=[
            (id_tiny, desc_tiny, tiny_tooltip, 1),
            (id_small, desc_small, small_tooltip, 2),
            (id_medium, desc_medium, medium_tooltip, 3),
            (id_large, desc_large, large_tooltip, 4),
        ],
        default=2
    )
    me_ui_prop_pose_depth133_enum: bpy.props.EnumProperty(
        name=depth_list_name,
        description=depth_list_desc,
        items=[
            (id_medium, desc_medium, medium_tooltip_133, 3),
            (id_large, desc_large, large_tooltip, 4),
        ],
        default=3
    )
    me_ui_prop_pose_precision17m_enum: bpy.props.EnumProperty(
        name=precision_list_name,
        description=precision_list_desc,
        items=[
            (id_single, desc_single, single_tooltip, 1),
            (id_double, desc_double, double_tooltip, 2),
        ],
        default=1
    )
    me_ui_prop_pose_precision17l_enum: bpy.props.EnumProperty(
        name=precision_list_name,
        description=precision_list_desc,
        items=[
            (id_single, desc_single, single_tooltip, 1),
            (id_double, desc_double, double_tooltip, 2),
        ],
        default=1
    )
    me_ui_prop_pose_precision26m_enum: bpy.props.EnumProperty(
        name=precision_list_name,
        description=precision_list_desc,
        items=[
            (id_single, desc_single, single_tooltip, 1),
            (id_double, desc_double, double_tooltip, 2),
        ],
        default=1
    )
    me_ui_prop_pose_precision26l_enum: bpy.props.EnumProperty(
        name=precision_list_name,
        description=precision_list_desc,
        items=[
            (id_single, desc_single, single_tooltip, 1),
            (id_double, desc_double, double_tooltip, 2),
        ],
        default=1
    )
    me_ui_prop_pose_precision133l_enum: bpy.props.EnumProperty(
        name=precision_list_name,
        description=precision_list_desc,
        items=[
            (id_single, desc_single, single_tooltip, 1),
            (id_double, desc_double, double_tooltip, 2),
        ],
        default=1
    )
    me_ui_prop_det_thresholding_enum: bpy.props.EnumProperty(
        name="Thresholding mode",
        description="Lets you decide whether to set threshold values automatically or manually",
        items=[
            ("AUTO", "Auto", "Set threshold values automatically (Recommended)"),
            ("MANUAL", "Manual", "Set threshold values manually"),
        ],
        default="AUTO",
        update=det_thresh_mode_update
    )
    me_ui_prop_cache_method_enum: bpy.props.EnumProperty(
        name="Cache configuration mode",
        description="Lets you decide whether to set image cache size automatically or manually",
        items=[
            ("AUTO", "Auto", "Set image cache automatically (Recommended)"),
            ("MANUAL", "Manual", "Set image cache size manually"),
        ],
        default="AUTO",
        update=cache_method_update
    )
    me_ui_prop_cache_size: bpy.props.IntProperty(
        name="Pre-inference cache size",
        description="Number of frames to load into memory prior to inference",
        default=256,
        min=1,
        max=1024
    )
    me_ui_prop_stats_collection: bpy.props.CollectionProperty(type=MotionEngineRunStatistics)
    me_ui_prop_pose_clip_collection: bpy.props.CollectionProperty(type=MEPoseTracksClip)
    me_ui_prop_active_pose_index: bpy.props.IntProperty(
        name="Active Pose Index",
        description="Active pose in the pose tracks list",
        update=active_pose_update
    )

    # Placeholder property. Must always remain empty.
    me_ui_prop_pose_empty_clip: bpy.props.PointerProperty(type=MEPoseTracksClip)


CLASSES = [
    MotionEngineUIProperties
]
