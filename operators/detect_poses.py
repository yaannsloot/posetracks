"""
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
"""

import math

import bpy

from .. import posetracks_core as pt_core
from .. import global_vars
from .. import events
from ..events import (InfoEvent,
                      CancelledEvent,
                      ErrorEvent,
                      PoseFinishedEvent)
from . import event_ops
from .event_ops import (EventOperator,
                        event_queue,
                        load_model_by_criteria,
                        setup_frame_provider,
                        get_frames,
                        batch_infer,
                        LoadException,
                        CancelledException)
import traceback


def adjust_det_aspect(det, net_size):
    aspect_ratio = net_size[0] / net_size[1]
    width = det.bbox.width
    height = det.bbox.height
    tl = det.bbox.tl()
    br = det.bbox.br()
    center = ((tl.x + br.x) / 2, (tl.y + br.y) / 2)
    if width > (aspect_ratio * height):
        height = width / aspect_ratio
    elif width < (aspect_ratio * height):
        width = height * aspect_ratio
    new_tl = (center[0] - width / 2, center[1] - height / 2)
    det.bbox = pt_core.Rect(new_tl[0], new_tl[1], width, height)


def pose_task(bpy_data: event_ops.BpyData):
    sorting_criteria = {
        'FAST': ['gflops'],
        'BALANCED': ['precision_ap', 'gflops'],
        'BAL_MEM': ['precision_ap', 'gflops', 'mparams'],
        'ACCURATE': ['precision_ap']
    }
    try:
        pose_model, _ = load_model_by_criteria('pose_estimation',
                                               bpy_data.ui_props['exe_pose_enum'],
                                               values={'target_class': bpy_data.ui_props['pose_target_enum'],
                                                       'keypoints': int(bpy_data.ui_props['pose_keypoints_enum'])},
                                               sorting_criteria=sorting_criteria[
                                                   bpy_data.ui_props['pose_model_sel_enum']])
        clip = setup_frame_provider(bpy_data.clip_info.abs_path, bpy_data.clip_info.source_type)
        scene_start = bpy_data.clip_info.scene_to_true(bpy_data.scene_first_frame)
        scene_end = bpy_data.clip_info.scene_to_true(bpy_data.scene_last_frame)
        data_frames = list(bpy_data.clip_tracks.detections.keys())
        clip_tracks_max_f = max(data_frames, default=0)
        clip_tracks_min_f = min(data_frames, default=0)
        tracks_min = bpy_data.clip_info.scene_to_true(clip_tracks_min_f)
        tracks_max = bpy_data.clip_info.scene_to_true(clip_tracks_max_f)
        first_frame = max(max(scene_start, tracks_min), 0)
        last_frame = min(min(scene_end, tracks_max), clip.frame_count())
        preload_size = 32  # frames to load before samples are taken
        batch_size = 32
        clip.set_frame(first_frame)
        f_num = first_frame
        all_poses = {}
        while True:
            if event_ops.cancel_task or global_vars.shutdown_state:
                raise CancelledException('Pose estimation cancelled')
            frames = get_frames(clip, preload_size, last_frame)
            if not frames:
                break

            samples = []
            sample_boxes = []
            sample_ids = []
            sample_frames = []
            for i in range(len(frames)):
                true_frame = f_num + i
                scene_frame = bpy_data.clip_info.true_to_scene(true_frame)
                if scene_frame not in bpy_data.clip_tracks.detections:
                    continue
                frame_data = bpy_data.clip_tracks.detections[scene_frame]
                for track in frame_data:
                    det = frame_data[track]
                    if pt_core.dnn.is_roi_outside_image(clip.frame_size(), det.bbox):
                        continue
                    adjust_det_aspect(det, pose_model.net_size())
                    det.scale_detection(1.2)
                    sample = pt_core.dnn.get_roi_with_padding(frames[i], det.bbox)
                    samples.append(sample)
                    sample_boxes.append(det)
                    sample_ids.append(track)
                    sample_frames.append(true_frame)

            poses, batch_size = batch_infer(batch_size, pose_model, samples)

            for i in range(len(samples)):
                clip_frame = bpy_data.clip_info.true_to_clip(sample_frames[i])
                det = sample_boxes[i]
                pose_id = sample_ids[i]
                pose = poses[i]
                for j in range(pose.num_joints()):
                    x = pose[j].pt.x
                    y = pose[j].pt.y
                    x = det.bbox.x + x * det.bbox.width
                    y = det.bbox.y + y * det.bbox.height
                    pose[j].pt.x = x
                    pose[j].pt.y = y
                if clip_frame not in all_poses:
                    all_poses[clip_frame] = {}
                all_poses[clip_frame][pose_id] = pose

            f_num += len(frames)
            percent_current = int(max(0, min(100 * ((f_num - first_frame) / (last_frame - first_frame)), 100)))
            event_queue.put(events.InfoEvent(f'Estimating poses: {percent_current}% (ESC to cancel)'))

        event_queue.put(PoseFinishedEvent(all_poses, 'Pose estimation task completed'))
        event_queue.put(InfoEvent('Done'))
    except MemoryError as e:
        event_queue.put(ErrorEvent("Out of memory", str(e)))
    except CancelledException as e:
        event_queue.put(CancelledEvent(str(e)))
    except LoadException as e:
        event_queue.put(ErrorEvent(str(e), e.detailed_message))
    except:
        event_queue.put(ErrorEvent(f'Error in pose estimation task: {traceback.format_exc()}',
                                   traceback.format_exc()))


class DetectPosesOperator(EventOperator):
    """Convert tracks to poses"""
    bl_idname = "posetracks.detect_poses_operator"
    bl_label = "Estimate Poses"

    overwrite: bpy.props.BoolProperty(
        name="Overwrite",
        description="If unchecked, will prioritize whichever estimate has the higher confidence score",
        default=True
    )

    def __init__(self):
        super().__init__()
        self.task_func = pose_task
        self.include_tracking_data = True
        self.selected_tracks_only = True
        self.filter_joints_and_tags = False

    def execute(self, context):
        if self.poses is None:
            return {'FINISHED'}
        try:
            clip_info = self.bpy_data.clip_info
            clip = bpy.data.movieclips[self.bpy_data.clip_info.bpy_name]
            clip_size = clip_info.clip_size
            tracks = clip.tracking.tracks
            conf_thresh = self.bpy_data.ui_props['joint_conf']
            pose_keypoints = self.bpy_data.ui_props['pose_keypoints_enum']
            pose_source = self.bpy_data.ui_props['pose_target_enum']
            for frame, poses in self.poses.items():
                scene_frame = self.bpy_data.clip_info.clip_to_scene(frame)
                for pose_id, pose in poses.items():
                    for j in range(pose.num_joints()):
                        if pose[j].prob < conf_thresh:
                            continue
                        track_name = f'{pose_id}.{pose_source}{pose_keypoints}.{j}'
                        if (not self.overwrite and scene_frame in self.bpy_data.clip_tracks.detections and
                                track_name in self.bpy_data.clip_tracks.detections[scene_frame]):
                            old_conf = self.bpy_data.clip_tracks[scene_frame][track_name].bbox.area() / 100
                            if old_conf > pose[j].prob:
                                continue
                        track = tracks.get(track_name)
                        if track is None:
                            track = tracks.new(name=track_name, frame=frame)
                        x = pose[j].pt.x / clip_size[0]
                        y = (clip_size[1] - pose[j].pt.y) / clip_size[1]
                        box_area = pose[j].prob * 100
                        side_length = math.sqrt(box_area)
                        x1 = -1 * (side_length / 2) / clip_size[0]
                        x2 = (side_length / 2) / clip_size[0]
                        y1 = -1 * (side_length / 2) / clip_size[1]
                        y2 = (side_length / 2) / clip_size[1]
                        marker = track.markers.insert_frame(frame, co=(x, y))
                        marker.pattern_corners = ((x1, y1),
                                                  (x2, y1),
                                                  (x2, y2),
                                                  (x1, y2))
                        marker.search_max *= 1.2
                        marker.search_min *= 1.2
                        marker.mute = True
                        marker.is_keyed = False
        except Exception:
            error = f'Error during evaluation: {traceback.format_exc()}'
            self.report({'ERROR'}, error)
        return {'FINISHED'}


CLASSES = [
    DetectPosesOperator
]
