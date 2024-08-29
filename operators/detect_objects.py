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

import bpy

from .. import posetracks_core as pt_core
from .. import global_vars
from ..events import (InfoEvent,
                      CancelledEvent,
                      ErrorEvent,
                      DetectionFinishedEvent)
from . import event_ops
from .event_ops import (EventOperator,
                        event_queue,
                        load_model_by_criteria,
                        load_model_by_name,
                        setup_frame_provider,
                        get_frames,
                        batch_infer,
                        LoadException,
                        CancelledException)
import traceback


def track_name(det_id, class_id):
    return f'{class_id.capitalize()} {det_id}'


def get_track_name_for_detection(clip, class_id):
    det_id = 0
    while True:
        name = track_name(det_id, class_id)
        if name not in clip.tracking.tracks:
            return name
        det_id += 1


def detection_task(bpy_data):
    sorting_criteria = {
        'FAST': ['gflops'],
        'BALANCED': ['precision_ap', 'gflops'],
        'BAL_MEM': ['precision_ap', 'gflops', 'mparams'],
        'ACCURATE': ['precision_ap']
    }
    try:
        det_model, det_model_sel = load_model_by_criteria('object_detection',
                                                          bpy_data.ui_props['exe_det_enum'],
                                                          values={'classes': [bpy_data.ui_props['det_class_enum']]},
                                                          sorting_criteria=sorting_criteria[
                                                              bpy_data.ui_props['det_simple_sel_enum']])
        try:
            target_cid = det_model_sel['classes'].index(bpy_data.ui_props['det_class_enum'])
        except ValueError:
            target_cid = 0
        feat_model, _ = load_model_by_name('feature_extraction', 'basic_conv_person_64',
                                           bpy_data.ui_props['exe_track_enum'])
        clip = setup_frame_provider(bpy_data.clip_info.abs_path, bpy_data.clip_info.source_type)
        event_queue.put(InfoEvent('Initializing tracker...', 'Obtaining feature length from model...'))
        init_result = feat_model.infer(pt_core.rand_img_rgb(feat_model.net_size()))
        tracker = pt_core.dnn.FeatureTracker(len(init_result))
        det_batch_size = 32
        feat_batch_size = 32
        first_frame = max(bpy_data.clip_info.scene_to_true(bpy_data.scene_first_frame), 0)
        last_frame = min(bpy_data.clip_info.scene_to_true(bpy_data.scene_last_frame), clip.frame_count())
        clip.set_frame(first_frame)
        f_num = first_frame
        all_detections = {}
        while True:
            if event_ops.cancel_task or global_vars.shutdown_state:
                raise CancelledException('Object detection cancelled')
            frames = get_frames(clip, det_batch_size, last_frame)
            if not frames:
                break
            detections, det_batch_size = batch_infer(det_batch_size, det_model, frames,
                                                     bpy_data.ui_props['det_conf'],
                                                     bpy_data.ui_props['det_iou'])
            if not detections:
                f_num += len(frames)
                continue
            detections = pt_core.dnn.fix_detection_coordinates(detections, det_model.net_size(), clip.frame_size(),
                                                          pt_core.dnn.AUTO)
            roi_detections = []
            roi_samples = []
            splits = []
            split_frames = []
            for i in range(len(detections)):
                for det in detections[i]:
                    if (not pt_core.dnn.is_roi_outside_image(clip.frame_size(), det.bbox)
                            and det.class_id == target_cid):
                        roi_detections.append(det)
                        roi_samples.append(pt_core.dnn.get_roi_with_padding(frames[i], det.bbox))
                if detections[i]:
                    splits.append(len(roi_detections))
                    split_frames.append(i)
            features, feat_batch_size = batch_infer(feat_batch_size, feat_model, roi_samples)
            beg = 0
            s = 0
            for i in range(len(detections)):
                if i not in split_frames:
                    tracker.assign([], [])
                    continue
                frame_feats = features[beg:splits[s]]
                frame_dets = roi_detections[beg:splits[s]]
                det_ids = tracker.assign(frame_dets, frame_feats,
                                         bpy_data.ui_props['track_score'],
                                         bpy_data.ui_props['track_reid_score'],
                                         bpy_data.ui_props['track_dist_type'])
                for id, det in zip(det_ids, frame_dets):
                    if i + f_num not in all_detections:
                        all_detections[i + f_num] = {}
                    all_detections[i + f_num][id] = det
                beg = splits[s]
                s += 1
            f_num += len(frames)
            percent_current = int(max(0, min(100 * ((f_num - first_frame) / (last_frame - first_frame)), 100)))
            event_queue.put(InfoEvent(f'Detecting objects: {percent_current}% (ESC to cancel)'))
        event_queue.put(DetectionFinishedEvent(all_detections, 'Detection task completed'))
        event_queue.put(InfoEvent('Done'))
    except MemoryError as e:
        event_queue.put(ErrorEvent("Out of memory", str(e)))
    except CancelledException as e:
        event_queue.put(CancelledEvent(str(e)))
    except LoadException as e:
        event_queue.put(ErrorEvent(str(e), e.detailed_message))
    except:
        event_queue.put(ErrorEvent(f'Error in object detection task: {traceback.format_exc()}',
                                   traceback.format_exc()))


class DetectObjectsOperator(EventOperator):
    """Scan playback range for target objects"""
    bl_idname = "posetracks.detect_objects_operator"
    bl_label = "Detect Objects"

    mute_results: bpy.props.BoolProperty(
        name="Mute Tracks",
        description="If enabled, all resulting tracks will be muted",
        default=True
    )

    def __init__(self):
        super().__init__()
        self.task_func = detection_task

    def execute(self, context):
        if self.detections is None:
            return {'FINISHED'}
        try:
            mute_results = self.mute_results
            class_id = str(self.bpy_data.ui_props['det_class_enum'])
            clip = bpy.data.movieclips[self.bpy_data.clip_info.bpy_name]
            clip_info = self.bpy_data.clip_info
            clip_size = clip_info.clip_size
            tracks = clip.tracking.tracks
            init_dict = {}
            for frame in self.detections:
                actual_frame = clip_info.true_to_clip(frame)
                frame_detections = self.detections[frame]
                for det_id in frame_detections:
                    detection = frame_detections[det_id]
                    bbox = detection.bbox
                    if det_id not in init_dict:
                        init_dict[det_id] = get_track_name_for_detection(clip, class_id)
                    det_name = init_dict[det_id]
                    track = tracks.get(det_name)
                    if track is None:
                        track = tracks.new(name=det_name, frame=actual_frame)
                    markers = track.markers
                    x = (bbox.x + bbox.width / 2) / clip_size[0]
                    y = (clip_size[1] - (bbox.y + bbox.height / 2)) / clip_size[1]
                    marker = markers.insert_frame(actual_frame, co=(x, y))
                    x1 = -1 * (bbox.width / 2) / clip_size[0]
                    x2 = (bbox.width / 2) / clip_size[0]
                    y1 = -1 * (bbox.height / 2) / clip_size[1]
                    y2 = (bbox.height / 2) / clip_size[1]

                    marker.pattern_corners = ((x1, y1),
                                              (x2, y1),
                                              (x2, y2),
                                              (x1, y2))
                    marker.search_max *= 1.2
                    marker.search_min *= 1.2
                    marker.is_keyed = False
                    marker.mute = mute_results
        except Exception:
            error = f'Error during evaluation: {traceback.format_exc()}'
            self.report({'ERROR'}, error)
        return {'FINISHED'}


CLASSES = [
    DetectObjectsOperator
]
