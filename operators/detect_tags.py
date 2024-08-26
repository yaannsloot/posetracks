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
from .. import utils
from ..events import (InfoEvent,
                      CancelledEvent,
                      ErrorEvent,
                      TagFinishedEvent)
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

"""
NOTE TO SELF
THIS DOESN'T SUPPORT ML TAG DETECTORS (YET)
PLEASE FIX WHEN MODELS ARE AVAILABLE
UPDATE: It does now. Kinda :)
Does not actually use the model selection system. 
Fix when more are available and performance data is collected. 
"""


def tag_task(bpy_data: event_ops.BpyData):
    # Normally sorting criteria would be defined here for ML corner detection models.
    # Since basic_reg_v1 is the only model currently in use, this will be omitted.
    # If more models are to be added, the appropriate sorting criteria should be defined here.
    tag_sorting_criteria = None

    # tag candidate detection model
    det_sorting_criteria = {
        'FAST': ['gflops'],
        'BALANCED': ['precision_ap', 'gflops'],
        'BAL_MEM': ['precision_ap', 'gflops', 'mparams'],
        'ACCURATE': ['precision_ap']
    }
    try:
        if bpy_data.ui_props['tag_detector_type_enum'] == 'ML':
            tag_model, _ = load_model_by_name('tag_detection', 'basic_reg_v1',
                                              bpy_data.ui_props['exe_tag_detector_ml_enum'])
        else:
            tag_model = me.dnn.CVTagDetector()
            tag_model.set_dict_type(bpy_data.ui_props['tag_detector_cv_dict_list_enum'])
            if bpy_data.ui_props['tag_detector_cv_resample_toggle']:
                tag_model.set_preprocess_size((224, 224))
        det_model, det_model_sel = load_model_by_criteria('object_detection',
                                                          bpy_data.ui_props['exe_det_tag_enum'],
                                                          values={'classes': ['Tag']},
                                                          sorting_criteria=det_sorting_criteria[
                                                              bpy_data.ui_props['det_tag_simple_sel_enum']])
        try:
            target_cid = det_model_sel['classes'].index('Tag')
        except ValueError:
            target_cid = 0
        clip = setup_frame_provider(bpy_data.clip_info.abs_path, bpy_data.clip_info.source_type)
        tag_batch_size = 32
        det_batch_size = 32
        first_frame = max(bpy_data.clip_info.scene_to_true(bpy_data.scene_first_frame), 0)
        last_frame = min(bpy_data.clip_info.scene_to_true(bpy_data.scene_last_frame), clip.frame_count())
        clip.set_frame(first_frame)
        f_num = first_frame
        all_tags = {}
        while True:
            if event_ops.cancel_task or global_vars.shutdown_state:
                raise CancelledException('Tag detection cancelled')
            frames = get_frames(clip, det_batch_size, last_frame)
            if not frames:
                break
            detections, det_batch_size = batch_infer(det_batch_size, det_model, frames,
                                                     bpy_data.ui_props['det_tag_conf'],
                                                     bpy_data.ui_props['det_tag_iou'])
            if not detections:
                f_num += len(frames)
                continue
            detections = me.dnn.fix_detection_coordinates(detections, det_model.net_size(), clip.frame_size(),
                                                          me.dnn.AUTO)
            samples = []
            sample_frames = []
            sample_regions = []
            for f, frame_dets in enumerate(detections):
                for det in frame_dets:
                    det.scale_detection(1.2)
                    if (me.dnn.is_roi_outside_image(clip.frame_size(), det.bbox)
                            or det.class_id != target_cid):
                        continue
                    samples.append(me.dnn.get_roi_no_padding(frames[f], det.bbox))
                    sample_frames.append(f)
                    sample_regions.append(det)
            if isinstance(tag_model, me.dnn.CVTagDetector):
                tags = tag_model.infer(samples)
            else:
                tags, tag_batch_size = batch_infer(tag_batch_size, tag_model, samples)
            for i, tag in enumerate(tags):
                if not tag:
                    continue
                tag = tag[0]
                if tag.conf < 0.85:
                    continue
                box = sample_regions[i]
                out_tag = me.dnn.Tag()
                out_tag.id = tag.id
                for c in range(4):
                    out_tag[c] = (tag[c].x * box.bbox.width + box.bbox.x,
                                  tag[c].y * box.bbox.height + box.bbox.y)
                true_frame = sample_frames[i] + f_num
                if true_frame not in all_tags:
                    all_tags[true_frame] = {}
                all_tags[true_frame][tag.id] = out_tag
            f_num += len(frames)
            percent_current = int(max(0, min(100 * ((f_num - first_frame) / (last_frame - first_frame)), 100)))
            event_queue.put(InfoEvent(f'Detecting tags: {percent_current}% (ESC to cancel)'))
        event_queue.put((TagFinishedEvent(all_tags, 'Tag detection task complete')))
        event_queue.put(InfoEvent('Done'))
    except MemoryError as e:
        event_queue.put(ErrorEvent("Out of memory", str(e)))
    except CancelledException as e:
        event_queue.put(CancelledEvent(str(e)))
    except LoadException as e:
        event_queue.put(ErrorEvent(str(e), e.detailed_message))
    except:
        event_queue.put(ErrorEvent(f'Error in tag detection task: {traceback.format_exc()}',
                                   traceback.format_exc()))


class DetectTagsOperator(EventOperator):
    """Scan playback range for tags"""
    bl_idname = "motionengine.detect_tags_operator"
    bl_label = "Detect Tags"

    def __init__(self):
        super().__init__()
        self.task_func = tag_task

    def execute(self, context):
        if self.tags is None:
            return {'FINISHED'}
        try:
            clip = bpy.data.movieclips[self.bpy_data.clip_info.bpy_name]
            clip_info = utils.ClipInfo(clip)
            clip_size = clip.size
            tracks = clip.tracking.tracks
            track_list = []
            tag_type = 'ML'
            if self.bpy_data.ui_props['tag_detector_type_enum'] == 'CV':
                tag_type = self.bpy_data.ui_props['tag_detector_cv_dict_list_enum']
            for frame in self.tags:
                actual_frame = clip_info.true_to_clip(frame)
                frame_tags = self.tags[frame]
                for tag_id in frame_tags:
                    tag = frame_tags[tag_id]
                    tag_track_name = f'Tag.{tag_type}.{tag.id}'
                    track = tracks.get(tag_track_name)
                    if tag_track_name not in track_list:
                        if track is None:
                            track = tracks.new(name=tag_track_name, frame=actual_frame)
                        else:
                            track_frames = [a for (a, _) in track.markers.items()]
                            for tf in track_frames:
                                track.markers.delete_frame(tf)
                            track = tracks.get(tag_track_name)
                            if track is None:
                                track = tracks.new(name=tag_track_name, frame=actual_frame)
                        track_list.append(tag_track_name)
                    center_x = 0
                    center_y = 0
                    corners = []
                    for c in range(4):
                        center_x += tag[c].x
                        center_y += tag[c].y
                        corners.append((tag[c].x / clip_size[0], (clip_size[1] - tag[c].y) / clip_size[1]))
                    center_x /= 4
                    center_y /= 4
                    norm_center_x = center_x / clip_size[0]
                    norm_center_y = (clip_size[1] - center_y) / clip_size[1]
                    corners = [(x - norm_center_x, y - norm_center_y) for (x, y) in corners]
                    corners.reverse()
                    markers = track.markers
                    marker = markers.insert_frame(actual_frame, co=(norm_center_x, norm_center_y))
                    marker.pattern_corners = tuple(corners)
                    marker.search_max *= 1.2
                    marker.search_min *= 1.2
                    marker.is_keyed = False
        except Exception:
            error = f'Error during evaluation: {traceback.format_exc()}'
            self.report({'ERROR'}, error)
        return {'FINISHED'}


CLASSES = [
    DetectTagsOperator
]
