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
from .. import events
from .. import utils
import queue
import traceback

"""
NOTE TO SELF
THIS DOESN'T SUPPORT ML TAG DETECTORS (YET)
PLEASE FIX WHEN MODELS ARE AVAILABLE
UPDATE: It does now. Kinda :)
Does not actually use the model selection system. 
Fix when more are available and performance data is collected. 
"""

task = None
cancel_task = False
event_queue = queue.Queue()
det_model = me.dnn.DetectionModel()
tag_model = me.dnn.FeatureModel()


class TaskSettings:
    def __init__(self, clip_info: utils.ClipInfo, det_model_path, det_driver, tag_model_path='',
                 tag_driver=me.dnn.CVTagDetector,
                 det_exec=me.dnn.CUDA,
                 tag_exec=me.dnn.CUDA,
                 det_conf=0.5,
                 det_iou=0.5,
                 target_cid=0,
                 cv_dict=me.DICT_4X4_50,
                 cv_resample=True,
                 batch_size=32):
        self.clip_info = clip_info
        self.det_model_path = det_model_path
        self.tag_model_path = tag_model_path
        if isinstance(det_driver, tuple):
            self.det_driver = det_driver[0]
        else:
            self.det_driver = det_driver
        if isinstance(tag_driver, tuple):
            self.tag_driver = tag_driver[0]
        else:
            self.tag_driver = tag_driver
        self.det_exec = det_exec
        self.tag_exec = tag_exec
        self.det_conf = det_conf
        self.det_iou = det_iou
        self.cv_dict = cv_dict
        self.cv_resample = cv_resample
        self.target_cid = target_cid
        self.batch_size = batch_size


def get_frames(frame_provider: me.io.FrameProvider, num_frames, last_frame):
    result = []
    for i in range(0, num_frames):
        f = me.Mat()
        f_num = frame_provider.current_frame()
        if f_num > last_frame:
            break
        s = frame_provider.next_frame(f)
        if not s:
            break
        result.append(f)
    return result


def tag_task(task_settings: TaskSettings):
    global cancel_task
    global det_model
    global tag_model
    clip_path = task_settings.clip_info.abs_path
    clip_type = task_settings.clip_info.source_type
    batch_size = task_settings.batch_size
    first_frame = task_settings.clip_info.scene_to_true(task_settings.clip_info.scene_first_frame)
    last_frame = task_settings.clip_info.scene_to_true(task_settings.clip_info.scene_last_frame)
    first_frame = max(first_frame, 0)
    try:
        tag_model.unload()
        det_model.unload()
        tag_model = task_settings.tag_driver()
        if isinstance(tag_model, me.dnn.CVTagDetector):
            tag_model.set_dict_type(task_settings.cv_dict)
            if task_settings.cv_resample:
                tag_model.set_preprocess_size((224, 224))
        else:
            event_queue.put(events.InfoEvent('Loading tag model...',
                                             f'Loading model {task_settings.tag_model_path}'))
            tag_model.load(task_settings.tag_model_path, task_settings.tag_exec)
            if not tag_model.is_loaded():
                event_queue.put(events.ErrorEvent('Failed to load tag model',
                                                  f'Could not load tag model {task_settings.tag_model_path}'))
                return
        det_model = task_settings.det_driver()
        event_queue.put(events.InfoEvent('Loading detection model...',
                                         f'Loading model {task_settings.det_model_path}'))
        det_model.load(task_settings.det_model_path, task_settings.det_exec)
        if not det_model.is_loaded():
            event_queue.put(events.ErrorEvent('Failed to load detection model',
                                              f'Could not load detection model {task_settings.det_model_path}'))
            return

        if clip_type == 'MOVIE':
            clip = me.io.Transcoder()
        else:
            clip = me.io.ImageList()
        clip.load(clip_path)
        if not clip.is_open():
            event_queue.put(events.ErrorEvent('Failed to load movie clip',
                                              f'Could not load media from movie clip source {clip_path}'))
            return
        last_frame = min(last_frame, clip.frame_count())
        clip.set_frame(first_frame)
        f_num = first_frame
        final_tags = {}
        while True:

            if cancel_task or global_vars.shutdown_state:
                event_queue.put(events.CancelledEvent('Tag detection cancelled'))
                return
            frames = get_frames(clip, batch_size, last_frame)
            if not frames:
                break

            while True:
                try:
                    detections = me.dnn.strict_batch_infer(batch_size, det_model, frames,
                                                           task_settings.det_conf, task_settings.det_iou)
                    break
                except RuntimeError as re:
                    error_tb = traceback.format_exc()
                    if 'Available memory of' not in error_tb or 'is smaller than requested bytes of' not in error_tb:
                        raise re
                    if batch_size <= 1:
                        raise RuntimeError('Failed to reduce batch size to a safe value. '
                                           'Try using a different model or target device.')
                    batch_size //= 2
                    event_queue.put(events.InfoEvent(msg=f'Exhausted memory when executing forward pass. '
                                                         f'Batch size reduced to {batch_size}.'))

            detections = me.dnn.fix_detection_coordinates(detections, det_model.net_size(), clip.frame_size(),
                                                          me.dnn.AUTO)

            old_detections = detections
            detections = []
            det_frames = []
            for f_n in range(len(old_detections)):
                det_frame = old_detections[f_n]
                for det in det_frame:
                    det.scale_detection(1.2)
                    if (not me.dnn.is_roi_outside_image(clip.frame_size(), det.bbox)
                            and det.class_id == task_settings.target_cid):
                        detections.append(det)
                        det_frames.append(f_n)

            tags = []

            for i in range(0, len(detections), batch_size):
                det_chunk = detections[i: i + batch_size]
                frames_chunk = det_frames[i: i + batch_size]
                chunk_samples = []
                for j in range(len(det_chunk)):
                    roi_det = det_chunk[j]
                    target_frame = frames_chunk[j]
                    chunk_samples.append(me.dnn.get_roi_no_padding(frames[target_frame], roi_det.bbox))

                # MUST do this to avoid crashes
                if isinstance(tag_model, me.dnn.CVTagDetector):
                    tags.extend(tag_model.infer(chunk_samples))
                else:
                    tags.extend(me.dnn.strict_batch_infer(batch_size, tag_model, chunk_samples))

            for i in range(len(detections)):
                if len(tags[i]) == 0:
                    continue
                tag = tags[i][0]
                if tag.conf < 0.85:
                    continue
                box = detections[i]
                out_tag = me.dnn.Tag()
                out_tag.id = tag.id
                for c in range(4):
                    out_tag[c] = (tag[c].x * box.bbox.width + box.bbox.x,
                                  tag[c].y * box.bbox.height + box.bbox.y)
                if det_frames[i] + f_num not in final_tags:
                    final_tags[det_frames[i] + f_num] = {}
                final_tags[det_frames[i] + f_num][tag.id] = out_tag

            f_num += len(frames)
            percent_current = int(max(0, min(100 * ((f_num - first_frame) / (last_frame - first_frame)), 100)))
            event_queue.put(events.InfoEvent(f'Detecting tags: {percent_current}% (ESC to cancel)'))
        event_queue.put((events.TagFinishedEvent(final_tags, 'Tag detection task complete')))

    except Exception:
        event_queue.put(events.ErrorEvent(f'Error in detection task: {traceback.format_exc()}',
                                          traceback.format_exc()))


class DetectTagsOperator(bpy.types.Operator):
    """Scan playback range for tags"""
    bl_idname = "motionengine.detect_tags_operator"
    bl_label = "Detect Tags"
    bl_options = {'REGISTER', 'UNDO'}
    _timer = None
    dispatch = events.EventDispatcher()
    info_listener = events.InfoEventListener()
    tag_finished_listener = events.TagFinishedEventListener()
    cancelled_listener = events.CancelledEventListener()
    error_listener = events.ErrorEventListener()
    queued_clip = None
    last_tag_type = me.DICT_4X4_50.name

    def __init__(self):
        super().__init__()
        self.dispatch.register_listener(self.info_listener)
        self.dispatch.register_listener(self.tag_finished_listener)
        self.dispatch.register_listener(self.cancelled_listener)
        self.dispatch.register_listener(self.error_listener)
        self.info_listener.notify_response = self.info_response
        self.tag_finished_listener.notify_response = self.task_finished_response
        self.cancelled_listener.notify_response = self.cancelled_response
        self.error_listener.notify_response = self.error_response

    def execute(self, context):
        global cancel_task, task

        if global_vars.ui_lock_state:
            return {'FINISHED'}

        if task is not None and task.running():
            cancel_task = True
            task.result()

        scene = context.scene
        properties = scene.motion_engine_ui_properties

        if properties.me_ui_prop_det_tag_simple_sel_enum == 'FAST':
            det_models = me.get_models('object_detection',
                                       values={'classes': ['Tag']},
                                       sorting_criteria=['gflops'])
        elif properties.me_ui_prop_det_tag_simple_sel_enum == 'BALANCED':
            det_models = me.get_models('object_detection',
                                       values={'classes': ['Tag']},
                                       sorting_criteria=['precision_ap', 'gflops'])
        elif properties.me_ui_prop_det_tag_simple_sel_enum == 'BAL_MEM':
            det_models = me.get_models('object_detection',
                                       values={'classes': ['Tag']},
                                       sorting_criteria=['precision_ap', 'gflops', 'mparams'])
        else:
            det_models = me.get_models('object_detection',
                                       values={'classes': ['Tag']},
                                       sorting_criteria=['precision_ap'])

        if len(det_models) == 0:
            self.report({'ERROR'}, 'Could not find suitable model based on provided settings')
            return {'FINISHED'}

        if properties.me_ui_prop_exe_det_tag_enum == 'CPU':
            det_exec = me.dnn.CPU
        else:
            det_exec = me.dnn.CUDA

        if properties.me_ui_prop_exe_tag_detector_ml_enum == 'CPU':
            tag_exec = me.dnn.CPU
        else:
            tag_exec = me.dnn.CUDA

        det_model_sel = det_models[0][1]
        det_model_path = det_model_sel['path']
        det_model_driver = det_model_sel['driver']
        det_model_classes = det_model_sel['classes']

        try:
            target_cid = det_model_classes.index(str(properties.me_ui_prop_det_class_enum))
        except ValueError:
            target_cid = 0

        selected_dict = properties.me_ui_tag_detector_cv_dict_list_enum

        tag_dictionary_members = [e for e in me.TagDictionary.__members__]

        try:
            dict_id = tag_dictionary_members.index(selected_dict)
            target_dict = me.TagDictionary(dict_id)
        except ValueError:
            target_dict = me.DICT_4X4_50

        if properties.me_ui_prop_tag_detector_type_enum == 'ML':
            tag_model_path = me.model_path('TagNet/basic_reg_v1.onnx')
            tag_model_driver = me.dnn.TagNetModel
            self.last_tag_type = 'ML'
        else:
            tag_model_path = ''
            tag_model_driver = me.dnn.CVTagDetector
            self.last_tag_type = target_dict.name

        current_clip = context.edit_movieclip

        task_settings = TaskSettings(
            utils.ClipInfo(current_clip),
            det_model_path,
            det_model_driver,
            tag_model_path,
            tag_model_driver,
            det_exec,
            tag_exec,
            properties.me_ui_prop_det_tag_conf,
            properties.me_ui_prop_det_tag_iou,
            target_cid,
            target_dict,
            properties.me_ui_tag_detector_cv_resample_toggle_prop
        )

        cancel_task = False

        self.digest_events(context, True)

        wm = context.window_manager
        self._timer = wm.event_timer_add(0.3, window=context.window)
        wm.modal_handler_add(self)

        task = global_vars.executor.submit(tag_task, task_settings)

        self.queued_clip = current_clip

        global_vars.ui_lock_state = True

        return {'RUNNING_MODAL'}

    def modal(self, context, event):
        global cancel_task
        if event.type in {'ESC'}:
            cancel_task = True
            task.result()
            self.digest_events(context)
            self.cancel(context)
            return {'CANCELLED'}
        if event.type == 'TIMER':
            self.digest_events(context)
            if task.done():
                self.cancel(context)
                return {'FINISHED'}
        return {'PASS_THROUGH'}

    def cancel(self, context):
        global det_model
        global tag_model
        wm = context.window_manager
        wm.event_timer_remove(self._timer)
        global_vars.ui_lock_state = False
        utils.force_ui_draw()
        det_model.unload()
        tag_model.unload()
        det_model = me.dnn.DetectionModel()
        tag_model = me.dnn.FeatureModel()

    def digest_events(self, context, no_response=False):
        while True:
            try:
                e = event_queue.get_nowait()
                if not no_response:
                    self.dispatch.broadcast(e, context)
            except queue.Empty:
                break

    def info_response(self, event, context):
        if event.info_msg != '':
            self.report({'INFO'}, event.info_msg)
        if event.msg != '':
            print(event.msg)

    def task_finished_response(self, event, context):
        try:
            if self.queued_clip is None:
                return
            self.report({'INFO'}, 'Done.')
            clip: bpy.types.MovieClip = self.queued_clip
            clip_info = utils.ClipInfo(clip)
            clip_size = clip.size
            tracks = clip.tracking.tracks
            track_list = []
            for frame in event.tags:
                actual_frame = clip_info.true_to_clip(frame)
                frame_tags = event.tags[frame]
                for tag_id in frame_tags:
                    tag = frame_tags[tag_id]
                    tag_track_name = f'Tag.{self.last_tag_type}.{tag.id}'
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

    def cancelled_response(self, event, context):
        if event.cancel_msg != '':
            self.report({'INFO'}, event.cancel_msg)

    def error_response(self, event, context):
        if event.error_msg != '':
            self.report({'ERROR'}, event.error_msg)
        if event.msg != '':
            print(event.msg)


CLASSES = [
    DetectTagsOperator
]
