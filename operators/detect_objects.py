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

task = None
cancel_task = False
event_queue = queue.Queue()
det_model = me.dnn.DetectionModel()
feat_model = me.dnn.FeatureModel()


class TaskSettings:
    def __init__(self, clip_info: utils.ClipInfo, det_model_path, feat_model_path, det_driver, feat_driver,
                 det_exec=me.dnn.CUDA,
                 feat_exec=me.dnn.CUDA,
                 det_conf=0.5,
                 det_iou=0.5,
                 target_cid=0,
                 feat_dist=1.4,
                 feat_reid=1.0,
                 dist_type=me.dnn.NORM_EUCLIDEAN,
                 batch_size=32):
        self.clip_info = clip_info
        self.det_model_path = det_model_path
        self.feat_model_path = feat_model_path
        if isinstance(det_driver, tuple):
            self.det_driver = det_driver[0]
        else:
            self.det_driver = det_driver
        if isinstance(feat_driver, tuple):
            self.feat_driver = feat_driver[0]
        else:
            self.feat_driver = feat_driver
        self.det_exec = det_exec
        self.feat_exec = feat_exec
        self.det_conf = det_conf
        self.det_iou = det_iou
        self.feat_dist = feat_dist
        self.feat_reid = feat_reid
        self.dist_type = dist_type
        self.target_cid = target_cid
        self.batch_size = batch_size


def get_frames(frame_provider: me.io.FrameProvider, num_frames):
    result = []
    for i in range(0, num_frames):
        f = me.Mat()
        s = frame_provider.next_frame(f)
        if not s:
            break
        result.append(f)
    return result


def detection_task(task_settings: TaskSettings):
    global cancel_task
    global det_model
    global feat_model
    clip_path = task_settings.clip_info.abs_path
    clip_type = task_settings.clip_info.source_type
    batch_size = task_settings.batch_size
    try:
        feat_model.unload()
        det_model.unload()
        feat_model = task_settings.feat_driver()
        event_queue.put(events.InfoEvent('Loading feature model...',
                                         f'Loading model {task_settings.feat_model_path}'))
        feat_model.load(task_settings.feat_model_path, task_settings.feat_exec)
        if not feat_model.is_loaded():
            event_queue.put(events.ErrorEvent('Failed to load feature model',
                                              f'Could not load feature model {task_settings.feat_model_path}'))
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
        event_queue.put(events.InfoEvent('Initializing tracker...',
                                         'Obtaining feature length from model...'))
        f_type = feat_model.infer(me.rand_img_rgb(feat_model.net_size()))
        f_tracker = me.dnn.FeatureTracker(len(f_type))
        f_num = 0
        final_detections = {}
        while True:

            if cancel_task or global_vars.shutdown_state:
                event_queue.put(events.CancelledEvent('Object detection cancelled'))
                return
            frames = get_frames(clip, batch_size)
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
            for i in range(len(detections)):
                frame_dets = []
                frame_feats = []
                for j in range(len(detections[i])):
                    if (not me.dnn.is_roi_outside_image(clip.frame_size(), detections[i][j].bbox)
                            and detections[i][j].class_id == task_settings.target_cid):
                        frame_feats.append(
                            feat_model.infer(me.dnn.get_roi_with_padding(frames[i], detections[i][j].bbox)))
                        frame_dets.append(detections[i][j])
                actual_frame = i + f_num
                det_ids = f_tracker.assign(frame_dets, frame_feats, task_settings.feat_dist, task_settings.feat_reid,
                                           task_settings.dist_type)
                num_assignments = len(det_ids)
                if num_assignments == 0:
                    continue
                if actual_frame not in final_detections:
                    final_detections[actual_frame] = {}
                for j in range(num_assignments):
                    final_detections[actual_frame][det_ids[j]] = frame_dets[j]

            f_num += len(frames)
            percent_current = int(max(0.0, min(100 * (f_num / clip.frame_count()), 100.0)))
            event_queue.put(events.InfoEvent(f'Detecting objects: {percent_current}% (ESC to cancel)'))
        event_queue.put(events.DetectionFinishedEvent(final_detections, 'Detection task completed.'))

    except Exception:
        event_queue.put(events.ErrorEvent(f'Error in detection task: {traceback.format_exc()}',
                                          traceback.format_exc()))


def track_name(det_id, class_id):
    return f'{class_id.capitalize()} {det_id}'


def get_track_name_for_detection(clip: bpy.types.MovieClip, class_id):
    det_id = 0
    while True:
        name = track_name(det_id, class_id)
        if name not in clip.tracking.tracks:
            return name
        det_id += 1


class DetectObjectsOperator(bpy.types.Operator):
    """Scan entire clip for target objects"""
    bl_idname = "motionengine.detect_objects_operator"
    bl_label = "Detect Objects"
    bl_options = {'REGISTER', 'UNDO'}
    _timer = None
    dispatch = events.EventDispatcher()
    info_listener = events.InfoEventListener()
    detect_finished_listener = events.DetectionFinishedEventListener()
    cancelled_listener = events.CancelledEventListener()
    error_listener = events.ErrorEventListener()
    queued_clip = None

    def __init__(self):
        super().__init__()
        self.dispatch.register_listener(self.info_listener)
        self.dispatch.register_listener(self.detect_finished_listener)
        self.dispatch.register_listener(self.cancelled_listener)
        self.dispatch.register_listener(self.error_listener)
        self.info_listener.notify_response = self.info_response
        self.detect_finished_listener.notify_response = self.task_finished_response
        self.cancelled_listener.notify_response = self.cancelled_response
        self.error_listener.notify_response = self.error_response

    @classmethod
    def poll(cls, context):
        return not global_vars.ui_lock_state

    def execute(self, context):
        global cancel_task, task

        if task is not None and task.running():
            cancel_task = True
            task.result()

        scene = context.scene
        properties = scene.motion_engine_ui_properties

        if properties.me_ui_prop_det_simple_sel_enum == 'FAST':
            det_models = me.get_models('object_detection',
                                       values={'classes': [str(properties.me_ui_prop_det_class_enum)]},
                                       sorting_criteria=['gflops'])
        elif properties.me_ui_prop_det_simple_sel_enum == 'BALANCED':
            det_models = me.get_models('object_detection',
                                       values={'classes': [str(properties.me_ui_prop_det_class_enum)]},
                                       sorting_criteria=['precision_ap', 'gflops'])
        elif properties.me_ui_prop_det_simple_sel_enum == 'BAL_MEM':
            det_models = me.get_models('object_detection',
                                       values={'classes': [str(properties.me_ui_prop_det_class_enum)]},
                                       sorting_criteria=['precision_ap', 'gflops', 'mparams'])
        else:
            det_models = me.get_models('object_detection',
                                       values={'classes': [str(properties.me_ui_prop_det_class_enum)]},
                                       sorting_criteria=['precision_ap'])

        if len(det_models) == 0:
            self.report({'ERROR'}, 'Could not find suitable model based on provided settings')
            return {'FINISHED'}

        if properties.me_ui_prop_exe_det_enum == 'CPU':
            det_exec = me.dnn.CPU
        else:
            det_exec = me.dnn.CUDA

        if properties.me_ui_prop_exe_track_enum == 'CPU':
            feat_exec = me.dnn.CPU
        else:
            feat_exec = me.dnn.CUDA

        if properties.me_ui_prop_track_dist_type == 'EUCLIDEAN':
            dist_type = me.dnn.EUCLIDEAN
        else:
            dist_type = me.dnn.NORM_EUCLIDEAN

        det_model_sel = det_models[0][1]
        det_model_path = det_model_sel['path']
        det_model_driver = det_model_sel['driver']
        det_model_classes = det_model_sel['classes']

        try:
            target_cid = det_model_classes.index(str(properties.me_ui_prop_det_class_enum))
        except ValueError:
            target_cid = 0

        current_clip = context.edit_movieclip

        task_settings = TaskSettings(
            utils.ClipInfo(current_clip),
            det_model_path,
            me.model_path('Features/basic_conv_person_64.onnx'),
            det_model_driver,
            me.dnn.GenericFeatureModel,
            det_exec,
            feat_exec,
            properties.me_ui_prop_det_conf,
            properties.me_ui_prop_det_iou,
            target_cid,
            properties.me_ui_prop_track_score,
            properties.me_ui_prop_track_reid_score,
            dist_type
        )

        cancel_task = False

        self.digest_events(context, True)

        wm = context.window_manager
        self._timer = wm.event_timer_add(0.3, window=context.window)
        wm.modal_handler_add(self)

        task = global_vars.executor.submit(detection_task, task_settings)

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
        global feat_model
        wm = context.window_manager
        wm.event_timer_remove(self._timer)
        global_vars.ui_lock_state = False
        utils.force_ui_draw()
        det_model.unload()
        feat_model.unload()
        det_model = me.dnn.DetectionModel()
        feat_model = me.dnn.FeatureModel()

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
            scene = context.scene
            properties = scene.motion_engine_ui_properties
            class_id = str(properties.me_ui_prop_det_class_enum)
            clip: bpy.types.MovieClip = self.queued_clip
            clip_info = utils.ClipInfo(clip)
            clip_size = clip.size
            tracks = clip.tracking.tracks
            if event.msg != '':
                print(event.msg)
            init_dict = {}
            for frame in event.detections:
                actual_frame = clip_info.true_to_clip(frame)
                frame_detections = event.detections[frame]
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
                    marker.mute = True
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
    DetectObjectsOperator
]
