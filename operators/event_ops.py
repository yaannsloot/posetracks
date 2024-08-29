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

import os.path
import traceback

import bpy
from .. import posetracks_core as pt_core
from .. import global_vars
from .. import events
from .. import utils
import queue

task = None
cancel_task = False
event_queue = queue.Queue()


def _str_to_enum(string, enum_class):
    names = list(enum_class.__members__)
    if string in names:
        e_idx = names.index(string)
        values = [int(e) for e in enum_class.__members__.values()]
        return enum_class(values[e_idx])


def _convert_if_enum(enum):
    if not isinstance(enum, str):
        return enum
    enum_classes = [pt_core.TagDictionary, pt_core.dnn.Executor, pt_core.dnn.FeatureDistanceType]
    for enum_class in enum_classes:
        converted = _str_to_enum(enum, enum_class)
        if converted is not None:
            return converted
    return enum


def setup_model(model_dict, executor):
    driver = utils.fix_class_ref(model_dict['driver'])()
    if isinstance(driver, pt_core.dnn.PoseModel):
        model_type = 'pose'
    elif isinstance(driver, pt_core.dnn.TagModel):
        model_type = 'tag'
    elif isinstance(driver, pt_core.dnn.FeatureModel):
        model_type = 'feature'
    else:
        model_type = 'detection'
    if not os.path.exists(model_dict['path']):
        event_queue.put(events.InfoEvent(f'Downloading {model_type} model...'))
        try:
            model_dict.fetch()
        except:
            raise LoadException(f'Failed to download {model_type} model',
                                f'An exception occurred while downloading {model_type} model: '
                                f'{traceback.format_exc()}')
    event_queue.put(events.InfoEvent(f'Loading {model_type} model...',
                                     f'Loading model {model_dict["path"]}'))
    try:
        driver.load(model_dict['path'], executor)
    except:
        raise LoadException(f'Failed to load {model_type} model',
                            f'An exception occurred while loading {model_type} model: '
                            f'{traceback.format_exc()}')
    if not driver.is_loaded():
        raise LoadException(f'Failed to load {model_type} model',
                            f'Could not load {model_type} model {model_dict["path"]}')
    return driver


def load_model_by_name(category, name, executor):
    models = pt_core.get_models(category)
    sel = None
    for m in models:
        if m[0] == name:
            sel = m[1]
            break
    if sel is None:
        raise LoadException('Could not find suitable model based on provided settings',
                            f'Model "{name}" not found')
    return setup_model(sel, executor), sel


def load_model_by_criteria(category, executor, attributes=None, values=None, sorting_criteria=None):
    models = pt_core.get_models(category, attributes, values, sorting_criteria)
    if not models:
        raise LoadException('Could not find suitable model based on provided settings',
                            'Model search returned no results')
    sel = models[0][1]
    return setup_model(sel, executor), sel


def setup_frame_provider(path, source_type):
    if source_type == 'MOVIE':
        fp = pt_core.io.Transcoder()
    else:
        fp = pt_core.io.ImageList()
    fp.load(path)
    if not fp.is_open():
        raise LoadException('Failed to load movie clip',
                            f'Could not load media from movie clip source {path}')
    return fp


class LoadException(Exception):
    def __init__(self, message, detailed_message):
        super().__init__(message)
        self.detailed_message = detailed_message


class CancelledException(Exception):
    def __init__(self, message):
        super().__init__(message)


def batch_infer(batch_size, model, images, *args):
    while True:
        try:
            output = pt_core.dnn.strict_batch_infer(batch_size, model, images, *args)
            break
        except RuntimeError as e:
            tb = traceback.format_exc()
            if 'Available memory of' not in tb or 'is smaller than requested bytes of' not in tb:
                raise e
            if batch_size <= 1:
                raise MemoryError('Memory exhausted. Try using a different model or target device.')
            batch_size //= 2
            print(f'Exhausted memory during forward pass. Batch size reduced to {batch_size}.')
    return output, batch_size


def get_frames(frame_provider, num_frames, last_frame):
    result = []
    for i in range(0, num_frames):
        f = pt_core.Mat()
        f_num = frame_provider.current_frame()
        if f_num > last_frame:
            break
        s = frame_provider.next_frame(f)
        if not s:
            break
        result.append(f)
    return result


class BpyData:
    def __init__(self, include_tracking_data=False, selected_tracks_only=True, filter_joints_and_tags=True):
        context = bpy.context
        scene = bpy.context.scene

        self.scene_first_frame = scene.frame_start
        self.scene_last_frame = scene.frame_end

        properties = scene.pt_ui_properties

        self.clip_info = None
        if (context.area.type == 'CLIP_EDITOR' and
                context.edit_movieclip is not None):
            self.clip_info = utils.ClipInfo(context.edit_movieclip)

        # Property group capture from ui_props.py
        self.ui_props = {}
        for key in properties.__annotations__.keys():
            val = getattr(properties, key)
            if isinstance(val, (int, float, bool, str)):
                self.ui_props[key] = _convert_if_enum(val)

        # Anchor cam data
        self.anchor_clip_info = properties.anchor_cam_selection
        if self.anchor_clip_info is not None:
            self.anchor_clip_info = utils.ClipInfo(self.anchor_clip_info)

        self.clip_tracks = {}
        self.anchor_tracks = {}
        if include_tracking_data:
            # Bounding box conversions
            if self.clip_info is not None:
                # function returns data in scene time
                self.clip_tracks = pt_core.clip_tracking_data(context.edit_movieclip, filter_selected=selected_tracks_only)
            if self.anchor_clip_info is not None:
                self.anchor_tracks = pt_core.clip_tracking_data(properties.anchor_cam_selection,
                                                           filter_selected=selected_tracks_only)
            # Additional info
            """
            self.clip_tracks_max_f = max([list(data.keys())[-1] for data in self.clip_tracks.values()], default=0)
            self.clip_tracks_min_f = min([list(data.keys())[0] for data in self.clip_tracks.values()], default=0)
            self.anchor_tracks_max_f = max([list(data.keys())[-1] for data in self.anchor_tracks.values()], default=0)
            self.anchor_tracks_min_f = min([list(data.keys())[0] for data in self.anchor_tracks.values()], default=0)
            """


def do_nothing(bpy_data):
    print("Nothing done!")
    pass


class EventOperator(bpy.types.Operator):
    bl_options = {'REGISTER', 'UNDO'}

    def __init__(self):
        super().__init__()
        self._dispatch = events.EventDispatcher()
        self._base_listener = events.EventListener()
        self._info_listener = events.InfoEventListener()
        self._error_listener = events.ErrorEventListener()
        self._cancelled_listener = events.CancelledEventListener()
        self._detection_listener = events.DetectionFinishedEventListener()
        self._tag_listener = events.TagFinishedEventListener()
        self._pose_listener = events.PoseFinishedEventListener()
        self._dispatch.register_listener(self._base_listener)
        self._dispatch.register_listener(self._info_listener)
        self._dispatch.register_listener(self._error_listener)
        self._dispatch.register_listener(self._cancelled_listener)
        self._dispatch.register_listener(self._detection_listener)
        self._dispatch.register_listener(self._tag_listener)
        self._dispatch.register_listener(self._pose_listener)
        self._base_listener.notify_response = self.base_response
        self._info_listener.notify_response = self.info_response
        self._error_listener.notify_response = self.error_response
        self._cancelled_listener.notify_response = self.cancelled_response
        self._detection_listener.notify_response = self.detection_finished_response
        self._tag_listener.notify_response = self.tag_detection_finished_response
        self._pose_listener.notify_response = self.pose_estimation_finished_response
        self._timer = None
        self.detections = None
        self.tags = None
        self.poses = None
        self.bpy_data = None
        self.include_tracking_data = False
        self.selected_tracks_only = True
        self.filter_joints_and_tags = True
        self.task_func = do_nothing

    @classmethod
    def poll(cls, context):
        return not global_vars.ui_lock_state

    def invoke(self, context, event):
        global cancel_task, task
        if task is not None and task.running():
            cancel_task = True
            task.result()
        cancel_task = False
        self.digest_events(True)
        wm = context.window_manager
        self._timer = wm.event_timer_add(0.3, window=context.window)
        wm.modal_handler_add(self)
        self.detections = None
        self.tags = None
        self.poses = None
        self.bpy_data = BpyData(self.include_tracking_data, self.selected_tracks_only, self.filter_joints_and_tags)
        task = global_vars.executor.submit(self.task_func, self.bpy_data)
        global_vars.ui_lock_state = True
        return {'RUNNING_MODAL'}

    def modal(self, context, event):
        global cancel_task
        if event.type in {'ESC'}:
            cancel_task = True
            task.result()
            self.digest_events()
            self.cancel(context)
            return {'CANCELLED'}
        if event.type == 'TIMER':
            self.digest_events()
            if task.done():
                self.cancel(context)
                return self.execute(context)
        return {'RUNNING_MODAL'}

    def cancel(self, context):
        wm = context.window_manager
        wm.event_timer_remove(self._timer)
        global_vars.ui_lock_state = False
        utils.force_ui_draw()

    def digest_events(self, no_response=False):
        while True:
            try:
                e = event_queue.get_nowait()
                if not no_response:
                    self._dispatch.broadcast(e)
            except queue.Empty:
                break

    def base_response(self, event):
        if event.msg != '':
            print(event.msg)

    def info_response(self, event):
        self.base_response(event)
        if event.info_msg != '':
            self.report({'INFO'}, event.info_msg)

    def error_response(self, event):
        self.base_response(event)
        if event.error_msg != '':
            self.report({'ERROR'}, event.error_msg)

    def cancelled_response(self, event):
        self.base_response(event)
        if event.cancel_msg != '':
            self.report({'INFO'}, event.cancel_msg)

    def detection_finished_response(self, event):
        self.base_response(event)
        self.detections = event.detections

    def tag_detection_finished_response(self, event):
        self.base_response(event)
        self.tags = event.tags

    def pose_estimation_finished_response(self, event):
        self.base_response(event)
        self.poses = event.poses
