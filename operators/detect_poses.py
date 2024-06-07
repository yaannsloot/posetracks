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

import math

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
pose_model = me.dnn.PoseModel()


class TaskSettings:
    def __init__(self, clip_info: utils.ClipTrackingData, pose_model_path, pose_driver, pose_exec=me.dnn.CUDA,
                 batch_size=32):
        self.clip_info = clip_info
        self.pose_model_path = pose_model_path
        if isinstance(pose_driver, tuple):
            self.pose_driver = pose_driver[0]
        else:
            self.pose_driver = pose_driver
        self.pose_exec = pose_exec
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


def adjust_det_aspect(det: me.dnn.Detection, net_size: tuple[int, int]):
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
    det.bbox = me.Rect(new_tl[0], new_tl[1], width, height)


def pose_task(task_settings: TaskSettings):
    global cancel_task
    global pose_model
    clip_path = task_settings.clip_info.abs_path
    clip_type = task_settings.clip_info.source_type
    batch_size = task_settings.batch_size
    try:
        pose_model.unload()
        pose_model = task_settings.pose_driver()
        event_queue.put(events.InfoEvent('Loading pose model...',
                                         f'Loading model {task_settings.pose_model_path}'))
        pose_model.load(task_settings.pose_model_path, task_settings.pose_exec)
        if not pose_model.is_loaded():
            event_queue.put(events.ErrorEvent('Failed to load pose model',
                                              f'Could not load pose model {task_settings.pose_model_path}'))
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

        target_detections = {}
        target_frames = set()

        for frame in task_settings.clip_info.track_data:
            for track in task_settings.clip_info.track_data[frame]:
                if track not in task_settings.clip_info.selected_tracks:
                    continue
                true_frame = task_settings.clip_info.clip_to_true(frame)
                if true_frame not in target_detections:
                    target_detections[true_frame] = {}
                target_detections[true_frame][track] = task_settings.clip_info.track_data[frame][track]
                target_frames.add(true_frame)

        if len(target_frames) == 0:
            error_msg = 'Task cancelled. No valid tracking data available.'
            event_queue.put(events.ErrorEvent(error_msg, error_msg))
            return

        target_frames = list(target_frames)
        target_frames.sort()

        first_frame = max(target_frames[0], 0)
        last_frame = min(target_frames[-1], clip.frame_count())

        clip.set_frame(first_frame)
        clip.current_frame()

        f_num = first_frame
        final_poses = {}
        while True:

            if cancel_task or global_vars.shutdown_state:
                event_queue.put(events.CancelledEvent('Pose estimation cancelled'))
                return
            frames = get_frames(clip, batch_size, last_frame)
            if not frames:
                break

            samples = []
            sample_boxes = []
            sample_ids = []
            sample_frames = []
            for i in range(len(frames)):
                true_frame = f_num + i
                if true_frame not in target_frames:
                    continue
                frame_data = target_detections[true_frame]
                for track in frame_data:
                    det = frame_data[track]
                    if me.dnn.is_roi_outside_image(clip.frame_size(), det.bbox):
                        continue
                    adjust_det_aspect(det, pose_model.net_size())
                    det.scale_detection(1.2)
                    sample = me.dnn.get_roi_with_padding(frames[i], det.bbox)
                    samples.append(sample)
                    sample_boxes.append(det)
                    sample_ids.append(track)
                    sample_frames.append(true_frame)

            while True:
                try:
                    poses = me.dnn.strict_batch_infer(batch_size, pose_model, samples)
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

            for i in range(len(samples)):
                clip_frame = task_settings.clip_info.true_to_clip(sample_frames[i])
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
                if clip_frame not in final_poses:
                    final_poses[clip_frame] = {}
                final_poses[clip_frame][pose_id] = pose

            f_num += len(frames)
            percent_current = int(max(0, min(100 * ((f_num - first_frame) / (last_frame - first_frame)), 100)))
            event_queue.put(events.InfoEvent(f'Estimating poses: {percent_current}% (ESC to cancel)'))
        event_queue.put(events.PoseFinishedEvent(final_poses, 'Pose estimation task complete'))

    except Exception:
        event_queue.put(events.ErrorEvent(f'Error in detection task: {traceback.format_exc()}',
                                          traceback.format_exc()))
    pass


class DetectPosesOperator(bpy.types.Operator):
    """Convert tracks to poses"""
    bl_idname = "motionengine.detect_poses_operator"
    bl_label = "Estimate Poses"
    bl_options = {'REGISTER', 'UNDO'}
    _timer = None
    dispatch = events.EventDispatcher()
    info_listener = events.InfoEventListener()
    pose_finished_event_listener = events.PoseFinishedEventListener()
    cancelled_listener = events.CancelledEventListener()
    error_listener = events.ErrorEventListener()
    queued_clip = None
    last_pose_source = 'person'
    last_pose_keypoints = '17'

    def __init__(self):
        super().__init__()
        self.dispatch.register_listener(self.info_listener)
        self.dispatch.register_listener(self.pose_finished_event_listener)
        self.dispatch.register_listener(self.cancelled_listener)
        self.dispatch.register_listener(self.error_listener)
        self.info_listener.notify_response = self.info_response
        self.pose_finished_event_listener.notify_response = self.task_finished_response
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

        target_class = properties.pose_target_enum
        keypoints = int(properties.pose_keypoints_enum)
        self.last_pose_keypoints = properties.pose_keypoints_enum
        self.last_pose_source = target_class

        if properties.pose_model_sel_enum == 'FAST':
            pose_models = me.get_models('pose_estimation',
                                        values={'target_class': target_class,
                                                'keypoints': keypoints},
                                        sorting_criteria=['gflops'])
        elif properties.pose_model_sel_enum == 'BALANCED':
            pose_models = me.get_models('pose_estimation',
                                        values={'target_class': target_class,
                                                'keypoints': keypoints},
                                        sorting_criteria=['precision_ap', 'gflops'])
        elif properties.pose_model_sel_enum == 'BAL_MEM':
            pose_models = me.get_models('pose_estimation',
                                        values={'target_class': target_class,
                                                'keypoints': keypoints},
                                        sorting_criteria=['precision_ap', 'gflops', 'mparams'])
        else:
            pose_models = me.get_models('pose_estimation',
                                        values={'target_class': target_class,
                                                'keypoints': keypoints},
                                        sorting_criteria=['precision_ap'])

        if len(pose_models) == 0:
            self.report({'ERROR'}, 'Could not find suitable model based on provided settings')
            return {'FINISHED'}

        if properties.exe_pose_enum == 'CPU':
            pose_exec = me.dnn.CPU
        else:
            pose_exec = me.dnn.CUDA

        pose_model_sel = pose_models[0][1]
        pose_model_path = pose_model_sel['path']
        pose_model_driver = pose_model_sel['driver']

        current_clip = context.edit_movieclip

        task_settings = TaskSettings(
            utils.ClipTrackingData(current_clip),
            pose_model_path,
            pose_model_driver,
            pose_exec
        )

        cancel_task = False

        self.digest_events(context, True)

        wm = context.window_manager
        self._timer = wm.event_timer_add(0.3, window=context.window)
        wm.modal_handler_add(self)

        task = global_vars.executor.submit(pose_task, task_settings)

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
        global pose_model
        wm = context.window_manager
        wm.event_timer_remove(self._timer)
        global_vars.ui_lock_state = False
        utils.force_ui_draw()
        pose_model.unload()
        pose_model = me.dnn.PoseModel()

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
            if event.msg != '':
                print(event.msg)
            if self.queued_clip is None:
                return
            self.report({'INFO'}, 'Done.')
            scene = context.scene
            properties = scene.motion_engine_ui_properties
            overwrite = properties.overwrite_poses
            conf_thresh = properties.joint_conf
            clip: bpy.types.MovieClip = self.queued_clip
            clip_size = clip.size
            tracks = clip.tracking.tracks
            track_list = {}
            for frame in event.poses:
                frame_poses = event.poses[frame]
                for pose_id in frame_poses:
                    pose: me.dnn.Pose = frame_poses[pose_id]
                    for j in range(pose.num_joints()):
                        if pose[j].prob < conf_thresh:
                            continue
                        joint_track_name = f'{pose_id}.{self.last_pose_source}{self.last_pose_keypoints}.{j}'
                        track = tracks.get(joint_track_name)
                        if joint_track_name not in track_list:
                            track_reset = False
                            if track is None:
                                track = tracks.new(name=joint_track_name, frame=frame)
                                track_reset = True
                            elif overwrite:
                                track_frames = [a for (a, _) in track.markers.items()]
                                for tf in track_frames:
                                    track.markers.delete_frame(tf)
                                track = tracks.get(joint_track_name)
                                if track is None:
                                    track = tracks.new(name=joint_track_name, frame=frame)
                                track_reset = True
                            if track_reset:
                                track_frames = []
                            else:
                                track_frames = [a for (a, _) in track.markers.items()]
                            track_list[joint_track_name] = set(track_frames)

                        conf = pose[j].prob
                        original_conf = 0
                        if not overwrite and frame in track_list[joint_track_name]:
                            orig_marker = track.markers[frame]
                            area = utils.get_marker_area(orig_marker, clip_size, True)
                            original_conf = area / 100
                        if conf > min(original_conf, 100):
                            x = pose[j].pt.x / clip_size[0]
                            y = (clip_size[1] - pose[j].pt.y) / clip_size[1]
                            box_area = conf * 100
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

    def cancelled_response(self, event, context):
        if event.cancel_msg != '':
            self.report({'INFO'}, event.cancel_msg)

    def error_response(self, event, context):
        if event.error_msg != '':
            self.report({'ERROR'}, event.error_msg)
        if event.msg != '':
            print(event.msg)


CLASSES = [
    DetectPosesOperator
]
