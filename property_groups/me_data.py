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
from .. import utils
from ..MotionEngine import MEPython


def point_to_blend(source, target):
    target.x = source.x
    target.y = source.y


def joint_to_blend(source, target, target_id):
    point_to_blend(source.pt, target.pt)
    target.prob = source.prob
    target.id = target_id


def pose_to_blend(source, target, target_id):
    source_ids = source.get_joint_ids()
    target_ids = {}
    for target_item in target.joints:
        target_ids[target_item.id] = target_item
    for source_id in source_ids:
        target_joint = None
        if source_id in target_ids:
            target_joint = target_ids[source_id]
        else:
            target_joint = target.joints.add()
        joint_to_blend(source[source_id], target_joint, source_id)
    target.id = target_id


def frame_to_blend(source, target, target_id):
    source_ids = source.get_pose_ids()
    target_ids = {}
    for target_item in target.poses:
        target_ids[target_item.id] = target_item
    for source_id in source_ids:
        target_pose = None
        if source_id in target_ids:
            target_pose = target_ids[source_id]
        else:
            target_pose = target.poses.add()
        pose_to_blend(source[source_id], target_pose, source_id)
    target.id = target_id


def clip_data_to_blend(source, target):
    source_ids = source.get_pose_frame_ids()
    target_ids = {}
    for target_item in target.frames:
        target_ids[target_item.id] = target_item
    for source_id in source_ids:
        target_frame = None
        if source_id in target_ids:
            target_frame = target_ids[source_id]
        else:
            target_frame = target.frames.add()
        frame_to_blend(source[source_id], target_frame, source_id)


def check_data_for_clip(me_data, movie_clip):
    #
    clip_data = None
    for entry in me_data.items:
        if entry.clip == movie_clip:
            clip_data = entry
            break
    return clip_data


def blend_to_point(source):
    return MEPython.Point(source.x, source.y)


def blend_to_joint(source):
    return MEPython.dnn.Joint(blend_to_point(source.pt), source.prob)


def blend_to_pose(source):
    pose = MEPython.dnn.Pose()
    for s_joint in source.joints:
        pose.set_joint(s_joint.id, blend_to_joint(s_joint))
    return pose


def blend_to_frame(source):
    frame = MEPython.dnn.PoseCollection()
    for s_pose in source.poses:
        frame.set_pose(s_pose.id, blend_to_pose(s_pose))
    return frame


def blend_to_clip_data(source):
    data = MEPython.dnn.PoseFrames()
    for s_frame in source.frames:
        data.set_pose_frame(s_frame.id, blend_to_frame(s_frame))
    return data


def raw_to_frames(source, start_index):
    frames = MEPython.dnn.PoseFrames()
    for f, frame in enumerate(source):
        collection = MEPython.dnn.PoseCollection()
        for p, pose in enumerate(frame):
            collection.set_pose(p, pose)
        frames.set_pose_frame(f + start_index, collection)
    return frames


def clear_blend_pose(source):
    source.joints.clear()


def clear_blend_frame(source):
    for pose in source.poses:
        clear_blend_pose(pose)
    source.poses.clear()


def clear_blend_clip_data(source: bpy.types.MovieClip):
    clip_data_collection = bpy.context.scene.motion_engine_data.items
    for clip_data in clip_data_collection:
        if clip_data.clip == source:
            for frame in clip_data.frames:
                clear_blend_frame(frame)
            clip_data.frames.clear()
            break


def get_track_tag(pose_tracks, joint_id):
    return pose_tracks.track_prefix + "." + str(joint_id)


def get_track_dict(clip, pose_tracks):
    track_dict = {}
    for track_id in range(pose_tracks.tracks):
        track_name = get_track_tag(pose_tracks, track_id)
        track = clip.tracking.tracks.get(track_name)
        if track is not None:
            track_dict[track_id] = track
    return track_dict


def create_pose_tracks(name, joint_num, clip, pose_tracks_list):
    # Needs to have a conflict check for new names

    names_list = []
    prefix_list = []
    for entry in pose_tracks_list:
        names_list.append(entry.name)
        prefix_list.append(entry.track_prefix)

    max_tries = 65536

    if name in names_list:
        for i in range(max_tries):
            n = "Pose " + str(i)
            if n not in names_list:
                name = n
                break

    prefix = "Pose 0"
    if prefix in prefix_list:
        for i in range(max_tries):
            n = "Pose " + str(i)
            if n not in prefix_list:
                prefix = n
                break

    pose_tracks = pose_tracks_list.add()
    pose_tracks.name = name
    pose_tracks.track_prefix = prefix
    pose_tracks.tracks = joint_num
    pose_tracks.keep_on_regen = False
    new_track_list = []
    for j in range(joint_num):
        new_track_list.append(clip.tracking.tracks.new(name=get_track_tag(pose_tracks, j)))
    utils.set_lock_on_tracks(new_track_list, True)
    return pose_tracks


def clear_tracks(clip, pose_tracks_list, ignore_keep=False):
    target_tracks = []
    for pose_tracks in pose_tracks_list:
        if not pose_tracks.keep_on_regen or ignore_keep:
            track_dict = get_track_dict(clip, pose_tracks)
            target_tracks.extend(list(track_dict.values()))
    utils.set_hidden_on_tracks(target_tracks, False)
    utils.set_select_tracks(clip.tracking.tracks, False)
    utils.set_select_tracks(target_tracks, True)

    bpy.ops.clip.delete_track()

    current_index = 0
    while current_index < len(pose_tracks_list):
        increment = True
        pose_tracks = pose_tracks_list[current_index]
        if not pose_tracks.keep_on_regen or ignore_keep:
            pose_tracks_list.remove(current_index)
            increment = False
        if increment:
            current_index = current_index + 1


def pose_tracks_name_check(self, value):
    if value != "":
        scene = bpy.context.scene
        properties = scene.motion_engine_ui_properties
        current_clip = bpy.context.edit_movieclip

        pose_clip_collection = properties.me_ui_prop_pose_clip_collection

        pose_tracks_list = None

        for entry in pose_clip_collection:
            if entry.clip == current_clip:
                pose_tracks_list = entry.pose_tracks_list

        pose_names = []
        if pose_tracks_list is not None:
            for entry in pose_tracks_list:
                pose_names.append(entry.name)
            if value not in pose_names:
                self.name = value
                self.keep_on_regen = True

    return None


def pose_tracks_name_get(self):
    return self.name


class MEPointBlender(bpy.types.PropertyGroup):
    x: bpy.props.FloatProperty()
    y: bpy.props.FloatProperty()


class MEJointBlender(bpy.types.PropertyGroup):
    id: bpy.props.IntProperty()
    prob: bpy.props.FloatProperty()
    pt: bpy.props.PointerProperty(type=MEPointBlender)


class MEPoseBlender(bpy.types.PropertyGroup):
    id: bpy.props.IntProperty()
    joints: bpy.props.CollectionProperty(type=MEJointBlender)


class MEFrameBlender(bpy.types.PropertyGroup):
    id: bpy.props.IntProperty()
    poses: bpy.props.CollectionProperty(type=MEPoseBlender)


class MEClipData(bpy.types.PropertyGroup):
    clip: bpy.props.PointerProperty(type=bpy.types.MovieClip)
    frames: bpy.props.CollectionProperty(type=MEFrameBlender)


class MEClipDataCollection(bpy.types.PropertyGroup):
    items: bpy.props.CollectionProperty(type=MEClipData)


class MEPoseTracks(bpy.types.PropertyGroup):
    name: bpy.props.StringProperty()  # can be changed by the user
    track_prefix: bpy.props.StringProperty()  # will be kept throughout property's lifetime
    mutable_name: bpy.props.StringProperty(get=pose_tracks_name_get, set=pose_tracks_name_check)
    tracks: bpy.props.IntProperty()
    keep_on_regen: bpy.props.BoolProperty()
    mark_as_hidden: bpy.props.BoolProperty()


class MEPoseTracksClip(bpy.types.PropertyGroup):
    clip: bpy.props.PointerProperty(type=bpy.types.MovieClip)
    pose_tracks_list: bpy.props.CollectionProperty(type=MEPoseTracks)


CLASSES = [
    MEPointBlender,
    MEJointBlender,
    MEPoseBlender,
    MEFrameBlender,
    MEClipData,
    MEClipDataCollection,
    MEPoseTracks,
    MEPoseTracksClip
]
