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
from .me_data import check_data_for_clip


def calculate_statistics(clip_data, stat_prop):
    stat_prop.clip = clip_data.clip
    frame_total = 0
    pose_total = 0
    confidence_sum = 0
    mean50_list = []
    for frame in clip_data.frames:
        if frame.poses is not None and len(frame.poses) > 0:
            frame_total = frame_total + 1
            if len(frame.poses) > pose_total:
                pose_total = len(frame.poses)
            frame_sum = 0
            frame50_list = []
            for pose in frame.poses:
                pose_sum = 0
                pose50_list = []
                for joint in pose.joints:
                    pose_sum = pose_sum + joint.prob
                    if joint.prob > 0.5:
                        pose50_list.append(joint.prob)
                frame_sum = frame_sum + pose_sum / len(pose.joints)
                if pose50_list:
                    frame50_list.append(sum(pose50_list) / len(pose50_list))
            confidence_sum = confidence_sum + frame_sum / len(frame.poses)
            if frame50_list:
                mean50_list.append(sum(frame50_list) / len(frame50_list))
    stat_prop.valid_frames = frame_total
    stat_prop.total_poses = pose_total
    if confidence_sum > 0 and frame_total > 0:
        stat_prop.mean_confidence = confidence_sum / frame_total
    if mean50_list:
        stat_prop.mean50_conf = sum(mean50_list) / len(mean50_list)


def check_stats_for_clip(stats_data, movie_clip):
    stat_prop = None
    for entry in stats_data:
        if entry.clip == movie_clip:
            stat_prop = entry
            break
    return stat_prop


def get_run_statistics(me_data, stats_data, movie_clip):
    clip_data = check_data_for_clip(me_data, movie_clip)
    stat_prop = check_stats_for_clip(stats_data, movie_clip)
    if clip_data is not None and stat_prop is not None:
        return stat_prop
    return None


class MotionEngineRunStatistics(bpy.types.PropertyGroup):
    clip: bpy.props.PointerProperty(type=bpy.types.MovieClip)
    valid_frames: bpy.props.IntProperty()
    total_poses: bpy.props.IntProperty()
    mean_confidence: bpy.props.FloatProperty()
    mean50_conf: bpy.props.FloatProperty()


CLASSES = [
    MotionEngineRunStatistics
]
