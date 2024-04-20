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
from .. import global_vars
from ..ui.panels import track_manager
from ..property_groups import me_data
from ..property_groups import me_stats


def ensure_pose_tracks_clip(clip, properties):
    pose_tracks_clip = track_manager.get_pose_tracks_clip(clip, properties)
    clip_collection = properties.me_ui_prop_pose_clip_collection
    if pose_tracks_clip is None:
        pose_tracks_clip = clip_collection.add()
        pose_tracks_clip.clip = clip
    return pose_tracks_clip


class CreateTracksOperator(bpy.types.Operator):
    """Create tracks from analysis data"""
    bl_idname = "motionengine.create_tracks_operator"
    bl_label = "Create tracks"

    def execute(self, context):
        bpy.ops.motionengine.prune_data_operator()
        if not global_vars.ui_lock_state:
            scene = context.scene
            properties = scene.motion_engine_ui_properties
            current_clip = context.edit_movieclip
            me_scene_data = scene.motion_engine_data

            clip_data = None
            for entry in me_scene_data.items:
                if entry.clip == current_clip:
                    clip_data = entry
                    break

            if clip_data is not None:
                pose_clips_list = properties.me_ui_prop_pose_clip_collection

                pose_tracks_list = None
                for entry in pose_clips_list:
                    if entry.clip == current_clip:
                        pose_tracks_list = entry.pose_tracks_list
                        break

                if pose_tracks_list is None:
                    pose_clip = pose_clips_list.add()
                    pose_clip.clip = current_clip
                    pose_tracks_list = pose_clip.pose_tracks_list

                clip_size = current_clip.size

                if len(clip_data.frames) > 0:
                    me_data.clear_tracks(current_clip, pose_tracks_list)

                    title_index = len(pose_tracks_list)
                    current_pose_tracks = {}

                    for frame in clip_data.frames:
                        for pose in frame.poses:
                            if pose.id not in current_pose_tracks:
                                new_pose_tracks = me_data.create_pose_tracks("Pose " + str(pose.id), len(pose.joints),
                                                                             current_clip, pose_tracks_list)

                                pose_tracks_dict = me_data.get_track_dict(current_clip, new_pose_tracks)
                                current_pose_tracks[pose.id] = (new_pose_tracks, pose_tracks_dict)
                            pose_tracks = current_pose_tracks[pose.id][0]
                            tracks_dict = current_pose_tracks[pose.id][1]
                            for joint in pose.joints:
                                track = tracks_dict[joint.id]
                                if track is not None:
                                    markers = track.markers
                                    x = joint.pt.x / clip_size[0]
                                    y = (clip_size[1] - joint.pt.y) / clip_size[1]
                                    marker = markers.insert_frame(frame.id + 1, co=(x, y))
                                    box_area = joint.prob * 100
                                    side_length = math.sqrt(box_area)
                                    x1 = -1 * (side_length / 2) / clip_size[0]
                                    x2 = (side_length / 2) / clip_size[0]
                                    y1 = -1 * (side_length / 2) / clip_size[1]
                                    y2 = (side_length / 2) / clip_size[1]

                                    marker.pattern_corners = ((x1, y1),
                                                              (x2, y1),
                                                              (x2, y2),
                                                              (x1, y2))

        return {'FINISHED'}


CLASSES = [
    CreateTracksOperator
]
