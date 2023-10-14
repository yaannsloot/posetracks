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
from .. import global_vars
from ..property_groups import me_data
from ..property_groups import me_stats


class CreateTracksOperator(bpy.types.Operator):
    """Create tracks from analysis data"""
    bl_idname = "motionengine.create_tracks_operator"
    bl_label = "Create tracks"

    def execute(self, context):
        if not global_vars.ui_lock_state:
            scene = context.scene
            properties = scene.motion_engine_ui_properties
            current_clip = context.edit_movieclip
            me_scene_data = scene.motion_engine_data
            stats_data = scene.motion_engine_ui_properties.me_ui_prop_stats_collection
            stat_prop = me_stats.check_stats_for_clip(stats_data, current_clip)
            clip_data = me_data.check_data_for_clip(me_scene_data, current_clip)
            pose_tracks_list = me_data.get_pose_tracks_list(current_clip, properties)
            clip_size = current_clip.size

            if clip_data is not None and len(clip_data.frames) > 0:
                clear_tracks(current_clip, pose_tracks_list)

                title_index = len(pose_tracks_list)
                current_pose_tracks = {}

                for frame in clip_data.frames:
                    for pose in frame.poses:
                        if pose.id not in current_pose_tracks:
                            new_pose_tracks = create_pose_tracks("Pose " + str(pose.id), len(pose.joints), current_clip,
                                                                 pose_tracks_list)
                            pose_tracks_dict = get_track_dict(current_clip, new_pose_tracks)
                            current_pose_tracks[pose.id] = (new_pose_tracks, pose_tracks_dict)
                        pose_tracks = current_pose_tracks[pose.id][0]
                        tracks_dict = current_pose_tracks[pose.id][1]
                        for joint in pose.joints:
                            track = tracks_dict[joint.id]
                            if track is not None:
                                markers = track.markers
                                x = joint.pt.x / clip_size[0]
                                y = (clip_size[1] - joint.pt.y) / clip_size[1]
                                markers.insert_frame(frame.id + 1, co=(x, y))

        return {'FINISHED'}


CLASSES = [
    CreateTracksOperator
]
