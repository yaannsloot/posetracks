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
from ... import global_vars, utils


class TrackManagerUIPanel(bpy.types.Panel):
    bl_label = "Track Manager"
    bl_idname = "MOTIONENGINE_TRACK_MANAGER_PT_panel"
    bl_space_type = "CLIP_EDITOR"
    bl_region_type = "UI"
    bl_category = "MotionEngine"

    display_priority = 3

    @classmethod
    def poll(cls, context):
        return context.edit_movieclip is not None

    def draw(self, context):
        layout = self.layout
        scene = context.scene
        properties = scene.motion_engine_ui_properties
        current_clip = context.edit_movieclip

        if current_clip is not None:
            active_track = current_clip.tracking.tracks.active
        else:
            active_track = None
        ui_lock = global_vars.ui_lock_state

        layout.enabled = not ui_lock and active_track is not None

        if active_track is None:
            layout.label(text='No active track')
            return

        clip_info = utils.ClipInfo(current_clip)

        row = layout.row()

        row_ele = row.row()

        row_ele.prop(properties, 'me_ui_active_track_name', text='')

        row_ele.enabled = not utils.is_valid_tag_name(active_track.name)

        row_ops = row.row(align=True)

        if utils.is_valid_joint_name(active_track.name):
            row_ops.operator("motionengine.select_pose_operator", icon='RESTRICT_SELECT_OFF', text='')
        if active_track.lock:
            row_ops.operator("motionengine.unlock_tracks_operator", icon='LOCKED', text='', depress=True)
        else:
            row_ops.operator("motionengine.lock_tracks_operator", icon='UNLOCKED', text='')

        if utils.is_valid_joint_name(active_track.name):
            split_name = active_track.name.split('.')
            joint_id = int(split_name[-1])
            joint_source = split_name[-2]
            joint_tracks = utils.get_joint_tracks(current_clip)
            num_joints_in_clip = len(joint_tracks[0][properties.me_ui_active_track_name][joint_source].keys())
            joint_area = utils.get_marker_area(
                active_track.markers.find_frame(clip_info.scene_to_clip(scene.frame_current), exact=False),
                tuple(clip_info.clip_size), True)
            box_elements = layout.column()
            box_row = box_elements.row()
            box_row.label(text=f'Pose Info', icon='ARMATURE_DATA')
            box_row = box_elements.row()
            box_row.label(text=f'Source: {joint_source.capitalize()}')
            box_row = box_elements.row()
            box_row.label(text=f'Total tracks: {num_joints_in_clip}')
            box_row = box_elements.row()
            box_row.label(text=f'Active ID: {joint_id}')
            box_row = box_elements.row()
            box_row.label(text=f'Confidence: {joint_area:0.1f}')

        if utils.is_valid_tag_name(active_track.name):
            row = layout.row()
            row.operator("motionengine.solve_camera_from_tag_operator")
            row.enabled = not ui_lock


CLASSES = [
    TrackManagerUIPanel
]
