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


class KeepPoseTracksOperator(bpy.types.Operator):
    """Item will be saved on track generation"""
    bl_idname = "motionengine.keep_pose_tracks_operator"
    bl_label = "Make persistent"

    target_index: bpy.props.IntProperty()
    def execute(self, context):
        scene = context.scene
        properties = scene.motion_engine_ui_properties
        current_clip = context.edit_movieclip

        target_list = None

        for item in properties.me_ui_prop_pose_clip_collection:
            if item.clip == current_clip:
                target_list = item.pose_tracks_list
                break

        if target_list is not None:
            target = target_list[self.target_index]

            if target is not None:
                if target.keep_on_regen:
                    target.keep_on_regen = False
                else:
                    target.keep_on_regen = True

        return {'FINISHED'}


class HidePoseTracksOperator(bpy.types.Operator):
    """Marks all tracks related to this pose as hidden"""
    bl_idname = "motionengine.hide_pose_tracks_operator"
    bl_label = "Hide tracks"

    target_index: bpy.props.IntProperty()
    def execute(self, context):
        scene = context.scene
        properties = scene.motion_engine_ui_properties
        current_clip = context.edit_movieclip
        clip_tracks = current_clip.tracking.tracks

        target_list = None

        for item in properties.me_ui_prop_pose_clip_collection:
            if item.clip == current_clip:
                target_list = item.pose_tracks_list
                break

        if target_list is not None:
            target = target_list[self.target_index]

            if target is not None:

                if target.mark_as_hidden:
                    target.mark_as_hidden = False
                else:
                    target.mark_as_hidden = True

                for i in range(target.tracks):
                    track = clip_tracks.get(target.track_prefix + '.' + str(i))
                    if track is not None:
                        track.hide = target.mark_as_hidden

        return {'FINISHED'}


class ME_UL_PoseListUIPanel(bpy.types.UIList):
    def draw_item(self, context, layout, data, item, icon, active_data, active_propname, index):

        custom_icon = "ARMATURE_DATA"

        save_target_index = index

        if self.layout_type in {'DEFAULT', 'COMPACT'}:
            layout.emboss = 'NONE'
            layout.prop(item, "mutable_name", text="", icon=custom_icon)

            row = layout.row()
            row.enabled = False
            row.label(text="id:" + item.track_prefix[4:])

            save_icon = "UNLOCKED"
            if item.keep_on_regen:
                save_icon = "LOCKED"
            op = layout.operator("motionengine.keep_pose_tracks_operator", text="", icon=save_icon)
            op.target_index = index
            hide_icon = "HIDE_OFF"
            if item.mark_as_hidden:
                hide_icon = "HIDE_ON"
            op = layout.operator("motionengine.hide_pose_tracks_operator", text="", icon=hide_icon)
            op.target_index = index

        elif self.layout_type in {'GRID'}:
            layout.alignment = 'CENTER'
            layout.operator(text="", icon=custom_icon)


CLASSES = [
    KeepPoseTracksOperator,
    HidePoseTracksOperator,
    ME_UL_PoseListUIPanel
]
