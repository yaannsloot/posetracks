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

from .. import MotionEngine as me
import bpy


def get_selected_tracks():
    clip = bpy.context.edit_movieclip
    return [t for t in clip.tracking.tracks if t.select]


class FilterTrackGaussian(bpy.types.Operator):
    """Apply a gaussian filter to selected tracks"""
    bl_idname = "motionengine.filter_tracks_gaussian_operator"
    bl_label = "Apply Gaussian Filter"
    bl_options = {'REGISTER', 'UNDO'}

    kernel_width: bpy.props.IntProperty(
        name="Kernel Width",
        description="Width of filter kernel",
        default=1,
        min=1,
    )

    @classmethod
    def poll(cls, context):
        return (context.area.type == 'CLIP_EDITOR' and
                context.edit_movieclip is not None and
                get_selected_tracks())

    def execute(self, context):
        me.blender.OP_FilterTrackGaussian(self.kernel_width)
        return {'FINISHED'}


class FilterFCurvesGaussian(bpy.types.Operator):
    """Apply a gaussian filter to selected f-curves"""
    bl_idname = "motionengine.filter_curves_gaussian_operator"
    bl_label = "Apply Gaussian Filter"
    bl_options = {'REGISTER', 'UNDO'}

    kernel_width: bpy.props.IntProperty(
        name="Kernel Width",
        description="Width of filter kernel",
        default=1,
        min=1,
    )

    @classmethod
    def poll(cls, context):
        return (context.area.type == 'GRAPH_EDITOR' and
                context.selected_editable_fcurves)

    def execute(self, context):
        me.blender.OP_FilterFCurvesGaussian(self.kernel_width)
        return {'FINISHED'}


CLASSES = [
    FilterTrackGaussian,
    FilterFCurvesGaussian
]
