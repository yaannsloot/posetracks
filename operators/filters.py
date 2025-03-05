"""
Copyright (C) 2025 Ian Sloat
Licensed under the GNU GPLv3 or later. See <https://www.gnu.org/licenses/>.
"""

from .. import posetracks_core as pt_core
import bpy


def get_selected_tracks():
    clip = bpy.context.edit_movieclip
    return [t for t in clip.tracking.tracks if t.select]


class FilterTrackGaussian(bpy.types.Operator):
    """Apply a gaussian filter to selected tracks"""
    bl_idname = "posetracks.filter_tracks_gaussian_operator"
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
        pt_core.blender.OP_FilterTrackGaussian(self.kernel_width)
        return {'FINISHED'}


class FilterTrackKalman(bpy.types.Operator):
    """Apply a kalman filter to selected tracks"""
    bl_idname = "posetracks.filter_tracks_kalman_operator"
    bl_label = "Apply Kalman Filter"
    bl_options = {'REGISTER', 'UNDO'}

    noise_scale: bpy.props.FloatProperty(
        name="Noise Scale",
        description="Relative noisiness of input",
        default=0.5,
        min=0,
        max=1,
    )

    @classmethod
    def poll(cls, context):
        return (context.area.type == 'CLIP_EDITOR' and
                context.edit_movieclip is not None and
                get_selected_tracks())

    def execute(self, context):
        pt_core.blender.OP_FilterTrackKalman(self.noise_scale)
        return {'FINISHED'}


class FilterFCurvesGaussian(bpy.types.Operator):
    """Apply a gaussian filter to selected f-curves"""
    bl_idname = "posetracks.filter_curves_gaussian_operator"
    bl_label = "Apply Gaussian Filter"
    bl_options = {'REGISTER', 'UNDO'}

    kernel_width: bpy.props.IntProperty(
        name="Kernel Width",
        description="Width of filter kernel",
        default=1,
        min=1,
    )

    selected_only: bpy.props.BoolProperty(
        name="Selected Only",
        description="Only filter selected keys",
        default=False,
    )

    @classmethod
    def poll(cls, context):
        return (context.area.type == 'GRAPH_EDITOR' and
                context.selected_editable_fcurves)

    def execute(self, context):
        pt_core.blender.OP_FilterFCurvesGaussian(
            self.kernel_width, self.selected_only)
        return {'FINISHED'}


class FilterFCurvesKalman(bpy.types.Operator):
    """Apply a kalman filter to selected f-curves"""
    bl_idname = "posetracks.filter_curves_kalman_operator"
    bl_label = "Apply Kalman Filter"
    bl_options = {'REGISTER', 'UNDO'}

    noise_scale: bpy.props.FloatProperty(
        name="Noise Scale",
        description="Relative noisiness of input",
        default=0.5,
        min=0,
        max=1,
    )

    selected_only: bpy.props.BoolProperty(
        name="Selected Only",
        description="Only filter selected keys",
        default=False,
    )

    @classmethod
    def poll(cls, context):
        return (context.area.type == 'GRAPH_EDITOR' and
                context.selected_editable_fcurves)

    def execute(self, context):
        pt_core.blender.OP_FilterFCurvesKalman(
            self.noise_scale, self.selected_only)
        return {'FINISHED'}


CLASSES = [
    FilterTrackGaussian,
    FilterTrackKalman,
    FilterFCurvesGaussian,
    FilterFCurvesKalman
]
