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
from .. import global_vars
import bpy


def get_selected_tracks():
    clip = bpy.context.edit_movieclip
    return [t for t in clip.tracking.tracks if t.select]


def intervals(values, key=None):
    if key is None:
        key = lambda val: val
    if not values:
        return values
    values = sorted(values, key=key)
    output = []
    last = values[0]
    inter = []
    for v in values:
        if key(v) - key(last) > 1:
            output.append(inter)
            inter = []
        inter.append(v)
        last = v
    if inter:
        output.append(inter)
    return output


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
        active_clip = bpy.data.movieclips[self.active_clip]
        selected_tracks = [active_clip.tracking.tracks[track] for track in self.selected_tracks]
        width = active_clip.size[0]
        height = active_clip.size[1]
        for track, frames, x, y in zip(selected_tracks, self.track_frames, self.track_x, self.track_y):
            for f_inter, x_inter, y_inter in zip(frames, x, y):
                kernel_width = min(len(f_inter) - 1, self.kernel_width)
                if kernel_width < 1:
                    continue
                x_inter = me.tracking.g_conv_1d(x_inter, kernel_width)
                y_inter = me.tracking.g_conv_1d(y_inter, kernel_width)
                for f, co in zip(f_inter, zip(x_inter, y_inter)):
                    track.markers.find_frame(f).co = (co[0] / width, co[1] / height)
        return {'FINISHED'}

    def invoke(self, context, event):
        active_clip = context.edit_movieclip
        width = active_clip.size[0]
        height = active_clip.size[1]
        selected_tracks = get_selected_tracks()
        self.track_frames = []
        self.track_x = []
        self.track_y = []
        for track in selected_tracks:
            frames = intervals([marker.frame for marker in track.markers])
            x = []
            y = []
            for inter in frames:
                co_inter = [track.markers.find_frame(f).co for f in inter]
                x.append([co.x * width for co in co_inter])
                y.append([co.y * height for co in co_inter])
            self.track_frames.append(frames)
            self.track_x.append(x)
            self.track_y.append(y)
        self.selected_tracks = [track.name for track in selected_tracks]
        self.active_clip = active_clip.name
        return self.execute(context)


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
        fcurves = [bpy.data.actions[action].fcurves.find(data_path, index=index)
                   for action, data_path, index in self.f_curve_paths]
        for fcurve, frame_intervals in zip(fcurves, self.fcurve_frames):
            for interval in frame_intervals:
                kernel_width = min(len(interval) - 1, self.kernel_width)
                if kernel_width < 1:
                    continue
                # insert is crazy slow for no good reason so the keys will be updated this way
                keys, vals = zip(*interval)
                vals = me.tracking.g_conv_1d(vals, kernel_width)
                key_table = dict(zip(keys, vals))
                for key in fcurve.keyframe_points:
                    if key.co[0] in key_table:
                        key.co[1] = key_table[key.co[0]]
            fcurve.update()
        return {'FINISHED'}

    def invoke(self, context, event):
        fcurves = context.selected_editable_fcurves
        self.fcurve_frames = [intervals([k.co.copy() for k in fcurve.keyframe_points if k.select_control_point],
                                        key=lambda v: v[0]) for fcurve in fcurves]
        self.f_curve_paths = [(fcurve.id_data.name, fcurve.data_path, fcurve.array_index) for fcurve in fcurves]
        return self.execute(context)


CLASSES = [
    FilterTrackGaussian,
    FilterFCurvesGaussian
]
