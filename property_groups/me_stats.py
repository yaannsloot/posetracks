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
