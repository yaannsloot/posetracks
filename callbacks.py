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
from . import global_vars
from .property_groups import me_data
from .property_groups import me_stats


def write_data_callback(raw_data):
    clip_data = global_vars.properties_tracker
    target_clip = global_vars.clip_tracker
    stats_data = global_vars.stats_tracker
    if clip_data is not None and target_clip is not None:
        print("[MotionEngine] Collecting results...")
        frames = me_data.raw_to_frames(raw_data, 0)
        clip_dict = {}
        for clip in clip_data.items:
            clip_dict[clip.clip] = clip
        target_clip_data = None
        if target_clip in clip_dict:
            target_clip_data = clip_dict[target_clip]
        else:
            target_clip_data = clip_data.items.add()
            target_clip_data.clip = target_clip
        print("[MotionEngine] Writing results to scene data...")
        me_data.clear_blend_clip_data(target_clip)
        me_data.clip_data_to_blend(frames, target_clip_data)
        stat_prop = me_stats.check_stats_for_clip(stats_data, target_clip)
        if stat_prop is None:
            stat_prop = stats_data.add()
        print("[MotionEngine] Calculating run statistics...")
        bpy.ops.motionengine.calculate_statistics_operator()


def ui_draw_callback():
    properties = bpy.context.scene.motion_engine_ui_properties
    if properties.me_ui_redraw_prop:
        properties.me_ui_redraw_prop = False
    else:
        properties.me_ui_redraw_prop = True


def display_warmup_state_callback(value):
    global_vars.warmup_state = value


def ui_lock_callback(value):
    global_vars.ui_lock_state = value
