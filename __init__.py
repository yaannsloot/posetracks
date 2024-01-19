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
import gc

if 'MotionEngine' in locals():
    import importlib

    importlib.reload(MotionEngine)
    importlib.reload(MEPython)
    importlib.reload(global_vars)
    importlib.reload(ui)
    importlib.reload(operators)
    importlib.reload(property_groups)
    importlib.reload(me_data)
    importlib.reload(me_ui)
else:
    from . import MotionEngine
    from .MotionEngine import MEPython
    from . import global_vars
    from . import ui
    from . import operators
    from . import property_groups
    from .property_groups import me_data
    from .property_groups import me_ui

bl_info = {
    "name": "MotionEngine",
    "author": "Ian Sloat",
    "version": (0, 1, 0),
    "blender": (2, 93, 0),
    "location": "Clip Editor > UI",
    "description": "AI based motion tracking system for blender",
    "warning": "Pre-release software! Some features might not work as expected",
    "wiki_url": "",
    "tracker_url": "",
    "category": "Motion Tracking"}


def register():
    gc.collect()

    for CLASS in property_groups.ALL_CLASSES:
        bpy.utils.register_class(CLASS)

    for CLASS in operators.ALL_CLASSES:
        bpy.utils.register_class(CLASS)

    for CLASS in ui.ALL_CLASSES:
        bpy.utils.register_class(CLASS)

    bpy.types.Scene.motion_engine_data = bpy.props.PointerProperty(type=me_data.MEClipDataCollection)
    bpy.types.Scene.motion_engine_ui_properties = bpy.props.PointerProperty(type=me_ui.MotionEngineUIProperties)

    global_vars.me_detectpose_model = MEPython.dnn.TopDownPoseDetector()
    global_vars.properties_tracker = None
    global_vars.clip_tracker = None
    global_vars.stats_tracker = None
    global_vars.context_tracker = None
    global_vars.warmup_state = False
    global_vars.ui_lock_state = False

    gc.collect()

    print("[MotionEngine] Registration complete.")


def unregister():
    for CLASS in ui.ALL_CLASSES:
        bpy.utils.unregister_class(CLASS)

    for CLASS in operators.ALL_CLASSES:
        bpy.utils.unregister_class(CLASS)

    for CLASS in property_groups.ALL_CLASSES:
        bpy.utils.unregister_class(CLASS)

    global_vars.me_detectpose_model.unload_all()
    global_vars.properties_tracker = None
    global_vars.clip_tracker = None
    global_vars.stats_tracker = None
    global_vars.context_tracker = None
    global_vars.warmup_state = False
    global_vars.ui_lock_state = False

    del bpy.types.Scene.motion_engine_ui_properties
    del bpy.types.Scene.motion_engine_data

    gc.collect()

    print("[MotionEngine] Unregistration complete.")


if __name__ == "__main__":
    register()
