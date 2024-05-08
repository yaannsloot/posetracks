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
import concurrent.futures

if 'MotionEngine' in locals():
    import importlib
    importlib.reload(MotionEngine)
    importlib.reload(global_vars)
    importlib.reload(ui)
    importlib.reload(operators)
    importlib.reload(property_groups)
    importlib.reload(me_ui)
else:
    from . import MotionEngine
    from . import global_vars
    from . import ui
    from . import operators
    from . import property_groups
    from .property_groups import me_ui

bl_info = {
    "name": "MotionEngine",
    "author": "Ian Sloat",
    "version": (1, 0, 0),
    "blender": (2, 93, 0),
    "location": "Movie Clip Editor > Sidebar > MotionEngine",
    "description": "Provides various computer vision and AI tracking tools in the clip editor",
    "warning": "Experimental software! Some features might not work as expected",
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

    bpy.types.Scene.motion_engine_ui_properties = bpy.props.PointerProperty(type=me_ui.MotionEngineUIProperties)

    global_vars.ui_lock_state = False
    global_vars.shutdown_state = False

    global_vars.executor = concurrent.futures.ThreadPoolExecutor()

    gc.collect()

    print("[MotionEngine] Registration complete.")


def unregister():
    for CLASS in ui.ALL_CLASSES:
        bpy.utils.unregister_class(CLASS)

    for CLASS in operators.ALL_CLASSES:
        bpy.utils.unregister_class(CLASS)

    for CLASS in property_groups.ALL_CLASSES:
        bpy.utils.unregister_class(CLASS)

    global_vars.ui_lock_state = False

    del bpy.types.Scene.motion_engine_ui_properties

    global_vars.shutdown_state = True

    global_vars.executor.shutdown()

    gc.collect()

    print("[MotionEngine] Unregistration complete.")


if __name__ == "__main__":
    register()
