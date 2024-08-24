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
    importlib.reload(ui_props)
else:
    from . import MotionEngine
    from . import global_vars
    from . import ui
    from . import operators
    from . import property_groups
    from .property_groups import ui_props

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
    "category": "Tracking"}

registered = False


def register():
    global registered

    # Core lib compatibility mapping

    if not ((2, 93, 0) <= bpy.app.version <= (4, 2, 1)):
        print("[MotionEngine] Registration failed.")
        registered = False
        return

    compat_ver = None

    if bpy.app.version < (2, 93, 4):
        compat_ver = MotionEngine.VER_2_93_0
    elif bpy.app.version < (3, 0, 0):
        compat_ver = MotionEngine.VER_2_93_4
    elif bpy.app.version < (3, 1, 0):
        compat_ver = MotionEngine.VER_3_0_0
    elif bpy.app.version < (3, 2, 0):
        compat_ver = MotionEngine.VER_3_1_0
    elif bpy.app.version < (3, 3, 0):
        compat_ver = MotionEngine.VER_3_2_0
    elif bpy.app.version < (3, 4, 0):
        compat_ver = MotionEngine.VER_3_3_0
    elif bpy.app.version < (3, 5, 0):
        compat_ver = MotionEngine.VER_3_4_0
    elif bpy.app.version < (3, 6, 0):
        compat_ver = MotionEngine.VER_3_5_0
    elif bpy.app.version < (3, 6, 8):
        compat_ver = MotionEngine.VER_3_6_0
    elif bpy.app.version < (4, 0, 0):
        compat_ver = MotionEngine.VER_3_6_8
    elif bpy.app.version < (4, 1, 0):
        compat_ver = MotionEngine.VER_4_0_0
    elif bpy.app.version < (4, 2, 0):
        compat_ver = MotionEngine.VER_4_1_0
    elif bpy.app.version < (4, 2, 1):
        compat_ver = MotionEngine.VER_4_2_0
    else:
        compat_ver = MotionEngine.VER_4_2_1

    MotionEngine.set_compatibility_mode(compat_ver)

    print(f"[MotionEngine] Set compatibility mode to {str(compat_ver)}")

    # Component registration

    gc.collect()

    for CLASS in property_groups.ALL_CLASSES:
        bpy.utils.register_class(CLASS)

    for CLASS in operators.ALL_CLASSES:
        bpy.utils.register_class(CLASS)

    for CLASS in ui.ALL_CLASSES:
        bpy.utils.register_class(CLASS)

    bpy.types.Scene.motion_engine_ui_properties = bpy.props.PointerProperty(type=ui_props.UIProperties)

    global_vars.ui_lock_state = False
    global_vars.shutdown_state = False

    global_vars.executor = concurrent.futures.ThreadPoolExecutor()

    gc.collect()

    print("[MotionEngine] Registration complete.")

    registered = True


def unregister():
    if registered:
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
