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

if 'posetracks_core' in locals():
    import importlib

    importlib.reload(posetracks_core)
    importlib.reload(global_vars)
    importlib.reload(ui)
    importlib.reload(operators)
    importlib.reload(property_groups)
    importlib.reload(ui_props)
else:
    from . import posetracks_core
    from . import global_vars
    from . import ui
    from . import operators
    from . import property_groups
    from .property_groups import ui_props

bl_info = {
    "name": "PoseTracks",
    "author": "Ian Sloat",
    "version": (1, 0, 0),
    "blender": (2, 93, 0),
    "location": "Movie Clip Editor > Sidebar > PoseTracks",
    "description": "Provides various computer vision and AI tracking tools in the clip editor",
    "warning": "Experimental software! Some features might not work as expected",
    "wiki_url": "",
    "tracker_url": "",
    "category": "Tracking"}

registered = False


def draw_menus_view3d(self, context):
    layout = self.layout
    if context.mode == 'OBJECT':
        layout.menu("VIEW3D_MT_tracking")


def draw_menus_graph_key(self, context):
    layout = self.layout
    layout.menu("GRAPH_MT_tracking_filters")


def draw_ops_track_panel(self, context):
    layout = self.layout
    mute_ops = layout.row(align=True)
    mute_ops.operator("posetracks.mute_tracks_operator")
    mute_ops.operator("posetracks.unmute_tracks_operator")


keymaps = []

def _compat_ver_to_tup(ver):
    ver = [int(v) for v in ver.replace("VER_", "").split("_")]
    return tuple(ver)

def register():
    global registered

    # Core lib compatibility mapping
    compat_map = zip(list(posetracks_core.BlenderVersion.__members__),
                      [int(e) for e in posetracks_core.BlenderVersion.__members__.values()])   
    compat_map = [(_compat_ver_to_tup(n), posetracks_core.BlenderVersion(v)) for n,v in compat_map]
    compat_map.sort(key=lambda item : item[0])
    bpy_ver = bpy.app.version
    compat_ver = None

    if not (compat_map[0][0] <= bpy.app.version <= compat_map[-1][0]):
        print("[PoseTracks] Registration failed.")
        registered = False
        return

    for ver in compat_map:
        if bpy_ver < ver[0]:
            break
        compat_ver = ver[1]

    posetracks_core.set_compatibility_mode(compat_ver)

    print(f"[PoseTracks] Set compatibility mode to {str(compat_ver)}")

    posetracks_core.set_pose_sources(utils.get_pose_sources())
    print("[PoseTracks] Updated pose sources")

    tag_sources = [e for e in posetracks_core.TagDictionary.__members__]
    tag_sources.append('ML')
    posetracks_core.set_tag_sources(tag_sources)
    print("[PoseTracks] Updated tag sources")

    # Component registration

    gc.collect()

    for CLASS in property_groups.ALL_CLASSES:
        bpy.utils.register_class(CLASS)

    for CLASS in operators.ALL_CLASSES:
        bpy.utils.register_class(CLASS)

    for CLASS in ui.ALL_CLASSES:
        bpy.utils.register_class(CLASS)

    bpy.types.Scene.pt_ui_properties = bpy.props.PointerProperty(type=ui_props.UIProperties)

    global_vars.ui_lock_state = False
    global_vars.shutdown_state = False

    global_vars.executor = concurrent.futures.ThreadPoolExecutor()

    # Menu registration

    bpy.types.VIEW3D_MT_editor_menus.append(draw_menus_view3d)
    bpy.types.GRAPH_MT_key.append(draw_menus_graph_key)
    bpy.types.CLIP_PT_track.append(draw_ops_track_panel)

    # Keymaps

    wm = bpy.context.window_manager
    kc = wm.keyconfigs.addon
    if kc:
        km = kc.keymaps.new(name='3D View', space_type='VIEW_3D')
        kmi = km.keymap_items.new("posetracks.triangulate_points_operator", type='T', value='PRESS', ctrl=True)
        keymaps.append((km, kmi))
        kmi = km.keymap_items.new("posetracks.solve_cameras_operator", type='T', value='PRESS', ctrl=True,
                                  shift=True)
        keymaps.append((km, kmi))
    gc.collect()

    print("[PoseTracks] Registration complete.")

    registered = True


def unregister():
    if registered:

        bpy.types.GRAPH_MT_key.remove(draw_menus_graph_key)
        bpy.types.VIEW3D_MT_editor_menus.remove(draw_menus_view3d)

        for km, kmi in keymaps:
            km.keymap_items.remove(kmi)
        keymaps.clear()

        for CLASS in ui.ALL_CLASSES:
            bpy.utils.unregister_class(CLASS)

        for CLASS in operators.ALL_CLASSES:
            bpy.utils.unregister_class(CLASS)

        for CLASS in property_groups.ALL_CLASSES:
            bpy.utils.unregister_class(CLASS)

        global_vars.ui_lock_state = False

        del bpy.types.Scene.pt_ui_properties

        global_vars.shutdown_state = True

        global_vars.executor.shutdown()

        gc.collect()

    print("[PoseTracks] Unregistration complete.")


if __name__ == "__main__":
    register()
