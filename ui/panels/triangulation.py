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
from ... import global_vars


class TrackingView3DUIPanel(bpy.types.Panel):
    bl_label = "Tracking"
    bl_idname = "MOTIONENGINE_TRACKING_VIEW3D_PT_panel"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "MotionEngine"
    bl_context = "objectmode"

    def draw(self, context):
        layout = self.layout
        scene = context.scene
        properties = scene.motion_engine_ui_properties
        ui_lock = global_vars.ui_lock_state

        layout.enabled = not ui_lock

        box = layout.box()
        box = box.column(align=True)
        row = box.row()
        row.label(text='Anchor view:')
        row = box.row()
        row.prop(properties, 'me_ui_prop_anchor_cam_selection', text='')

        column = layout.column(align=True)
        row = column.row(align=True)

        row.operator("motionengine.solve_cameras_operator")

        row = column.row(align=True)

        row.prop(properties, 'me_ui_prop_solution_scale')

        row = layout.row()

        row.operator("motionengine.triangulate_points_operator")


CLASSES = [
    TrackingView3DUIPanel
]
