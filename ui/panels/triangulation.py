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


class TriangulationUIPanel(bpy.types.Panel):
    bl_label = "Triangulation"
    bl_idname = "MOTIONENGINE_TRIANGULATION_PT_panel"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "MotionEngine"

    def draw(self, context):
        layout = self.layout
        scene = context.scene
        ui_lock = global_vars.ui_lock_state

        layout.enabled = not ui_lock

        row = layout.row()

        row.operator("motionengine.triangulate_points_operator")


CLASSES = [
    TriangulationUIPanel
]
