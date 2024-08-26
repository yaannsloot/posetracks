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


class VIEW3D_MT_tracking(bpy.types.Menu):
    bl_label = "Tracking"
    bl_idname = "VIEW3D_MT_tracking"

    def draw(self, context):
        layout = self.layout
        layout.operator("motionengine.solve_cameras_operator")
        layout.operator("motionengine.triangulate_points_operator")


CLASSES = [
    VIEW3D_MT_tracking
]
