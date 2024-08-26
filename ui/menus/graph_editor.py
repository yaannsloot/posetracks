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


class GRAPH_MT_tracking_filters(bpy.types.Menu):
    bl_label = "Tracking Filters"
    bl_idname = "GRAPH_MT_tracking_filters"

    def draw(self, context):
        layout = self.layout
        layout.operator("motionengine.filter_curves_gaussian_operator")


CLASSES = [
    GRAPH_MT_tracking_filters
]
