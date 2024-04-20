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
from .. import MotionEngine as me
from .. import global_vars


class FilterFCurvesOperator(bpy.types.Operator):
    """Refine location keyframes with a kalman filter"""
    bl_idname = "motionengine.filter_fcurves_operator"
    bl_label = "Filter FCurves"

    def execute(self, context):
        if not global_vars.ui_lock_state:

            scene = context.scene

            me_collection = scene.collection.children.get("MotionEngine")

            if me_collection is not None:
                for obj in me_collection.objects:
                    me.filter_3D(obj, 1e-2, 5e-4)

        return {"FINISHED"}


CLASSES = [
    FilterFCurvesOperator
]
