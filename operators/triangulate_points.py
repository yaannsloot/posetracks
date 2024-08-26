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


def clip_list(self, context):
    solution_ids = {}
    for obj in context.scene.objects:
        if obj.type != 'CAMERA':
            continue
        solution_id = obj.data.get('solution_id', None)
        if solution_id is None:
            continue
        if obj.data.name not in bpy.data.movieclips:
            continue
        if solution_id not in solution_ids:
            solution_ids[solution_id] = []
        solution_ids[solution_id].append(obj.data.name)
    items = []
    for _, cams in solution_ids.items():
        if len(cams) > 1:
            if items:
                items.append(None)
            items.extend([(cam, cam, f'Set anchor view to {cam}') for cam in cams])
    return items


class TriangulatePointsOperator(bpy.types.Operator):
    """Triangulate tracked points and place them in the current scene"""
    bl_idname = "motionengine.triangulate_points_operator"
    bl_label = "Triangulate Points"
    bl_options = {'REGISTER', 'UNDO'}

    views: bpy.props.EnumProperty(
        items=clip_list,
        name="Anchor View",
        description="Clip to use as anchor view"
    )

    @classmethod
    def poll(cls, context):
        return (not global_vars.ui_lock_state
                and context.area.type == 'VIEW_3D'
                and context.mode == 'OBJECT'
                and len(bpy.data.movieclips) > 1
                and len(clip_list(cls, context)) > 1)

    def execute(self, context):
        me.blender.OP_TriangulatePoints(self, self.views)
        return {"FINISHED"}

    def invoke(self, context, event):
        active = context.active_object
        cameras = [item[0] for item in clip_list(self, context)]
        if active is not None and active.type == 'CAMERA' and active.data.name in cameras:
            self.views = active.data.name
            return self.execute(context)
        return self.execute(context)


CLASSES = [
    TriangulatePointsOperator
]
