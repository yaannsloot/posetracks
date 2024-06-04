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
from ...operators.rigging_constraints import (gen_new_types, track_axis_button_lbl, track_axis_dir_lbl,
                                              track_axis_dir_id)
from ...operators.rigging_constraints import contexts as rigging_op_contexts

idname_base = "MOTIONENGINE_RIGGING_{}_PT_panel"


class RiggingUIPanel(bpy.types.Panel):
    bl_label = "Rigging"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "MotionEngine"

    display_priority = 0

    def draw(self, context):
        layout = self.layout
        scene = context.scene
        properties = scene.motion_engine_ui_properties
        selected_axis = properties.me_ui_prop_rigging_avg_locked_axis
        ui_lock = global_vars.ui_lock_state
        selected_objects = context.selected_objects

        layout.enabled = not ui_lock

        box = layout.box()
        box = box.column(align=True)
        row = box.row()

        constraint_area_header = "Constraint tools"

        if context.mode == 'POSE':
            row.label(text=constraint_area_header, icon="CONSTRAINT_BONE")
        else:
            row.label(text=constraint_area_header, icon="CONSTRAINT")

        box.separator()

        row = box.row()

        row.label(text="Avg. location:")

        grid = box.row(align=True)

        row = grid.split(align=True)

        row.operator(f"motionengine.avgloc_c_s_to_a_{rigging_op_contexts[context.mode]}_operator",
                     icon="STICKY_UVS_DISABLE", text="")

        row.enabled = len(selected_objects) > 1

        for item in gen_new_types.items():
            row = grid.split(align=True)
            row.operator(f"motionengine.avgloc_c_s_to_{item[1]['idname']}_operator",
                         icon=item[1]["icon"], text="")
            row.enabled = len(selected_objects) > 0

        box.separator()
        row = box.row()

        row.label(text="Avg. damped track:")

        grid = box.row(align=True)

        grid.enabled = len(selected_objects) > 1

        for i in range(6):
            if i >= 3:
                axis = i - 3
                direction = 1
            else:
                axis = i
                direction = 0
            row = grid.split(align=True)
            row.operator("motionengine.avgdamped_c_s_to_a_{}_{}_operator"
                         .format(track_axis_button_lbl[axis].format(track_axis_dir_id[direction]).lower(),
                                 rigging_op_contexts[context.mode]),
                         text=track_axis_button_lbl[axis].format(track_axis_dir_lbl[direction]))

        box.separator()
        row = box.row()

        row.label(text="Avg. locked track:")

        grid = box.row(align=True)

        grid.enabled = len(selected_objects) > 1

        for i in range(6):
            if i >= 3:
                axis = i - 3
                direction = 1
            else:
                axis = i
                direction = 0
            row = grid.split(align=True)
            row.operator("motionengine.avglocked_c_s_to_a_{}_{}_operator"
                         .format(track_axis_button_lbl[axis].format(track_axis_dir_id[direction]).lower(),
                                 rigging_op_contexts[context.mode]),
                         text=track_axis_button_lbl[axis].format(track_axis_dir_lbl[direction]))

        grid = box.row(align=True)

        grid.enabled = len(selected_objects) > 1

        for i in range(3):
            row = grid.split(align=True)
            row.operator(f"motionengine.avglocked_axis_{track_axis_button_lbl[i].format('').lower()}_operator",
                         text=track_axis_button_lbl[i].format(''), depress=(i == selected_axis))

        box.separator()
        row = box.row()
        row.label(text="Ngon test:")
        grid = box.row(align=True)
        row = grid.split(align=True)
        row.operator("motionengine.gen_ngon_s_to_a_operator", text="NGon gen test")
        row.enabled = len(selected_objects) > 1


context_classes = []

for c in rigging_op_contexts.values():
    context_panel = type(
        f"RiggingUI{c.capitalize()}Panel",
        (RiggingUIPanel,),
        {
            'bl_idname': idname_base.format(c.upper()),
            'bl_context': c
        }
    )
    context_classes.append(context_panel)

CLASSES = [
    *context_classes
]
