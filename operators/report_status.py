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
from .. import global_vars


class ReportStatusOperator(bpy.types.Operator):
    """Report messages from global_vars.info_message"""
    bl_idname = "motionengine.report_status_operator"
    bl_label = "Report status"

    _timer = None

    # Will run until the ui is unlocked

    def modal(self, context, event):

        if not global_vars.ui_lock_state:
            self.cancel(context)
            return {'CANCELLED'}

        if event.type == 'TIMER':
            self.report({'INFO'}, global_vars.info_message.msg)

        return {'PASS_THROUGH'}

    def execute(self, context):
        wm = context.window_manager
        self._timer = wm.event_timer_add(0.1, window=context.window)
        wm.modal_handler_add(self)

        return {'RUNNING_MODAL'}

    def cancel(self, context):
        wm = context.window_manager
        wm.event_timer_remove(self._timer)


CLASSES = [
    ReportStatusOperator
]
