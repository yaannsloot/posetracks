'''
Copyright (C) 2023 Ian Sloat

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

me_detectpose_model = None
"""Abstract pose detection model for use with other functions"""

clip_tracker = None
"""Used to keep track of the last active clip during certain operator calls"""

properties_tracker = None
"""Used with write data callback to keep track of the run data property in the current scene"""

stats_tracker = None
"""Used with write data callback to keep track of the run statistics property in the current scene"""

context_tracker = None
"""Used with certain callback functions to maintain correct context"""

warmup_state = False
"""Flag that indicates whether models are currently in a warm-up state"""

ui_lock_state = False
"""Disables all ui elements when false"""

analysis_data = None
"""Used on async analysis functions to store resulting data"""


class InfoMessage:
    """Modifiable container for python bindings"""
    msg: ""


info_message = InfoMessage()
"""String used in the report status operator"""
