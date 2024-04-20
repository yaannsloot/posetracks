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

pose_types = {'wb17', 'wb26', 'wb133', 'f106', 'h21', 'a17'}
"""
All valid pose types. Used for track identification. 
More can be added during initialization for additional pose models.
"""

"""finish this"""
joint_groups = {
    'wb17': {
        'head': {(0, 4)},
        'arms': {
            'left': {5, 7, 9},
            'right': {6, 8, 10}
        },
        'legs': {
            'left': {11, 13, 15},
            'right': {12, 14, 16}
        }
    },
    'wb26': {
        'head': {(0, 5), 17},
        'arms': {
            'left': {5, 7, 9},
            'right': {6, 8, 10}
        },
        'legs': {
            'left': {11, 13, 15},
            'right': {12, 14, 16}
        },
        'torso': {18, 19},
        'feet': {
            'left': {20, 22, 24},
            'right': {21, 23, 25}
        }
    },
    'wb133': {
        'head': {(0, 4)},
        'arms': {
            'left': {5, 7, 9},
            'right': {6, 8, 10}
        },
        'legs': {
            'left': {11, 13, 15},
            'right': {12, 14, 16}
        }
    }
}


class InfoMessage:
    """Modifiable container for python bindings"""
    msg: ""


info_message = InfoMessage()
"""String used in the report status operator"""


def resolve_collection_path(path, context, make_collections=True):
    scene = context.scene
    scene_root = scene.collection
    current = scene_root
    for c in path:
        if current.children.get(c) is None and not make_collections:
            return None
        collection = bpy.data.collections.get(c)
        if collection is None:
            collection = bpy.data.collections.new(c)
        if c in current.children:
            current = collection
        else:
            current.children.link(collection)
            current = collection
    return current


def unlink_from_all_collections(obj):
    for collection in bpy.data.collections:
        if obj.name in collection.objects:
            collection.objects.unlink(obj)


executor = None

shutdown_state = False
