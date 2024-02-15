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

import sys
import bpy
import math
import mathutils
import itertools
from .. import global_vars

contexts = {
    "OBJECT": "objectmode",
    "POSE": "posemode",
}

gen_ops = []


def avg_loc_s_to_a(self, context):
    if not global_vars.ui_lock_state:
        active_object = context.active_object

        selected_objects = [obj for obj in context.selected_objects if obj != active_object]

        prev_gen = 0
        for c in active_object.constraints:
            if c.name.startswith("ME_Gen_CopyLoc"):
                prev_gen += 1

        for i, obj in enumerate(selected_objects):
            constraint = active_object.constraints.new(type="COPY_LOCATION")
            constraint.name = "ME_Gen_CopyLoc"
            constraint.show_expanded = False
            constraint.target = obj
            constraint.influence = 1 / (i + 1 + prev_gen)

    return {"FINISHED"}


for c in contexts.values():
    new_op = type(
        f"AvgLocSelectedToActive{c.capitalize()}Operator",
        (bpy.types.Operator,),
        {
            '__doc__': f"Generate constraints for the active {'bone' if c == 'posemode' else 'object'}",
            'bl_idname': f"motionengine.avgloc_c_s_to_a_{c.lower()}_operator",
            'bl_label': 'Selected to active',
            'execute': lambda self, context: avg_loc_s_to_a(self, context)
        }
    )
    gen_ops.append(new_op)

gen_new_types = {
    "PLAIN_AXES": {"class_name": "Axes", "idname": "axes", "desc": "plain axes", "icon": "EMPTY_AXIS"},
    "ARROWS": {"class_name": "Arrows", "idname": "arrows", "desc": "arrows", "icon": "EMPTY_ARROWS"},
    "SPHERE": {"class_name": "Sphere", "idname": "sphere", "desc": "sphere", "icon": "SPHERE"}
}


def avgloc_gen_new(self, context):
    if global_vars.ui_lock_state:
        return {"FINISHED"}

    selected_objects = context.selected_objects

    if len(selected_objects) > 0:
        new_empty = bpy.data.objects.new("ME_Gen_Empty", None)
        new_empty.empty_display_type = self.display_type

        sum_scales = 0

        for obj in selected_objects:
            if obj.type is None:
                scale = obj.empty_display_size
            else:
                scales = obj.scale
                scale = (scales[0] + scales[1] + scales[2]) / 3
            sum_scales = sum_scales + scale

        new_empty.empty_display_size = sum_scales / len(selected_objects)

        for i, obj in enumerate(selected_objects):
            constraint = new_empty.constraints.new(type="COPY_LOCATION")
            constraint.name = "ME_Gen_CopyLoc"
            constraint.show_expanded = False
            constraint.target = obj
            constraint.influence = 1 / (i + 1)

        dest_path = global_vars.resolve_collection_path(["MotionEngine", "Rigging"], context)
        dest_path.objects.link(new_empty)

    return {"FINISHED"}


for item in gen_new_types.items():
    new_op = type(
        f"AvgLocNewFromSelected{item[1]['class_name']}Operator",
        (bpy.types.Operator,),
        {
            '__doc__': "Generate constraints for a new empty object",
            'bl_idname': f"motionengine.avgloc_c_s_to_{item[1]['idname']}_operator",
            'bl_label': f"Create new {item[1]['desc']} empty",
            'display_type': item[0],
            'execute': lambda self, context: avgloc_gen_new(self, context)
        }
    )
    gen_ops.append(new_op)

track_axis_list = ['TRACK_{}X', 'TRACK_{}Y', 'TRACK_{}Z']
track_axis_direction = ['', 'NEGATIVE_']
track_axis_button_lbl = ['{}X', '{}Y', '{}Z']
track_axis_dir_lbl = ['+', '-']
track_axis_dir_id = ['p', 'n']


def avgdamped_gen_new(self, context):
    if not global_vars.ui_lock_state:
        active_object = context.active_object

        selected_objects = [obj for obj in context.selected_objects if obj != active_object]

        prev_gen = 0
        for c in active_object.constraints:
            if c.name.startswith("ME_Gen_Damped"):
                prev_gen += 1

        for i, obj in enumerate(selected_objects):
            constraint = active_object.constraints.new(type="DAMPED_TRACK")
            constraint.name = "ME_Gen_Damped"
            constraint.show_expanded = False
            constraint.track_axis = self.track_axis
            constraint.target = obj

            constraint.influence = 1 / (i + 1 + prev_gen)

    return {"FINISHED"}


def avglocked_gen_new(self, context):
    if not global_vars.ui_lock_state:
        scene = context.scene
        properties = scene.motion_engine_ui_properties
        ui_locked_axis = properties.me_ui_prop_rigging_avg_locked_axis
        lock_axis = track_axis_button_lbl[ui_locked_axis].format('LOCK_')

        active_object = context.active_object

        selected_objects = [obj for obj in context.selected_objects if obj != active_object]

        prev_gen = 0
        for c in active_object.constraints:
            if c.name.startswith("ME_Gen_Locked"):
                prev_gen += 1

        for i, obj in enumerate(selected_objects):
            constraint = active_object.constraints.new(type="LOCKED_TRACK")
            constraint.name = "ME_Gen_Locked"
            constraint.show_expanded = False
            constraint.track_axis = self.track_axis
            constraint.lock_axis = lock_axis
            constraint.target = obj

            constraint.influence = 1 / (i + 1 + prev_gen)

    return {"FINISHED"}


def set_lock_axis(self, context):
    if not global_vars.ui_lock_state:
        scene = context.scene
        properties = scene.motion_engine_ui_properties
        properties.me_ui_prop_rigging_avg_locked_axis = self.lock_axis
    return {'FINISHED'}


for i_ in range(6):
    if i_ >= 3:
        axis = i_ - 3
        direction = 1
    else:
        axis = i_
        direction = 0
    for c in contexts.values():
        idname = ("motionengine.avgdamped_c_s_to_a_{}_{}_operator"
                  .format(track_axis_button_lbl[axis].format(track_axis_dir_id[direction]).lower(),
                          c.lower()))
        label = f"Selected to active: {track_axis_button_lbl[axis].format(track_axis_dir_lbl[direction])}"
        new_op_a = type(
            f"AvgDampedOperator{track_axis_button_lbl[axis].format(track_axis_dir_id[direction]).upper()}"
            f"{c.capitalize()}",
            (bpy.types.Operator,),
            {
                '__doc__': label + f"\nGenerate constraints for the active {'bone' if c == 'posemode' else 'object'}",
                'bl_idname': idname,
                'bl_label': label,
                'track_axis': track_axis_list[axis].format(track_axis_direction[direction]),
                'execute': lambda self, context: avgdamped_gen_new(self, context)
            }
        )
        idname = ("motionengine.avglocked_c_s_to_a_{}_{}_operator"
                  .format(track_axis_button_lbl[axis].format(track_axis_dir_id[direction]).lower(),
                          c.lower()))
        new_op_b = type(
            f"AvgLockedOperator{track_axis_button_lbl[axis].format(track_axis_dir_id[direction]).upper()}"
            f"{c.capitalize()}",
            (bpy.types.Operator,),
            {
                '__doc__': label + f"\nGenerate constraints for the active {'bone' if c == 'posemode' else 'object'}",
                'bl_idname': idname,
                'bl_label': label,
                'track_axis': track_axis_list[axis].format(track_axis_direction[direction]),
                'execute': lambda self, context: avglocked_gen_new(self, context)
            }
        )
        gen_ops.append(new_op_a)
        gen_ops.append(new_op_b)
    if i_ < 3:
        idname = f"motionengine.avglocked_axis_{track_axis_button_lbl[axis].format('').lower()}_operator"
        label = f"Axis that points upward: {track_axis_button_lbl[axis].format('')}"
        new_op_c = type(
            f"AvgLockedAxisOperator{track_axis_button_lbl[axis].format('')}",
            (bpy.types.Operator,),
            {
                '__doc__': label,
                'bl_idname': idname,
                'bl_label': label,
                'lock_axis': i_,
                'execute': lambda self, context: set_lock_axis(self, context)
            }
        )
        gen_ops.append(new_op_c)


def clamp(n, minn, maxn):
    return max(min(maxn, n), minn)


def calc_angle(a: mathutils.Vector, b: mathutils.Vector, c: mathutils.Vector):
    ab = a - b
    bc = c - b
    dot = ab.dot(bc)
    abbc = ab.magnitude * bc.magnitude
    if abbc < sys.float_info.epsilon:
        abbc = sys.float_info.epsilon
    return math.degrees(math.acos(clamp(dot / abbc, -1, 1)))


def normalize(min_value, max_value, value):
    r = max_value - min_value
    if r == 0:
        r = sys.float_info.epsilon
    normalized_value = (value - min_value) / r
    return clamp(normalized_value, 0, 1)


class GenNgonSelectedToActiveOperator(bpy.types.Operator):
    """Generate n-gon using the active object as a starting vert"""
    bl_idname = "motionengine.gen_ngon_s_to_a_operator"
    bl_label = "Selected to active"

    # CURRENT IDEA:
    # Shortest distance and widest angle. This will be achieved by repeatedly
    # choosing the point with the minimum di / dmax + 1 - Oi / 180 where d is distance and O is the angle
    # between the last edge and the next edge that would be created. Edge candidates will be placed in a
    # list and sorted in ascending order, with the first being chosen. When a point is chosen, it is removed
    # from the list of candidates.

    def execute(self, context):
        if not global_vars.ui_lock_state:

            active_object = context.active_object

            if active_object not in context.selected_objects:
                active_object = context.selected_objects[0]

            debug = False

            solver_weight = 0.66
            dist_weight = 1 - solver_weight

            selected_objects = [obj for obj in context.selected_objects if obj != active_object]

            points = [active_object.matrix_world.decompose()[0],
                      *[obj.matrix_world.decompose()[0] for obj in selected_objects]]

            all_objs = [active_object, *selected_objects]

            mesh_data = bpy.data.meshes.new("ME_Gen_Mesh")

            obj = bpy.data.objects.new("ME_Gen_Loop", mesh_data)
            norm_obj = bpy.data.objects.new("ME_Gen_Norm", None)
            norm_obj.empty_display_type = 'SINGLE_ARROW'

            centroid = mathutils.Vector()
            for point in points:
                centroid += point
            centroid /= len(points)

            norm_display_scale = 0
            for point in points:
                norm_display_scale += (centroid - point).magnitude
            norm_display_scale /= len(points)

            norm_obj.empty_display_size = norm_display_scale

            anchor_obj = bpy.data.objects.new("ME_Gen_Anchor", None)
            anchor_obj.empty_display_type = 'PLAIN_AXES'
            anchor_obj.empty_display_size = norm_display_scale / 10

            for i, o in enumerate(all_objs):
                constraint = anchor_obj.constraints.new(type="COPY_LOCATION")
                constraint.name = "ME_Gen_CopyLoc"
                constraint.show_expanded = False
                constraint.target = o
                constraint.influence = 1 / (i + 1)

            constraint = norm_obj.constraints.new(type="COPY_LOCATION")
            constraint.name = "ME_Gen_CopyLoc"
            constraint.show_expanded = False
            constraint.target = anchor_obj
            constraint = norm_obj.constraints.new(type="SHRINKWRAP")
            constraint.name = "ME_Gen_Shrinkwrap"
            constraint.show_expanded = False
            constraint.target = obj
            constraint.use_track_normal = True
            constraint.track_axis = 'TRACK_Z'
            constraint = norm_obj.constraints.new(type="COPY_LOCATION")
            constraint.name = "ME_Gen_CopyLoc"
            constraint.show_expanded = False
            constraint.target = anchor_obj

            collection_path = ["MotionEngine", "Rigging", norm_obj.name]
            dest_collection = global_vars.resolve_collection_path(collection_path, context)
            dest_collection.objects.link(norm_obj)
            dest_collection.objects.link(anchor_obj)
            dest_collection.objects.link(obj)

            vertices = points
            edges = []
            faces = []

            indices = list(range(1, len(points)))
            last = 0

            # If the number of vertices exceeds 3, this can help stabilize the starting trace
            if len(points) > 3:
                adjacent = list(itertools.combinations(list(range(1, len(points))), 2))
                b = points[0]
                max_dist = 0
                angles = []
                dists = []
                for adj in adjacent:
                    a = points[adj[0]]
                    c = points[adj[1]]
                    angle = calc_angle(a, b, c)
                    dist = (b - a).magnitude + (b - c).magnitude
                    if max_dist < dist:
                        max_dist = dist
                    dists.append(dist)
                    angles.append(angle)
                costs = []
                for i in range(len(dists)):
                    adj = adjacent[i]
                    dist = normalize(0, max_dist, dists[i])
                    angle = 1 - normalize(0, 180, angles[i])
                    score = dist * dist_weight + angle * solver_weight
                    costs.append((score, adj))
                costs.sort(key=lambda a: a[0])
                choice = costs[0]
                adj = choice[1]
                rev = (b - points[adj[0]]).magnitude > (b - points[adj[1]]).magnitude
                if rev:
                    edges.append((adj[0], 0))
                    edges.append((0, adj[1]))
                else:
                    edges.append((adj[1], 0))
                    edges.append((0, adj[0]))
                indices.remove(adj[0])
                indices.remove(adj[1])
                last = adj[1] if rev else adj[0]

            # Main quick path loop
            while len(indices) > 0:
                last_vert = vertices[last]
                dists = []
                angles = []
                max_dist = 0
                init_dist = True
                with_angle = False
                for i in indices:
                    next_vert = vertices[i]
                    dist = (last_vert - next_vert).magnitude
                    dists.append(dist)
                    if init_dist:
                        max_dist = dist
                        init_dist = False
                    else:
                        if dist > max_dist:
                            max_dist = dist
                    if len(edges) > 0:
                        first_vert = vertices[edges[-1][0]]
                        angle = calc_angle(first_vert, last_vert, next_vert)
                        angles.append(angle)
                        with_angle = True
                scores = []
                for j in range(len(dists)):
                    score = normalize(0, max_dist, dists[j])
                    if with_angle:
                        score *= dist_weight
                        angle_score = 1 - normalize(0, 180, angles[j])
                        angle_score *= solver_weight
                        score += angle_score
                        scores.append((indices[j], score, angle_score))
                    else:
                        scores.append((indices[j], score))
                scores.sort(key=lambda a: a[1])
                if debug:
                    print("Step")
                    for score in scores:
                        print(vertices[score[0]], score[1],
                              score[1] - score[2] if len(score) > 2 else 'N/A',
                              score[2] if len(score) > 2 else 'N/A'
                              )
                edges.append((last, scores[0][0]))
                last = scores[0][0]
                indices.remove(last)

            if len(vertices) > 2:
                edges.append((last, edges[0][0]))

            mesh_data.from_pydata(vertices, edges, faces)

            mesh_data.update()

            for v in range(len(all_objs)):
                v_group = obj.vertex_groups.new(name="ME_Gen_Hook")
                v_group.add([v], 1, 'REPLACE')
                mod = obj.modifiers.new(name="ME_Gen_Hook", type="HOOK")
                mod.object = all_objs[v]
                mod.vertex_group = v_group.name
                mod.falloff_type = 'CONSTANT'
                mod.show_expanded = False

            bpy.ops.object.select_all(action='DESELECT')
            obj.select_set(True)
            context.view_layer.objects.active = obj

            obj.display_type = 'WIRE'
            current_mode = context.object.mode
            bpy.ops.object.mode_set(mode='EDIT')
            bpy.ops.mesh.select_all(action='SELECT')
            bpy.ops.mesh.fill_holes(sides=0)
            bpy.ops.object.mode_set(mode=current_mode)

        return {"FINISHED"}


CLASSES = [
    *gen_ops,
    GenNgonSelectedToActiveOperator
]
