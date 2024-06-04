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
import mathutils

from .. import MotionEngine as me
from .. import global_vars
from .. import utils
from .. import pose_autogen_defs

track_axis = ['TRACK_X', 'TRACK_Y', 'TRACK_Z', 'TRACK_NEGATIVE_X', 'TRACK_NEGATIVE_Y', 'TRACK_NEGATIVE_Z']
lock_axis = ['LOCK_X', 'LOCK_Y', 'LOCK_Z']


def get_avg_pos(targets):
    avg_loc = mathutils.Vector()
    for t in targets:
        avg_loc += t.location
    return avg_loc / len(targets)


def joints_to_objs(joints, pose_objs):
    objs = []
    try:
        for j in joints:
            objs.append(pose_objs[j])
    except TypeError:
        objs.append(pose_objs[joints])
    return objs


def get_direction_vector(direction_str):
    direction = pose_autogen_defs.AXIS_DIRECTIONS.index(direction_str)
    if direction == 0:
        return mathutils.Vector((1, 0, 0))
    if direction == 1:
        return mathutils.Vector((0, 1, 0))
    if direction == 2:
        return mathutils.Vector((0, 0, 1))
    if direction == 3:
        return mathutils.Vector((-1, 0, 0))
    if direction == 4:
        return mathutils.Vector((0, -1, 0))
    if direction == 5:
        return mathutils.Vector((0, 0, -1))
    raise ValueError("Direction string did not match expected values")


def apply_copy_loc(src, target):
    num_prev = 0
    for c in src.constraints:
        if isinstance(c, bpy.types.CopyLocationConstraint):
            num_prev += 1
    constraint = src.constraints.new(type="COPY_LOCATION")
    constraint.name = "ME_Gen_CopyLoc"
    constraint.show_expanded = False
    constraint.target = target
    constraint.influence = 1 / (num_prev + 1)


def apply_damped_track(src, target, t_axis):
    num_prev = 0
    for c in src.constraints:
        if isinstance(c, bpy.types.DampedTrackConstraint):
            num_prev += 1
    constraint = src.constraints.new(type="DAMPED_TRACK")
    constraint.name = "ME_Gen_Damped"
    constraint.show_expanded = False
    constraint.track_axis = track_axis[pose_autogen_defs.AXIS_DIRECTIONS.index(t_axis)]
    constraint.target = target
    constraint.influence = 1 / (num_prev + 1)


def apply_locked_track(src, target, l_axis, t_axis):
    num_prev = 0
    for c in src.constraints:
        if isinstance(c, bpy.types.LockedTrackConstraint):
            num_prev += 1
    constraint = src.constraints.new(type="LOCKED_TRACK")
    constraint.name = "ME_Gen_Locked"
    constraint.show_expanded = False
    constraint.lock_axis = lock_axis[pose_autogen_defs.AXIS_LIST.index(l_axis)]
    constraint.track_axis = track_axis[pose_autogen_defs.AXIS_DIRECTIONS.index(t_axis)]
    constraint.target = target
    constraint.influence = 1 / (num_prev + 1)


def apply_copy_rot(src, target):
    constraint = src.constraints.new(type="COPY_ROTATION")
    constraint.name = "ME_Gen_CopyRot"
    constraint.show_expanded = False
    constraint.target = target


def apply_constraints(src, pose_objs, constraints):
    for c in constraints:
        if isinstance(c, int):
            apply_copy_loc(src, pose_objs[c])
        elif len(c) == 2:
            apply_damped_track(src, pose_objs[c[0]], c[1])
        elif len(c) == 3:
            apply_locked_track(src, pose_objs[c[0]], c[1], c[2])
        else:
            raise ValueError(f"Constraint definition '{c}' is invalid")


def recursive_build_armature(armature, pose_objs, current_tree, parent=None, last_tail=None, last_head=None):
    bone_name = current_tree['name']
    tail = get_avg_pos(joints_to_objs(current_tree['tail'], pose_objs))
    tail_direction = get_direction_vector(current_tree['tail_direction'])
    if 'head' in current_tree:
        head = get_avg_pos(joints_to_objs(current_tree['head'], pose_objs))
    else:
        head = last_tail
    if parent is None:
        bone = armature.edit_bones.new(bone_name)
        bone.head = mathutils.Vector((0, 0, head[2]))
    else:
        bone = armature.edit_bones.new(bone_name)
        bone.head = parent.tail
        bone.use_connect = True
    bone.parent = parent
    length = (head - tail).length
    offset = tail_direction * length
    if 'head_direction' in current_tree and last_head is not None:
        h_offset_direction = get_direction_vector(current_tree['head_direction'])
        head_offset = h_offset_direction * (head - last_head).length
        bone.use_connect = False
        bone.head = parent.head + head_offset
    bone.tail = bone.head + offset
    last_tail = tail
    last_head = head
    if 'children' in current_tree:
        for c in current_tree['children']:
            recursive_build_armature(armature, pose_objs, c, bone, last_tail, last_head)


def get_rot_src_obj(armature, bone):
    context = bpy.context
    scene = context.scene
    bone_length = (bone.head - bone.tail).length
    obj_name = bone.name + '.rot_src'
    obj = None
    for o in bpy.data.objects:
        if o.name == obj_name:
            obj = o
            break
    if obj is None:
        obj = bpy.data.objects.new(obj_name, None)
    obj.data = None
    obj.empty_display_type = 'ARROWS'
    obj.empty_display_size = bone_length * 0.3
    if obj.name in scene.objects:
        return obj
    collection = global_vars.resolve_collection_path(['MotionEngine', 'Armatures', armature.name], context)
    collection.objects.link(obj)
    return obj


def recursive_add_constraints(armature, pose_objs, current_tree):
    bone_name = current_tree['name']
    pose_bone = get_armature_obj(armature).pose.bones.get(bone_name)
    if 'rotation' in current_tree:
        obj = get_rot_src_obj(armature, pose_bone)
        for c in obj.constraints:
            obj.constraints.remove(c)
        apply_constraints(obj, pose_objs, current_tree['rotation'])
        apply_copy_rot(pose_bone, obj)
    if 'constraints' in current_tree:
        apply_constraints(pose_bone, pose_objs, current_tree['constraints'])
    if 'children' in current_tree:
        for c in current_tree['children']:
            recursive_add_constraints(armature, pose_objs, c)


def get_armature_obj(armature):
    context = bpy.context
    scene = context.scene
    obj = None
    for o in bpy.data.objects:
        if o.data == armature:
            obj = o
            break
    if obj is None:
        obj = bpy.data.objects.new(armature.name, armature)
    if obj.name in scene.objects:
        return obj
    collection = global_vars.resolve_collection_path(['MotionEngine', 'Armatures', armature.name], context)
    collection.objects.link(obj)
    return obj


def armature_mode_set(armature, mode):
    context = bpy.context
    armature_obj = get_armature_obj(armature)
    for obj in context.selected_objects:
        obj.select_set(False)
    context.view_layer.objects.active = armature_obj
    armature_obj.select_set(True)
    bpy.ops.object.mode_set(mode=mode)


def remove_lr_from_name(name):
    split_name = name.split('.')
    if len(split_name) < 2:
        return name
    return '.'.join(split_name[:-1])


def has_suffix(name, suffix):
    split_name = name.split('.')
    if len(split_name) < 2:
        return False
    n_suffix = split_name[-1].lower()
    if n_suffix == suffix.lower():
        return True
    return False


def get_symmetrical_pose_bones(armature):
    armature_obj = get_armature_obj(armature)
    pose_bones = armature_obj.pose.bones
    left_bones = {remove_lr_from_name(bone.name): bone for bone in pose_bones if has_suffix(bone.name, 'l')}
    right_bones = {remove_lr_from_name(bone.name): bone for bone in pose_bones if has_suffix(bone.name, 'r')}
    return [(left_bones[name], right_bones[name]) for name in left_bones if name in right_bones]


def clear_armature_constraints(armature):
    armature_obj = get_armature_obj(armature)
    pose_bones = armature_obj.pose.bones
    for b in pose_bones:
        for c in b.constraints:
            b.constraints.remove(c)


class GenerateArmatureOperator(bpy.types.Operator):
    """Generate armature for this pose"""
    bl_idname = "motionengine.generate_armature_operator"
    bl_label = "Generate Armature"
    bl_options = {'REGISTER', 'UNDO'}

    @classmethod
    def poll(cls, context):
        active_obj = context.active_object
        if active_obj is None:
            return False
        req_attr = ['pose_source', 'pose_name', 'joint_id', 'cam_solution_id']
        return (all(attr in active_obj.keys() for attr in req_attr) and
                hasattr(pose_autogen_defs, active_obj['pose_source']))

    def execute(self, context):
        scene = context.scene
        active_obj = context.active_object
        pose_source = active_obj['pose_source']
        pose_name = active_obj['pose_name']
        cam_solution_id = active_obj['cam_solution_id']
        pose_objs = {obj['joint_id']: obj for obj in scene.objects if
                     obj.get('pose_source', None) == pose_source and
                     obj.get('pose_name', None) == pose_name and
                     obj.get('cam_solution_id', None) == cam_solution_id and
                     obj.get('joint_id', None) is not None}

        base_tree = getattr(pose_autogen_defs, pose_source)

        gen_name = f'{pose_name}.{pose_source}'

        armature = None

        for a in bpy.data.armatures:
            if a.name == gen_name:
                armature = a
                break

        if armature is None:
            armature = bpy.data.armatures.new(gen_name)

        armature_obj = get_armature_obj(armature)

        if armature_obj.animation_data is not None:
            old_action = armature_obj.animation_data.action
            bpy.data.actions.remove(old_action)
            armature_obj.animation_data_clear()

        armature_mode_set(armature, 'EDIT')

        for bone in armature.edit_bones:
            armature.edit_bones.remove(bone)

        recursive_build_armature(armature, pose_objs, base_tree)

        bpy.ops.armature.calculate_roll(type='GLOBAL_POS_X')

        armature_mode_set(armature, 'POSE')

        clear_armature_constraints(armature)

        for bone in get_armature_obj(armature).pose.bones:
            bone.scale = (1, 1, 1)

        symmetrical_bones = get_symmetrical_pose_bones(armature)

        for left_bone, right_bone in symmetrical_bones:
            context.view_layer.update()
            l_bone_length = left_bone.length
            r_bone_length = right_bone.length
            avg_length = (l_bone_length + r_bone_length) / 2
            l_bone_adj = avg_length / l_bone_length
            r_bone_adj = avg_length / r_bone_length
            left_bone.scale = (l_bone_adj, l_bone_adj, l_bone_adj)
            right_bone.scale = (r_bone_adj, r_bone_adj, r_bone_adj)

        bpy.ops.pose.armature_apply(selected=False)

        context.view_layer.update()

        for left_bone, right_bone in symmetrical_bones:
            armature_mode_set(armature, 'EDIT')
            l_bone_edit = armature.edit_bones[left_bone.name]
            r_bone_edit = armature.edit_bones[right_bone.name]
            armature_mode_set(armature, 'POSE')
            context.view_layer.update()
            if l_bone_edit.use_connect or r_bone_edit.use_connect:
                continue
            loc_lb = left_bone.matrix.to_translation()
            loc_rb = right_bone.matrix.to_translation()
            avg_loc = (loc_lb + mathutils.Vector((-loc_rb[0], loc_rb[1], loc_rb[2]))) / 2
            left_bone.matrix.translation = avg_loc
            right_bone.matrix.translation = (-avg_loc[0], avg_loc[1], avg_loc[2])

        bpy.ops.pose.armature_apply(selected=False)

        context.view_layer.update()

        recursive_add_constraints(armature, pose_objs, base_tree)

        armature_mode_set(armature, 'EDIT')
        for bone in armature.edit_bones:
            bone.inherit_scale = 'NONE'

        armature_mode_set(armature, 'OBJECT')

        armature_obj['pose_source'] = pose_source
        armature_obj['pose_name'] = pose_name
        armature_obj['cam_solution_id'] = cam_solution_id

        self.report({'INFO'}, f"Generated armature for pose '{gen_name}'")

        return {'FINISHED'}


class BakeAnimationOperator(bpy.types.Operator):
    """Bake animation for this armature"""
    bl_idname = "motionengine.bake_animation_operator"
    bl_label = "Bake Animation"
    bl_options = {'REGISTER', 'UNDO'}

    @classmethod
    def poll(cls, context):
        active_obj = context.active_object
        if active_obj is None:
            return False
        req_attr = ['pose_source', 'pose_name', 'cam_solution_id']
        return (all(attr in active_obj.keys() for attr in req_attr) and
                active_obj.type == 'ARMATURE')

    def execute(self, context):
        scene = context.scene
        active_obj = context.active_object
        pose_source = active_obj['pose_source']
        pose_name = active_obj['pose_name']
        cam_solution_id = active_obj['cam_solution_id']

        armature = active_obj.data
        armature_mode_set(armature, 'POSE')

        context.view_layer.update()

        bpy.ops.nla.bake(
            frame_start=scene.frame_start,
            frame_end=scene.frame_end,
            only_selected=False,
            visual_keying=True,
            clear_constraints=True,
            use_current_action=True,
            clean_curves=True,
            channel_types={'LOCATION', 'ROTATION'}
        )

        current_frame = scene.frame_current

        scene.frame_current = scene.frame_start

        bpy.ops.pose.select_all(action='SELECT')
        bpy.ops.pose.transforms_clear()
        bpy.ops.anim.keyframe_insert_by_name(type='BUILTIN_KSI_LocRot')

        scene.frame_set(current_frame)

        armature_mode_set(armature, 'OBJECT')

        self.report({'INFO'}, f"Baked animation to '{active_obj.animation_data.action.name}'")

        return {'FINISHED'}


CLASSES = [
    GenerateArmatureOperator,
    BakeAnimationOperator
]
