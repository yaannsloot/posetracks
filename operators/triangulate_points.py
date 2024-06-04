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
from .. import utils


def prepare_empty(context, empty_name, collection_path):
    empty = None
    for obj in bpy.data.objects:
        if obj.name == empty_name:
            empty = obj
            break
    if empty is None:
        empty = bpy.data.objects.new(empty_name, None)
        empty.empty_display_type = 'SPHERE'
        empty.empty_display_size = 0.1
        collection = global_vars.resolve_collection_path(collection_path, context)
        collection.objects.link(empty)
    action = bpy.data.actions.get(f'{empty_name}_Action', None)
    if action is not None:
        bpy.data.actions.remove(action)
    action = bpy.data.actions.new(f'{empty_name}_Action')
    empty.animation_data_clear()
    empty.animation_data_create()
    empty.animation_data.action = action
    empty.parent = None
    return empty


def write_anim_location(empty, frame, point):
    empty_anim_data = empty.animation_data
    empty_fcurves = empty_anim_data.action.fcurves
    for i in range(3):
        fcurve = empty_fcurves.find('location', index=i)
        if fcurve is None:
            fcurve = empty_fcurves.new('location', index=i)
        keys = fcurve.keyframe_points
        y = point[i]
        if i > 0:
            y *= -1
        keys.add(1)
        key = keys[-1]
        key.co = (frame, y)


class TriangulatePointsOperator(bpy.types.Operator):
    """Triangulate tracked points and place them in the current scene"""
    bl_idname = "motionengine.triangulate_points_operator"
    bl_label = "Triangulate points"
    bl_options = {'REGISTER', 'UNDO'}

    @classmethod
    def poll(cls, context):
        scene = context.scene
        properties = scene.motion_engine_ui_properties
        active_clip = properties.me_ui_prop_anchor_cam_selection
        return not global_vars.ui_lock_state and active_clip is not None

    def execute(self, context):

        scene = context.scene
        properties = scene.motion_engine_ui_properties

        active_clip = properties.me_ui_prop_anchor_cam_selection

        anchor_cam = None

        for obj in scene.objects:
            if obj.type == 'CAMERA' and obj.data.name == active_clip.name:
                anchor_cam = obj
                break

        if anchor_cam is None:
            self.report({'ERROR'}, f"No matching cameras were found for anchor '{active_clip.name}'")
            return {'FINISHED'}

        solution_id = anchor_cam.data.get('solution_id', None)

        if solution_id is None:
            self.report({'ERROR'}, "Anchor camera has no solution id!")
            return {'FINISHED'}

        other_cams = [cam for cam in scene.objects if cam != anchor_cam and cam.type == 'CAMERA' and
                      cam.data.name in bpy.data.movieclips and
                      cam.data.get('solution_id', None) == solution_id]

        other_clips = [bpy.data.movieclips[cam.data.name] for cam in other_cams]

        clips = [active_clip] + other_clips

        all_cams = [anchor_cam] + other_cams

        cam_matrix_worlds = [cam.matrix_world.normalized() for cam in all_cams]

        flip_mtx = [
            [1, -1, -1, 1],
            [-1, 1, 1, -1],
            [-1, 1, 1, -1],
            [1, 1, 1, 1]
        ]

        cam_Rt = []

        for i in range(len(cam_matrix_worlds)):
            mat4x4 = me.tracking.Mat4x4()
            for a in range(4):
                for b in range(4):
                    mat4x4[a, b] = cam_matrix_worlds[i][a][b] * flip_mtx[a][b]
            rt = me.tracking.Rt()
            rt.from4x4(mat4x4)
            rt.invert()
            cam_Rt.append(rt)

        t_data = []
        cam_Kk = []

        for clip in clips:
            t_data.append(utils.get_clip_tracking_data(clip, filter_locked=True, pose_joint_conf=0))
            cam_Kk.append(utils.get_clip_Kk(clip))

        t_data3d = me.tracking.triangulate_static(t_data, cam_Kk, cam_Rt)

        poses = t_data3d.poses
        detections = t_data3d.detections
        tags = t_data3d.tags

        written_objs = {}

        for frame, pose_dict in poses.items():
            for pose_id, joint_dict in pose_dict.items():
                for joint_id, joint in joint_dict.items():
                    empty_name = f'{pose_id}.{joint_id}'
                    empty = written_objs.get(empty_name, None)
                    if empty is None:
                        empty = prepare_empty(context, empty_name,
                                              ['MotionEngine', 'Tracking', 'Poses', pose_id])
                        written_objs[empty_name] = empty
                        pose_id_split = pose_id.split('.')
                        empty['pose_source'] = pose_id_split[-1]
                        empty['pose_name'] = '.'.join(pose_id_split[:-1])
                        empty['joint_id'] = joint_id
                        empty['cam_solution_id'] = solution_id
                    write_anim_location(empty, frame, joint)

        for frame, det_dict in detections.items():
            for det_name, (class_id, point) in det_dict.items():
                empty = written_objs.get(det_name, None)
                if empty is None:
                    empty = prepare_empty(context, det_name,
                                          ['MotionEngine', 'Tracking', 'Detections'])
                    written_objs[det_name] = empty
                    empty['cam_solution_id'] = solution_id
                write_anim_location(empty, frame, point)

        for o_name, obj in written_objs.items():
            anim_data = obj.animation_data
            if anim_data is None:
                continue
            anim_data = anim_data.action
            if anim_data is None:
                continue
            fcurves = anim_data.fcurves
            for i in range(3):
                fcurve = fcurves.find('location', index=i)
                if fcurve is None:
                    continue
                points = fcurve.keyframe_points
                points.sort()
                points.deduplicate()
                fcurve.update()

        for obj in context.selected_objects:
            obj.select_set(False)
        for obj in written_objs.values():
            obj.select_set(True)
        context.view_layer.objects.active = anchor_cam
        bpy.ops.object.parent_set(type='OBJECT', keep_transform=True)

        return {"FINISHED"}


CLASSES = [
    TriangulatePointsOperator
]
