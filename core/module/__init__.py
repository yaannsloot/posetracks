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

import os
import sys
import importlib
import json

# Temporarily add module directory to environment
old_path = os.environ.get('PATH', '')
init_path = os.path.dirname(os.path.abspath(__file__))
dll_path = os.path.join(init_path, "bin")
os.environ['PATH'] = init_path + os.pathsep + dll_path + os.pathsep + old_path
os.add_dll_directory(init_path)
os.add_dll_directory(dll_path)

# Get the current Python version
python_version = f"cp{sys.version_info.major}{sys.version_info.minor}"

# Get the directory of the correct version of the module
module_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), python_version)
mod_py_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "python")

# Add the module directory to sys.path
sys.path.insert(0, module_dir)
sys.path.insert(0, mod_py_dir)

# Compiled modules
MEPython = importlib.import_module("MEPython")

# Remove the module directory from sys.path
sys.path.pop(0)
sys.path.pop(0)

model_dir = os.path.join(init_path, "models")


# General functions, classes, and constants

def model_path(model_file):
    return os.path.join(model_dir, model_file)


def dnn_module(module_name):
    if not module_name.endswith(".dll"):
        module_name += ".dll"
    return os.path.join(init_path, module_name)


class PoseCollection:
    def __init__(self):
        self.poses = {}

    def add_pose(self, pose_id: int):
        self.poses[pose_id] = MEPython.dnn.Pose()

    def set_pose(self, pose_id: int, pose: MEPython.dnn.Pose):
        self.poses[pose_id] = pose

    def get_pose(self, pose_id: int):
        return self.poses[pose_id]

    def has_pose(self, pose_id: int):
        return pose_id in self.poses

    def get_pose_ids(self):
        return list(self.poses.keys())

    def num_poses(self):
        return len(self.poses)

    def __getitem__(self, item: int):
        return self.poses[item]


MEPython.dnn.PoseCollection = PoseCollection


class PoseFrames:
    def __init__(self):
        self.pose_frames = {}

    def add_pose_frame(self, frame_id: int):
        self.pose_frames[frame_id] = PoseCollection()

    def set_pose_frame(self, frame_id: int, pose_frame: PoseCollection):
        self.pose_frames[frame_id] = pose_frame

    def get_pose_frame(self, frame_id: int):
        return self.pose_frames[frame_id]

    def has_pose_frame(self, frame_id: int):
        return frame_id in self.pose_frames

    def get_pose_frame_ids(self):
        return list(self.pose_frames.keys())

    def num_pose_frames(self):
        return len(self.pose_frames)

    def __getitem__(self, item: int):
        return self.pose_frames[item]


MEPython.dnn.PoseFrames = PoseFrames


def pose_common_points(pose1: MEPython.dnn.Pose, pose2: MEPython.dnn.Pose, threshold):
    points1 = []
    points2 = []
    pose1_joint_ids = pose1.get_joint_ids()
    pose2_joint_ids = pose2.get_joint_ids()
    for joint_id in pose1_joint_ids:
        if joint_id in pose2_joint_ids:
            joint1 = pose1.get_joint(joint_id)
            joint2 = pose2.get_joint(joint_id)
            if joint1.prob > threshold and joint2.prob > threshold:
                points1.append((joint1.pt.x, joint1.pt.y))
                points2.append((joint2.pt.x, joint2.pt.y))
    return points1, points2


def pose_collection_common_points(pose_collection1: PoseCollection, pose_collection2: PoseCollection, threshold):
    points1 = []
    points2 = []
    pose1_ids = pose_collection1.get_pose_ids()
    pose2_ids = pose_collection2.get_pose_ids()
    for pose_id in pose1_ids:
        if pose_id in pose2_ids:
            pose1 = pose_collection1.get_pose(pose_id)
            pose2 = pose_collection2.get_pose(pose_id)
            p1, p2 = pose_common_points(pose1, pose2, threshold)
            points1.extend(p1)
            points2.extend(p2)
    return points1, points2


def pose_frames_common_points(pose_frames1: PoseFrames, pose_frames2: PoseFrames, threshold):
    points1 = []
    points2 = []
    frame_ids1 = pose_frames1.get_pose_frame_ids()
    frame_ids2 = pose_frames2.get_pose_frame_ids()
    for frame_id in frame_ids1:
        if frame_id in frame_ids2:
            pose_collection1 = pose_frames1.get_pose_frame(frame_id)
            pose_collection2 = pose_frames2.get_pose_frame(frame_id)
            p1, p2 = pose_collection_common_points(pose_collection1, pose_collection2, threshold)
            points1.extend(p1)
            points2.extend(p2)
    return points1, points2


# Blender functions and classes
try:
    import bpy


    class MotionEngineProperties(bpy.types.PropertyGroup):
        data: bpy.props.StringProperty(
            name="MotionEngine Datastore",
            default="{}",
            options={'HIDDEN'},
        )


    bpy.utils.register_class(MotionEngineProperties)

    bpy.types.Scene.motionengine = bpy.props.PointerProperty(type=MotionEngineProperties)


    def get_property(key):
        scene = bpy.context.scene
        if "motionengine" not in scene:
            scene.motionengine.data = "{}"
        property_str = scene.motionengine.data
        out_dict = {}
        try:
            if isinstance(property_str, str):
                datastore = json.loads(property_str)
                if isinstance(datastore, dict):
                    out_dict = datastore
            else:
                scene.motionengine.data = "{}"
        except Exception as e:
            print("Error decoding JSON:", str(e))
        out_var = None
        if key in out_dict:
            out_var = out_dict[key]
        return out_var


    def set_property(key, value):
        scene = bpy.context.scene
        if "motionengine" not in scene:
            scene.motionengine.data = "{}"
        property_str = scene.motionengine.data
        out_dict = {}
        try:
            if isinstance(property_str, str):
                datastore = json.loads(property_str)
                if isinstance(datastore, dict):
                    out_dict = datastore
            else:
                scene.motionengine.data = "{}"
        except Exception as e:
            pass
        out_dict[key] = value
        try:
            out_str = json.dumps(out_dict)
            scene.motionengine.data = out_str
        except Exception as e:
            print("Error serializing to JSON:", str(e))


    def clear_properties():
        scene = bpy.context.scene
        scene.motionengine.data = "{}"


    def check_if_clip_exists(clip_name):
        clips = bpy.data.movieclips
        for clip in clips:
            if clip.name == clip_name:
                return True
        return False


    def get_clip_names(movie_clips):
        # Get the names of the video files.
        names = []
        for clip in movie_clips:
            name = clip.name
            names.append(name)
        return names


    class MEPointConverter:
        def __init__(self, width, height):
            self.width = width
            self.height = height

        def cv_to_blend(self, x, y):
            return x / self.width, (self.height - y) / self.height

        def blend_to_cv(self, x, y):
            return x * self.width, self.height - (y * self.height)

        def cv_to_blend(self, point: MEPython.Point):
            return MEPython.Point(point.x / self.width, (self.height - point.y) / self.height)

        def blend_to_cv(self, point: MEPython.Point):
            return MEPython.Point(point.x * self.width, self.height - (point.y * self.height))


    class MEViewData:
        def __init__(self, clip_name, frame, view):
            self.clip_name = clip_name
            self.frame = frame
            self.view = view

except:
    pass
