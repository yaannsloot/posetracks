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
MEUtils = importlib.import_module("MEUtils")
MECrypto = importlib.import_module("MECrypto")
MEDNNBase = importlib.import_module("MEDNNBase")

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

class MEPoint:
    def __init__(self, x, y):
        self.x = x
        self.y = y

class MEJoint:
    def __init__(self, x, y, prob):
        self.pt = MEPoint(x, y)
        self.prob = prob

class MEPose:
    def __init__(self):
        self.data = {}

    def set_joint(self, joint_id, x, y, prob):
        self.data[joint_id] = MEJoint(x, y, prob)

    def get_joint(self, joint_id):
        if joint_id in self.data:
            return self.data[joint_id]
        else:
            return None

    def has_joint(self, joint_id):
        return joint_id in self.data

    def get_joint_ids(self):
        return list(self.data.keys())

    def get_joint_count(self):
        return len(self.data)

def pose_to_dict(pose: MEPose):
    out_dict = {}
    for joint_id in pose.data:
        joint = pose.data[joint_id]
        out_dict[joint_id] = {
            "x": joint.pt.x,
            "y": joint.pt.y,
            "prob": joint.prob
        }
    return out_dict

def dict_to_pose(pose_dict):
    out_pose = MEPose()
    for joint_id in pose_dict:
        joint_dict = pose_dict[joint_id]
        x = joint_dict["x"]
        y = joint_dict["y"]
        prob = joint_dict["prob"]
        out_pose.set_joint(joint_id, x, y, prob)
    return out_pose

class MEPoseCollection:
    def __init__(self):
        self.data = {}

    def add_pose(self, pose_id):
        self.data[pose_id] = MEPose()

    def set_pose(self, pose_id, pose: MEPose):
        self.data[pose_id] = pose

    def get_pose(self, pose_id):
        if pose_id in self.data:
            return self.data[pose_id]
        else:
            return None

    def has_pose(self, pose_id):
        return pose_id in self.data

    def get_pose_ids(self):
        return list(self.data.keys())

    def get_pose_count(self):
        return len(self.data)

def pose_collection_to_dict(pose_collection: MEPoseCollection):
    out_dict = {}
    for pose_id in pose_collection.data:
        pose = pose_collection.data[pose_id]
        out_dict[pose_id] = pose_to_dict(pose)
    return out_dict

def dict_to_pose_collection(pose_collection_dict):
    out_pose_collection = MEPoseCollection()
    for pose_id in pose_collection_dict:
        pose_dict = pose_collection_dict[pose_id]
        pose = dict_to_pose(pose_dict)
        out_pose_collection.set_pose(pose_id, pose)
    return out_pose_collection

class MEPoseFrames:
    def __init__(self):
        self.data = {}

    def add_pose_frame(self, frame):
        self.data[frame] = MEPoseCollection()

    def set_pose_frame(self, frame, pose_collection: MEPoseCollection):
        self.data[frame] = pose_collection

    def get_pose_frame(self, frame):
        if frame in self.data:
            return self.data[frame]
        else:
            return None

    def has_pose_frame(self, frame):
        return frame in self.data

    def get_pose_frame_ids(self):
        return list(self.data.keys())

    def get_pose_frame_count(self):
        return len(self.data)

def pose_frames_to_dict(pose_frames: MEPoseFrames):
    out_dict = {}
    for frame in pose_frames.data:
        pose_collection = pose_frames.data[frame]
        out_dict[frame] = pose_collection_to_dict(pose_collection)
    return out_dict

def dict_to_pose_frames(pose_frames_dict):
    out_pose_frames = MEPoseFrames()
    for frame in pose_frames_dict:
        pose_collection_dict = pose_frames_dict[frame]
        pose_collection = dict_to_pose_collection(pose_collection_dict)
        out_pose_frames.set_pose_frame(frame, pose_collection)
    return out_pose_frames

class MEStreamer:
    def __init__(self, videofile):
        self.reader = None
        self.videofile = videofile

    def close(self):
        if self.reader == None:
            return
        self.reader.close()
        self.reader = None

    def open(self):
        self.close()
        self.reader = MEUtils.MovieReader(self.videofile)

    def is_open(self):
        if self.reader == None:
            return False
        return self.reader.is_open()

    def frame_count(self):
        if self.reader == None:
            return 0
        return self.reader.frame_count()

    def current_frame(self):
        if self.reader == None:
            return 0
        return self.reader.current_frame()

    def frame_size(self):
        if self.reader == None:
            return (0, 0)
        return self.reader.frame_size()

    def frame_rate(self):
        if self.reader == None:
            return 0
        return self.reader.get_fps()

    def frame(self, frame):
        if self.reader == None:
            return None
        return self.reader.grab_frame(frame)

    def next_frame(self):
        if self.reader == None:
            return None
        return self.reader.frame()

# Global pose model and driver class
class MEPoseModel:
    def __init__(self):
        self.module_config = {}
        self.module_config["detector_model_path"] = model_path("rtmdet-m")
        self.module_config["pose_model_path"] = model_path("rtmpose-m")
        self.module_config["target_device"] = "gpu"
        self.module_instance = MEDNNBase.MEDNNModule(dnn_module("MEDNNMMDeploy"))
        self.module_instance.SetForwardRules(self.module_config)
        self.module_instance.LoadModel(self.module_config)
    
    def reload(self):
        self.module_instance.UnloadModel()
        self.module_instance.LoadModel(self.module_config)

    def unload(self):
        self.module_instance.UnloadModel()

    def detect(self, frame):
        result = self.module_instance.Forward(frame)
        pose_collection = MEPoseCollection()
        for kp in result:
            pose_id = int(kp[0])
            joint_id = int(kp[1])
            x = float(kp[2])
            y = float(kp[3])
            score = float(kp[4])
            pose = pose_collection.get_pose(pose_id)
            if pose == None:
                pose_collection.add_pose(pose_id)
                pose = pose_collection.get_pose(pose_id)
            pose.set_joint(joint_id, x, y, score)
        return pose_collection

MEGlobalPoseDetector = MEPoseModel()

class MEPoseStreamer(MEStreamer):
    def grab_poses(self, frame):
        if self.reader == None:
            return None, False
        np_frame, success = self.frame(frame)
        for i in range(100):
            if not success:
                np_frame, success = self.frame(frame)
            else:
                break
        if not success:
            return None, False
        pose_collection = MEGlobalPoseDetector.detect(np_frame)
        return pose_collection, True

    def next_poses(self):
        if self.reader == None:
            return None, False
        np_frame, success = self.next_frame()
        for i in range(100):
            if not success:
                np_frame, success = self.next_frame()
            else:
                break
        if not success:
            return None, False
        pose_collection = MEGlobalPoseDetector.detect(np_frame)
        return pose_collection, True

def pose_common_points(pose1: MEPose, pose2: MEPose, threshold):
    points1 = []
    points2 = []
    pose1_joint_ids = pose1.get_joint_ids()
    pose2_joint_ids = pose2.get_joint_ids()
    for joint_id in pose1_joint_ids:
        joint1 = pose1.get_joint(joint_id)
        joint2 = pose2.get_joint(joint_id)
        if joint_id in pose2_joint_ids and joint1.prob > threshold and joint2.prob > threshold:
            points1.append((joint1.pt.x, joint1.pt.y))
            points2.append((joint2.pt.x, joint2.pt.y))
    return points1, points2

def pose_collection_common_points(pose_collection1: MEPoseCollection, pose_collection2: MEPoseCollection, threshold):
    points1 = []
    points2 = []
    pose1_ids = pose_collection1.get_pose_ids()
    pose2_ids = pose_collection2.get_pose_ids()
    for pose_id in pose1_ids:
        pose1 = pose_collection1.get_pose(pose_id)
        pose2 = pose_collection2.get_pose(pose_id)
        if pose_id in pose2_ids:
            p1, p2 = pose_common_points(pose1, pose2, threshold)
            points1.extend(p1)
            points2.extend(p2)
    return points1, points2

def pose_frames_common_points(pose_frames1: MEPoseFrames, pose_frames2: MEPoseFrames, threshold):
    points1 = []
    points2 = []
    frame_ids = pose_frames1.get_pose_frame_ids()
    for frame_id in frame_ids:
        pose_collection1 = pose_frames1.get_pose_frame(frame_id)
        pose_collection2 = pose_frames2.get_pose_frame(frame_id)
        if pose_collection1 != None and pose_collection2 != None:
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

        def cv_to_blend(self, point: MEPoint):
            return MEPoint(point.x / self.width, (self.height - point.y) / self.height)

        def blend_to_cv(self, point: MEPoint):
            return MEPoint(point.x * self.width, self.height - (point.y * self.height))

    

    class MEViewData:
        def __init__(self, clip_name, frame, view):
            self.clip_name = clip_name
            self.frame = frame
            self.view = view

except:
    pass