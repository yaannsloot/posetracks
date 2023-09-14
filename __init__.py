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

import bpy

if 'MotionEngine' in locals():
    import importlib
    importlib.reload(MotionEngine)
    importlib.reload(MEPython)
else:
    from . import MotionEngine
    from .MotionEngine import MEPython
    
from enum import Enum

bl_info = {
    "name": "MotionEngine",
    "author": "Ian Sloat",
    "version": (0, 1, 0),
    "blender": (2, 93, 0),
    "location": "Clip Editor > UI",
    "description": "AI based tracking system",
    "warning": "Pre-release software! Some features might not work as expected",
    "wiki_url": "",
    "tracker_url": "",
    "category": "Motion Tracking"}

me_global_register = {}

me_model_load_state = Enum('me_model_load_state', ['UNLOADED', 'LOADING', 'WARMUP', 'COMPLETE'])

me_tracking_state = Enum('me_tracking_state', ['INACTIVE', 'CACHING', 'WORKING', 'COMPLETE'])


# Property groups for data storage
class MEPointBlender(bpy.types.PropertyGroup):
    x: bpy.props.FloatProperty()
    y: bpy.props.FloatProperty()

class MEJointBlender(bpy.types.PropertyGroup):
    id: bpy.props.IntProperty()
    prob: bpy.props.FloatProperty()
    pt: bpy.props.PointerProperty(type=MEPointBlender)
    
class MEPoseBlender(bpy.types.PropertyGroup):
    id: bpy.props.IntProperty()
    joints: bpy.props.CollectionProperty(type=MEJointBlender)
    
class MEFrameBlender(bpy.types.PropertyGroup):
    id: bpy.props.IntProperty()
    poses: bpy.props.CollectionProperty(type=MEPoseBlender)
    
class MEClipData(bpy.types.PropertyGroup):
    clip: bpy.props.PointerProperty(type=bpy.types.MovieClip)
    frames: bpy.props.CollectionProperty(type=MEFrameBlender)
    
class MEClipDataCollection(bpy.types.PropertyGroup):
    items: bpy.props.CollectionProperty(type=MEClipData)
    
    
# Data interchange utility functions
def PointToBlend(source: MEPython.Point, target: MEPointBlender):
    target.x = source.x
    target.y = source.y
    
def JointToBlend(source: MEPython.dnn.Joint, target: MEJointBlender, id):
    PointToBlend(source.pt, target.pt)
    target.prob = source.prob
    target.id = id
    
def PoseToBlend(source: MEPython.dnn.Pose, target: MEPoseBlender, id):
    source_ids = source.get_joint_ids()
    target_ids = {}
    for target_item in target.joints:
        target_ids[target_item.id] = target_item
    for source_id in source_ids:
        target_joint = None
        if source_id in target_ids:
            target_joint = target_ids[source_id]
        else:
            target_joint = target.joints.add()
        JointToBlend(source[source_id], target_joint, source_id)
    target.id = id
        
def FrameToBlend(source: MEPython.dnn.PoseCollection, target: MEFrameBlender, id):
    source_ids = source.get_pose_ids()
    target_ids = {}
    for target_item in target.poses:
        target_ids[target_item.id] = target_item
    for source_id in source_ids:
        target_pose = None
        if source_id in target_ids:
            target_pose = target_ids[source_id]
        else:
            target_pose = target.poses.add()
        PoseToBlend(source[source_id], target_pose, source_id)
    target.id = id
    
def ClipDataToBlend(source: MEPython.dnn.PoseFrames, target: MEClipData):
    source_ids = source.get_pose_frame_ids()
    target_ids = {}
    for target_item in target.frames:
        target_ids[target_item.id] = target_item
    for source_id in source_ids:
        target_frame = None
        if source_id in target_ids:
            target_frame = target_ids[source_id]
        else:
            target_frame = target.frames.add()
        FrameToBlend(source[source_id], target_frame, source_id)
    
def BlendToPoint(source: MEPointBlender):
    return MEPython.Point(source.x, source.y)

def BlendToJoint(source: MEJointBlender):
    return MEPython.dnn.Joint(BlendToPoint(source.pt), source.prob)

def BlendToPose(source: MEPoseBlender):
    pose = MEPython.dnn.Pose()
    for s_joint in source.joints:
        pose.set_joint(s_joint.id, BlendToJoint(s_joint))
    return pose

def BlendToFrame(source: MEFrameBlender):
    frame = MEPython.dnn.PoseCollection()
    for s_pose in source.poses:
        frame.set_pose(s_pose.id, BlendToPose(s_pose))
    return frame

def BlendToClipData(source: MEClipData):
    data = MEPython.dnn.PoseFrames()
    for s_frame in source.frames:
        data.set_pose_frame(s_frame.id, BlendToFrame(s_frame))
    return data


# Data removal functions
def ClearBlendJoint(source: MEJointBlender):
    del source.id
    del source.prob
    del source.pt
    
def ClearBlendPose(source: MEPoseBlender):
    for joint in source.joints:
        ClearBlendJoint(joint)
    source.joints.clear()
    del source.id

def ClearBlendFrame(source: MEFrameBlender):
    for pose in source.poses:
        ClearBlendPose(pose)
    source.poses.clear()
    del source.id

def ClearBlendClipData(source: bpy.types.MovieClip):
    clip_data_collection = bpy.context.scene.motion_engine_data.items
    for clip_data in clip_data_collection:
        if clip_data.clip == source:
            for frame in clip_data.frames:
                ClearBlendFrame(frame)
            clip_data.frames.clear()
            clip_data_collection.remove(clip_data)
            break
        



# UI Code

def check_model_ready():
    
    # load process control logic
    if me_global_register["me_load_future"] != None:
        state = me_global_register["me_load_future"].wait_for(0)
        if state != MEPython.future_status.ready:
            return False
        me_global_register["me_load_future"] = None
        
        if me_global_register["me_load_state"] == me_model_load_state.LOADING:
            me_global_register["me_load_future"] = MEPython.mt.RTMDP_warmup_async(me_global_register["me_detectpose_model"])
            me_global_register["me_load_state"] = me_model_load_state.WARMUP
            return False
        elif me_global_register["me_load_state"] == me_model_load_state.WARMUP:
            me_global_register["me_load_state"] = me_model_load_state.COMPLETE
    
    
    return me_global_register["me_detectpose_model"].is_ready()

def update_analysis_task():
    print("wow")
    return True
    

class MotionEngineUIProperties(bpy.types.PropertyGroup):
    me_ui_prop_remove_poses: bpy.props.BoolProperty(
        name="Remove existing poses",
        description="Delete existing poses on new analysis",
        default=False
    )
    me_ui_prop_exe_det_enum: bpy.props.EnumProperty(
        name="Target execution device",
        description="Model execution device",
        items=[
            ("CPU", "CPU", "Run model on the CPU"),
            ("CUDA", "CUDA", "Run model on GPU using CUDA (Recommended)"),
        ],
        default="CUDA"
    )
    me_ui_prop_exe_pose_enum: bpy.props.EnumProperty(
        name="Target execution device",
        description="Model execution device",
        items=[
            ("CPU", "CPU", "Run model on the CPU"),
            ("CUDA", "CUDA", "Run model on GPU using CUDA (Recommended)"),
            ("TENSORRT", "TensorRT", "Run model on GPU using TensorRT. Faster than CUDA but prone to loading errors"),
        ],
        default="CUDA"
    )
    me_ui_prop_det_batch_size: bpy.props.IntProperty(
        name="Batch size for detection model",
        description="Sets the amount of frames to be processed by the detection model at any given time",
        default=32,
        min=1,
        max=256
    )
    me_ui_prop_pose_batch_size: bpy.props.IntProperty(
        name="Batch size for pose model",
        description="Sets the amount of detections to be processed by the pose model at any given time",
        default=32,
        min=1,
        max=256
    )
    me_ui_prop_cache_method_enum: bpy.props.EnumProperty(
        name="Cache configuration method",
        description="Lets you decide whether to set image cache size automatically or manually",
        items=[
            ("AUTO", "Auto", "Set image cache automatically"),
            ("MANUAL", "Manual", "Set image cache size manually"),
        ],
        default="AUTO"
    )
    me_ui_prop_cache_size: bpy.props.IntProperty(
        name="Pre-inference cache size",
        description="Number of frames to load into memory prior to inference",
        default=256,
        min=1,
        max=1024
    )

class ModelManagerUIPanel(bpy.types.Panel):
    bl_label = "Model Manager"
    bl_idname = "MOTIONENGINE_MODEL_MANAGER_PT_panel"
    bl_space_type = "CLIP_EDITOR"
    bl_region_type = "UI"
    bl_category = "MotionEngine"

    def draw(self, context):
        layout = self.layout
        scene = context.scene
        properties = scene.motion_engine_ui_properties

        column = layout.column()
        
        box = column.box()
        column = box.column()
        column.label(text="Detection Model", icon="CON_OBJECTSOLVER")
        row = column.row()
        row.label(text="Executor:")
        row.prop(properties, "me_ui_prop_exe_det_enum", text='')
        row = column.row()
        row.label(text="Batch size:")
        row.prop(properties, "me_ui_prop_det_batch_size", text='')
        
        column = layout.column()
        
        box = column.box()
        column = box.column()
        column.label(text="Pose Model", icon="OUTLINER_DATA_ARMATURE")
        row = column.row()
        row.label(text="Executor:")
        row.prop(properties, "me_ui_prop_exe_pose_enum", text='')
        row = column.row()
        row.label(text="Batch size:")
        row.prop(properties, "me_ui_prop_pose_batch_size", text='')
        
        row = layout.row()
        b_text = "Load Models"
        
        readystate = check_model_ready()
        loadstate = me_global_register["me_load_state"]
        
        if loadstate == me_model_load_state.COMPLETE:
            b_text = "Reload Models"
        elif loadstate == me_model_load_state.LOADING:
            b_text = "Loading Models..."
        elif loadstate == me_model_load_state.WARMUP:
            b_text = "Warming Up..."
        b_enabled = False
        if loadstate == me_model_load_state.UNLOADED or loadstate == me_model_load_state.COMPLETE:
            b_enabled = True
        row.enabled = b_enabled
        row.operator("motionengine.model_load_operator", text=b_text)
        
        row = layout.row()
        row.enabled = readystate
        row.operator("motionengine.model_unload_operator")

class ModelLoadOperator(bpy.types.Operator):
    """Load or reload models"""
    bl_idname = "motionengine.model_load_operator"
    bl_label = "Load Models"
    
    _timer = None
    
    def modal(self, context, event):
        check_model_ready()
        loadstate = me_global_register["me_load_state"]
        if loadstate == me_model_load_state.UNLOADED or loadstate == me_model_load_state.COMPLETE:
            self.cancel(context)
            for area in bpy.context.screen.areas:
                if area.type == 'CLIP_EDITOR':
                    area.tag_redraw()
            return {'CANCELLED'}

        if event.type == 'TIMER':
            for area in bpy.context.screen.areas:
                if area.type == 'CLIP_EDITOR':
                    area.tag_redraw()

        return {'PASS_THROUGH'}
    
    def execute(self, context):
        wm = context.window_manager
        self._timer = wm.event_timer_add(0.2, window=context.window)
        wm.modal_handler_add(self)
        
        scene = context.scene
        properties = scene.motion_engine_ui_properties
        
        selected_exec_det = MEPython.dnn.Executor.CPU
        if properties.me_ui_prop_exe_det_enum == "CUDA":
            selected_exec_det = MEPython.dnn.Executor.CUDA
        elif properties.me_ui_prop_exe_det_enum == "TENSORRT":
            selected_exec_det = MEPython.dnn.Executor.TENSORRT
            
        selected_exec_pose = MEPython.dnn.Executor.CPU
        if properties.me_ui_prop_exe_pose_enum == "CUDA":
            selected_exec_pose = MEPython.dnn.Executor.CUDA
        elif properties.me_ui_prop_exe_pose_enum == "TENSORRT":
            selected_exec_pose = MEPython.dnn.Executor.TENSORRT
        
        det_path = MotionEngine.model_path("rtmdet-nano/end2end320.onnx")
        pose_path = MotionEngine.model_path("rtmpose-m/end2end.onnx")
    
        me_global_register["me_load_future"] = MEPython.mt.RTMDP_load_async(me_global_register["me_detectpose_model"], det_path, selected_exec_det, pose_path, selected_exec_pose)
        me_global_register["me_load_state"] = me_model_load_state.LOADING
        
        return {'RUNNING_MODAL'}
    
    def cancel(self, context):
        wm = context.window_manager
        wm.event_timer_remove(self._timer)
    
class ModelUnloadOperator(bpy.types.Operator):
    """Unload models"""
    bl_idname = "motionengine.model_unload_operator"
    bl_label = "Unload Models"
    
    def execute(self, context):
        
        me_global_register["me_detectpose_model"].unload_all()
        if me_global_register["me_load_future"] != None:
            me_global_register["me_load_future"].wait()
        me_global_register["me_load_future"] = None
        me_global_register["me_load_state"] = me_model_load_state.UNLOADED
        
        return {'FINISHED'}
        

class PoseEstimationUIPanel(bpy.types.Panel):
    bl_label = "Pose Estimation"
    bl_idname = "MOTIONENGINE_POSE_ESTIMATION_PT_panel"
    bl_space_type = "CLIP_EDITOR"
    bl_region_type = "UI"
    bl_category = "MotionEngine"
    
    def draw(self, context):
        layout = self.layout
        scene = context.scene
        properties = scene.motion_engine_ui_properties
        
        column = layout.column()
        
        box = column.box()
        column = box.column()
        column.label(text="Image cache size", icon="RENDERLAYERS")
        row = column.row()
        row = row.grid_flow(columns=2, align=True)
        r = row.row(align=True)
        r.prop(properties, "me_ui_prop_cache_method_enum", text='')
        r = row.row(align=True)
        r.enabled = properties.me_ui_prop_cache_method_enum == "MANUAL"
        r.prop(properties, "me_ui_prop_cache_size", text='')
        
        row = layout.row()
        
        loadstate = me_global_register["me_load_state"]
        trackingstate = me_global_register["me_tracking_state"]
        row.enabled = (loadstate == me_model_load_state.UNLOADED or loadstate == me_model_load_state.COMPLETE) and (trackingstate == me_tracking_state.INACTIVE or trackingstate == me_tracking_state.COMPLETE)
        row.operator("motionengine.pose_estimation_task_operator")
        
        

class PoseEstimationTaskOperator(bpy.types.Operator):
    """Start pose estimation analysis"""
    bl_idname = "motionengine.pose_estimation_task_operator"
    bl_label = "Detect poses"
    
    _timer = None
    
    def modal(self, context, event):
        update_analysis_task()
        trackingstate = me_global_register["me_tracking_state"]
        if trackingstate == me_tracking_state.INACTIVE or trackingstate == me_tracking_state.COMPLETE:
            self.cancel(context)
            for area in bpy.context.screen.areas:
                if area.type == 'CLIP_EDITOR':
                    area.tag_redraw()
            return {'CANCELLED'}
        
        if event.type == 'TIMER':
            for area in bpy.context.screen.areas:
                if area.type == 'CLIP_EDITOR':
                    area.tag_redraw()

        return {'PASS_THROUGH'}
    
    def execute(self, context):
        wm = context.window_manager
        self._timer = wm.event_timer_add(0.2, window=context.window)
        wm.modal_handler_add(self)
        
        scene = context.scene
        properties = scene.motion_engine_ui_properties
        
        # Task setup and execution code here
        
        return {'RUNNING_MODAL'}
    
    def cancel(self, context):
        wm = context.window_manager
        wm.event_timer_remove(self._timer)
         


def register():
    bpy.utils.register_class(MEPointBlender)
    bpy.utils.register_class(MEJointBlender)
    bpy.utils.register_class(MEPoseBlender)
    bpy.utils.register_class(MEFrameBlender)
    bpy.utils.register_class(MEClipData)
    bpy.utils.register_class(MEClipDataCollection)
    bpy.types.Scene.motion_engine_data = bpy.props.PointerProperty(type=MEClipDataCollection)
    
    bpy.utils.register_class(MotionEngineUIProperties)
    bpy.types.Scene.motion_engine_ui_properties = bpy.props.PointerProperty(type=MotionEngineUIProperties)
    bpy.utils.register_class(ModelLoadOperator)
    bpy.utils.register_class(ModelUnloadOperator)
    bpy.utils.register_class(PoseEstimationTaskOperator)
    
    bpy.utils.register_class(ModelManagerUIPanel)
    bpy.utils.register_class(PoseEstimationUIPanel)

    # Global vars
    me_global_register["me_detectpose_model"] = MEPython.dnn.RTMDetPoseBundleModel()
    me_global_register["me_load_future"] = None
    me_global_register["me_load_state"] = me_model_load_state.UNLOADED
    me_global_register["me_cache_futures"] = None
    me_global_register["me_tracking_state"] = me_tracking_state.INACTIVE
    

def unregister():
    bpy.utils.unregister_class(PoseEstimationUIPanel)
    bpy.utils.unregister_class(ModelManagerUIPanel)
    
    bpy.utils.unregister_class(PoseEstimationTaskOperator)
    bpy.utils.unregister_class(ModelUnloadOperator)
    bpy.utils.unregister_class(ModelLoadOperator)
    del bpy.types.Scene.motion_engine_ui_properties
    bpy.utils.unregister_class(MotionEngineUIProperties)

    me_global_register["me_detectpose_model"].unload_all()
    if me_global_register["me_load_future"] != None:
        me_global_register["me_load_future"].wait()
    me_global_register["me_load_future"] = None
    if me_global_register["me_cache_futures"] != None:
        for future in me_global_register["me_cache_futures"]:
            future.wait()
    me_global_register["me_cache_futures"] = None
    me_global_register.clear()
    
    del bpy.types.Scene.motion_engine_data
    bpy.utils.unregister_class(MEClipDataCollection)
    bpy.utils.unregister_class(MEClipData)
    bpy.utils.unregister_class(MEFrameBlender)
    bpy.utils.unregister_class(MEPoseBlender)
    bpy.utils.unregister_class(MEJointBlender)
    bpy.utils.unregister_class(MEPointBlender)


if __name__ == "__main__":
    register()
