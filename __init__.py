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
import gc

if 'MotionEngine' in locals():
    import importlib
    importlib.reload(MotionEngine)
    importlib.reload(MEPython)
else:
    from . import MotionEngine
    from .MotionEngine import MEPython

bl_info = {
    "name": "MotionEngine",
    "author": "Ian Sloat",
    "version": (0, 1, 0),
    "blender": (2, 93, 0),
    "location": "Clip Editor > UI",
    "description": "AI based motion tracking system for blender",
    "warning": "Pre-release software! Some features might not work as expected",
    "wiki_url": "",
    "tracker_url": "",
    "category": "Motion Tracking"}

me_global_register = {}

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

def RawToFrames(source, start_index):
    frames = MEPython.dnn.PoseFrames()
    for f, frame in enumerate(source):
        collection = MEPython.dnn.PoseCollection()
        for p, pose in enumerate(frame):
            collection.set_pose(p, pose)
        frames.set_pose_frame(f + start_index, collection)
    return frames  
        

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

def force_redraw(self, context):
    if context.area != None:
        for region in context.area.regions:
            if region.type == "UI":
                region.tag_redraw()
    return None

def det_thresh_mode_update(self, context):
    scene = context.scene
    properties = scene.motion_engine_ui_properties
    if properties.me_ui_prop_det_thresholding_enum == "AUTO":
        properties.me_ui_prop_det_conf = 0.5
        properties.me_ui_prop_det_iou = 0.5
    return None

def cache_method_update(self, context):
    scene = context.scene
    properties = scene.motion_engine_ui_properties
    if properties.me_ui_prop_cache_method_enum == "AUTO":
        properties.me_ui_prop_cache_size = 256
    return None

class MotionEngineRunStatistics(bpy.types.PropertyGroup):
    clip: bpy.props.PointerProperty(type=bpy.types.MovieClip)
    valid_frames: bpy.props.IntProperty()
    total_poses: bpy.props.IntProperty()
    mean_confidence: bpy.props.FloatProperty()
    mean50_conf: bpy.props.FloatProperty()

def check_data_for_clip(me_data, movie_clip):
    clip_data = None
    for entry in me_data.items:
        if entry.clip == movie_clip:
            clip_data = entry
            break
    return clip_data

def check_stats_for_clip(stats_data, movie_clip):
    stat_prop = None
    for entry in stats_data:
        if entry.clip == movie_clip:
            stat_prop = entry
            break
    return stat_prop

def calculate_statistics(clip_data, stat_prop):
    print("[MotionEngine] Calculating run statistics...")
    stat_prop.clip = clip_data.clip
    frame_total = 0
    pose_total = 0
    confidence_sum = 0
    mean50_list = []
    for frame in clip_data.frames:
        if frame.poses is not None and len(frame.poses) > 0:
            frame_total = frame_total + 1
            if len(frame.poses) > pose_total:
                pose_total = len(frame.poses)
            frame_sum = 0
            frame50_list = []
            for pose in frame.poses:
                pose_sum = 0
                pose50_list = []
                for joint in pose.joints:
                    pose_sum = pose_sum + joint.prob
                    if joint.prob > 0.5:
                        pose50_list.append(joint.prob)
                frame_sum = frame_sum + pose_sum / len(pose.joints)
                if pose50_list:
                    frame50_list.append(sum(pose50_list) / len(pose50_list))
            confidence_sum = confidence_sum + frame_sum / len(frame.poses)
            if frame50_list:
                mean50_list.append(sum(frame50_list) / len(frame50_list))
    stat_prop.valid_frames = frame_total
    stat_prop.total_poses = pose_total
    stat_prop.mean_confidence = confidence_sum / frame_total
    stat_prop.mean50_conf = sum(mean50_list) / len(mean50_list)

def get_run_statistics(me_data, stats_data, movie_clip):
    clip_data = check_data_for_clip(me_data, movie_clip)
    stat_prop = check_stats_for_clip(stats_data, movie_clip)
    if clip_data is not None and stat_prop is not None:
        return stat_prop
    return None

def write_data_callback(raw_data):
    clip_data = me_global_register["properties_tracker"]
    target_clip = me_global_register["clip_tracker"]
    stats_data = me_global_register["stats_tracker"]
    if clip_data is not None and target_clip is not None:
        print("[MotionEngine] Collecting results...")
        frames = RawToFrames(raw_data, 0)
        clip_dict = {}
        for clip in clip_data.items:
            clip_dict[clip.clip] = clip
        target_clip_data = None
        if target_clip in clip_dict:
            target_clip_data = clip_dict[target_clip]
        else:
            target_clip_data = clip_data.items.add()
            target_clip_data.clip = target_clip
        print("[MotionEngine] Writing results to scene data...")
        ClipDataToBlend(frames, target_clip_data)
        stat_prop = check_stats_for_clip(stats_data, target_clip)
        if stat_prop is None:
            stat_prop = stats_data.add()
        calculate_statistics(target_clip_data, stat_prop)

class MotionEngineUIProperties(bpy.types.PropertyGroup):
    me_ui_redraw_prop: bpy.props.BoolProperty(
        name="Update property",
        description="Hidden prop",
        default=False,
        update=force_redraw
    )
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
    me_ui_prop_det_conf: bpy.props.FloatProperty(
        name="Confidence threshold",
        description="Sets the minimum acceptable confidence score for detections",
        default=0.5,
        min=0,
        max=1
    )
    me_ui_prop_det_iou: bpy.props.FloatProperty(
        name="IoU threshold",
        description="Sets the maximum acceptable overlap between detections. Non-maximum suppression will be used on boxes with overlap values higher than this",
        default=0.5,
        min=0,
        max=1
    )
    me_ui_prop_det_thresholding_enum: bpy.props.EnumProperty(
        name="Thresholding mode",
        description="Lets you decide whether to set threshold values automatically or manually",
        items=[
            ("AUTO", "Auto", "Set threshold values automatically"),
            ("MANUAL", "Manual", "Set threshold values manually"),
        ],
        default="AUTO",
        update=det_thresh_mode_update
    )
    me_ui_prop_cache_method_enum: bpy.props.EnumProperty(
        name="Cache configuration mode",
        description="Lets you decide whether to set image cache size automatically or manually",
        items=[
            ("AUTO", "Auto", "Set image cache automatically"),
            ("MANUAL", "Manual", "Set image cache size manually"),
        ],
        default="AUTO",
        update=cache_method_update
    )
    me_ui_prop_cache_size: bpy.props.IntProperty(
        name="Pre-inference cache size",
        description="Number of frames to load into memory prior to inference",
        default=256,
        min=1,
        max=1024
    )
    me_ui_prop_stats_collection: bpy.props.CollectionProperty(type=MotionEngineRunStatistics)

def ui_draw_toggle():
    properties = bpy.context.scene.motion_engine_ui_properties
    if properties.me_ui_redraw_prop == True:
        properties.me_ui_redraw_prop = False
    else:
        properties.me_ui_redraw_prop = True

def display_warmup_state(value):
    me_global_register["warmup_state"] = value    

def ui_lock(value):
    me_global_register["ui_lock_state"] = value
    

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
        warmup = me_global_register["warmup_state"]
        ui_lock = me_global_register["ui_lock_state"]
        model_ready = me_global_register["me_detectpose_model"].is_ready()

        column = layout.column()
        
        box = column.box()
        column = box.column()
        column.label(text="Detection Model", icon="CON_OBJECTSOLVER")
        row = column.row()
        row.label(text="Executor:")
        row.prop(properties, "me_ui_prop_exe_det_enum", text='')
        row.enabled = not ui_lock
        row = column.row()
        row.label(text="Batch size:")
        row.prop(properties, "me_ui_prop_det_batch_size", text='')
        row.enabled = not ui_lock
        row = column.row()
        row.label(text="Thresholding:")
        row.prop(properties, "me_ui_prop_det_thresholding_enum", text='')
        row.enabled = not ui_lock
        if properties.me_ui_prop_det_thresholding_enum == "MANUAL":
            row = column.row()
            row.alignment = 'RIGHT'
            row.label(text="Confidence:")
            row.prop(properties, "me_ui_prop_det_conf", text="")
            row.enabled = not ui_lock
            row = column.row()
            row.alignment = 'RIGHT'
            row.label(text="IoU:")
            row.prop(properties, "me_ui_prop_det_iou", text="")
            row.enabled = not ui_lock
            
        
        column = layout.column()
        
        box = column.box()
        column = box.column()
        column.label(text="Pose Model", icon="OUTLINER_DATA_ARMATURE")
        row = column.row()
        row.label(text="Executor:")
        row.prop(properties, "me_ui_prop_exe_pose_enum", text='')
        row.enabled = not ui_lock
        row = column.row()
        row.label(text="Batch size:")
        row.prop(properties, "me_ui_prop_pose_batch_size", text='')
        row.enabled = not ui_lock
        
        row = layout.row()
        b_text = "Load Models"
        
        if model_ready and not warmup:
            b_text = "Reload Models"
        elif not model_ready and warmup:
            b_text = "Loading Models..."
        elif model_ready and warmup:
            b_text = "Warming Up..."
        row.enabled = not ui_lock
        row.operator("motionengine.model_load_operator", text=b_text)
        
        row = layout.row()
        row.enabled = model_ready and not ui_lock
        row.operator("motionengine.model_unload_operator")

class ModelLoadOperator(bpy.types.Operator):
    """Load or reload models"""
    bl_idname = "motionengine.model_load_operator"
    bl_label = "Load Models"
    
    def execute(self, context):
        if not me_global_register["ui_lock_state"]:
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
        
            me_global_register["me_detectpose_model"].unload_all()
            
            me_global_register["context_tracker"] = context
            
            ui_lock(True)
            display_warmup_state(True)
            ui_draw_toggle()
        
            MEPython.mt.rtm_load_async(
                me_global_register["me_detectpose_model"], 
                det_path, 
                selected_exec_det, 
                pose_path, 
                selected_exec_pose,
                ui_draw_toggle,
                ui_lock,
                display_warmup_state
            )
            
        return {'FINISHED'}
    
class ModelUnloadOperator(bpy.types.Operator):
    """Unload models"""
    bl_idname = "motionengine.model_unload_operator"
    bl_label = "Unload Models"
    
    def execute(self, context):
        if not me_global_register["ui_lock_state"]:
            me_global_register["me_detectpose_model"].unload_all()
            print("[MotionEngine] Unloaded models.")
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
        warmup = me_global_register["warmup_state"]
        ui_lock = me_global_register["ui_lock_state"]
        model_ready = me_global_register["me_detectpose_model"].is_ready()
        
        column = layout.column()
        
        box = column.box()
        column = box.column()
        column.label(text="Image cache size", icon="RENDERLAYERS")
        row = column.row()
        row = row.grid_flow(columns=2, align=True)
        r = row.row(align=True)
        r.prop(properties, "me_ui_prop_cache_method_enum", text='')
        r.enabled = not ui_lock
        r = row.row(align=True)
        r.enabled = properties.me_ui_prop_cache_method_enum == "MANUAL" and not ui_lock
        r.prop(properties, "me_ui_prop_cache_size", text='')
        
        row = layout.row()
        row.enabled = not ui_lock
        row.operator("motionengine.pose_estimation_task_operator")
        
        run_stats = get_run_statistics(scene.motion_engine_data, properties.me_ui_prop_stats_collection, context.edit_movieclip)
        
        stats_ui_string_frames = "Valid frames: "
        if run_stats is not None and run_stats.valid_frames > 0:
            stats_ui_string_frames += str(run_stats.valid_frames) + " / " + str(context.edit_movieclip.frame_duration)
        else:
            stats_ui_string_frames += "n/a"
        
        stats_ui_string_poses = "Total poses: "
        if run_stats is not None and run_stats.valid_frames > 0:
            stats_ui_string_poses += str(run_stats.total_poses)
        else:
            stats_ui_string_poses += "n/a"
        
        stats_ui_string_conf = "Mean confidence: "
        if run_stats is not None and run_stats.valid_frames > 0:
            stats_ui_string_conf += f"{run_stats.mean_confidence:.0%}"
        else:
            stats_ui_string_conf += "n/a"
            
        stats_ui_string_conf50 = "Mean conf. > 50: "
        if run_stats is not None and run_stats.valid_frames > 0 and run_stats.mean50_conf >= 0.5:
            stats_ui_string_conf50 += f"{run_stats.mean50_conf:.0%}"
        else:
            stats_ui_string_conf50 += "n/a"
            
        column = layout.column()
        row = column.row()
        row.label(text=stats_ui_string_frames)
        row = column.row()
        row.label(text=stats_ui_string_poses)
        row = column.row()
        row.label(text=stats_ui_string_conf)
        row = column.row()
        row.label(text=stats_ui_string_conf50)
        

class PoseEstimationTaskOperator(bpy.types.Operator):
    """Start pose estimation analysis"""
    bl_idname = "motionengine.pose_estimation_task_operator"
    bl_label = "Detect poses"
    
    def execute(self, context):
        if not me_global_register["ui_lock_state"]:
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
            
            me_global_register["context_tracker"] = context
            
            me_global_register["clip_tracker"] = context.edit_movieclip
            
            me_global_register["properties_tracker"] = scene.motion_engine_data
            
            me_global_register["stats_tracker"] = scene.motion_engine_ui_properties.me_ui_prop_stats_collection
            
            cache_size = properties.me_ui_prop_cache_size
            
            det_batch_size = properties.me_ui_prop_det_batch_size
            
            pose_batch_size = properties.me_ui_prop_pose_batch_size
            
            det_conf_thresh = properties.me_ui_prop_det_conf
            
            det_iou_thresh = properties.me_ui_prop_det_iou
            
            ui_lock(True)
            display_warmup_state(False)
            ui_draw_toggle()
        
            MEPython.mt.infer_async(
                me_global_register["me_detectpose_model"], 
                det_path, 
                selected_exec_det, 
                pose_path, 
                selected_exec_pose,
                cache_size,
                det_batch_size,
                pose_batch_size,
                det_conf_thresh,
                det_iou_thresh,
                me_global_register["clip_tracker"],
                bpy.path.abspath,
                ui_draw_toggle,
                ui_lock,
                display_warmup_state,
                write_data_callback
            )
            
        return {'FINISHED'}

def register():
    gc.collect()
    bpy.utils.register_class(MEPointBlender)
    bpy.utils.register_class(MEJointBlender)
    bpy.utils.register_class(MEPoseBlender)
    bpy.utils.register_class(MEFrameBlender)
    bpy.utils.register_class(MEClipData)
    bpy.utils.register_class(MEClipDataCollection)
    bpy.types.Scene.motion_engine_data = bpy.props.PointerProperty(type=MEClipDataCollection)
    
    bpy.utils.register_class(MotionEngineRunStatistics)
    bpy.utils.register_class(MotionEngineUIProperties)
    bpy.types.Scene.motion_engine_ui_properties = bpy.props.PointerProperty(type=MotionEngineUIProperties)
    bpy.utils.register_class(ModelLoadOperator)
    bpy.utils.register_class(ModelUnloadOperator)
    bpy.utils.register_class(PoseEstimationTaskOperator)
    
    bpy.utils.register_class(ModelManagerUIPanel)
    bpy.utils.register_class(PoseEstimationUIPanel)

    # Global vars
    me_global_register["me_detectpose_model"] = MEPython.dnn.RTMDetPoseBundleModel()
    me_global_register["context_tracker"] = None
    me_global_register["properties_tracker"] = None
    me_global_register["clip_tracker"] = None
    me_global_register["stats_tracker"] = None
    me_global_register["warmup_state"] = False
    me_global_register["ui_lock_state"] = False
   
    gc.collect()
    
    print("[MotionEngine] Registration complete.")
    

def unregister():
    bpy.utils.unregister_class(PoseEstimationUIPanel)
    bpy.utils.unregister_class(ModelManagerUIPanel)
    
    bpy.utils.unregister_class(PoseEstimationTaskOperator)
    bpy.utils.unregister_class(ModelUnloadOperator)
    bpy.utils.unregister_class(ModelLoadOperator)
    del bpy.types.Scene.motion_engine_ui_properties
    bpy.utils.unregister_class(MotionEngineUIProperties)
    bpy.utils.unregister_class(MotionEngineRunStatistics)

    me_global_register["me_detectpose_model"].unload_all()
    me_global_register["context_tracker"] = None
    me_global_register["properties_tracker"] = None
    me_global_register["clip_tracker"] = None
    me_global_register["stats_tracker"] = None
    me_global_register.clear()
    
    del bpy.types.Scene.motion_engine_data
    bpy.utils.unregister_class(MEClipDataCollection)
    bpy.utils.unregister_class(MEClipData)
    bpy.utils.unregister_class(MEFrameBlender)
    bpy.utils.unregister_class(MEPoseBlender)
    bpy.utils.unregister_class(MEJointBlender)
    bpy.utils.unregister_class(MEPointBlender)
    
    gc.collect()
    
    print("[MotionEngine] Unregistration complete.")


if __name__ == "__main__":
    register()
