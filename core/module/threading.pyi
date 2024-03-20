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

from . import dnn, io, Mat, future_frame_poses, future_frames
from typing import List, Callable

def RTMDP_infer_async(arg0: dnn.TopDownPoseDetector, arg1: List[Mat], arg2: int, arg3: int, arg4: float, arg5: float) -> future_frame_poses:
    """
    DEPRECATED! To be removed following separation of bpy calls from core module
    :param arg0: Top down pose detection module
    :param arg1: Input images
    :param arg2: Detection batch size
    :param arg3: Pose estimation batch size
    :param arg4: Detection confidence threshold
    :param arg5: Detection IoU threshold
    """
def Transcoder_read_async(arg0: io.FrameProvider, arg1: int) -> future_frames:
    """
    DEPRECATED! To be removed following separation of bpy calls from core module
    :param arg0: Frame provider to use during async read
    :param arg1: Number of frames to read
    """
def infer_async(arg0: dnn.TopDownPoseDetector, arg1: str, arg2: dnn.Executor, arg3: str, arg4: dnn.Executor, arg5: int,
                arg6: int, arg7: int, arg8: float, arg9: float, arg10: object, arg11: object, arg12: Callable[[str], str],
                arg13: Callable[[], None], arg14: Callable[[bool], None], arg15: Callable[[bool], None], arg16: Callable[[List[List[dnn.Pose]]], None]) -> None:
    """
    DEPRECATED! To be removed following separation of bpy calls from core module
    :param arg0: Top down pose detection module
    :param arg1: Detection model path
    :param arg2: Detection executor
    :param arg3: Pose model path
    :param arg4: Pose executor
    :param arg5: Frame cache size
    :param arg6: Detection batch size
    :param arg7: Pose batch size
    :param arg8: Detection confidence threshold
    :param arg9: Detection IoU threshold
    :param arg10: Target bpy clip
    :param arg11: Banner message object
    :param arg12: bpy abspath function
    :param arg13: bpy UI redraw callback
    :param arg14: bpy UI lock callback
    :param arg15: bpy UI warmup status toggle callback
    :param arg16: bpy data write function
    """
def rtm_load_async(arg0: dnn.TopDownPoseDetector, arg1: str, arg2: dnn.Executor, arg3: str,
                   arg4: dnn.Executor, arg5: Callable[[], None], arg6: Callable[[bool], None], arg7: Callable[[bool], None]) -> None:
    """
    DEPRECATED! To be removed following separation of bpy calls from core module
    :param arg0: Top down pose detection module
    :param arg1: Detection model path
    :param arg2: Detection executor
    :param arg3: Pose model path
    :param arg4: Pose executor
    :param arg5: bpy UI redraw callback
    :param arg6: bpy UI lock callback
    :param arg7: bpy UI warmup status toggle callback
    """
