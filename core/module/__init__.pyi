"""
Base module. Contains core types and common functions.
"""

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

from typing import ClassVar, List, Tuple, Iterator, overload
from . import crypto
from . import data
from . import dnn
from . import io
from . import threading
from . import tracking

DICT_4X4_100: TagDictionary
DICT_4X4_1000: TagDictionary
DICT_4X4_250: TagDictionary
DICT_4X4_50: TagDictionary
DICT_5X5_100: TagDictionary
DICT_5X5_1000: TagDictionary
DICT_5X5_250: TagDictionary
DICT_5X5_50: TagDictionary
DICT_6X6_100: TagDictionary
DICT_6X6_1000: TagDictionary
DICT_6X6_250: TagDictionary
DICT_6X6_50: TagDictionary
DICT_7X7_100: TagDictionary
DICT_7X7_1000: TagDictionary
DICT_7X7_250: TagDictionary
DICT_7X7_50: TagDictionary
DICT_APRILTAG_16h5: TagDictionary
DICT_APRILTAG_25h9: TagDictionary
DICT_APRILTAG_36h10: TagDictionary
DICT_APRILTAG_36h11: TagDictionary
DICT_ARUCO_ORIGINAL: TagDictionary
deferred: future_status
ready: future_status
timeout: future_status

class Mat:
    """
    Reference class for cv::Mat objects
    """
    def __init__(self) -> None:
        """
        Create a new empty matrix
        """

class Point:
    """
    Reference class for cv::Point2d objects
    """
    x: float
    y: float
    @overload
    def __init__(self) -> None:
        """
        Create a new point
        """
    @overload
    def __init__(self, arg0: float, arg1: float) -> None:
        """
        Create a new point
        :param arg0: X coordinate
        :param arg1: Y coordinate
        """
    def __iter__(self) -> Iterator[float]: ...
    def __getitem__(self, item) -> float: ...
    def __setitem__(self, key, value) -> None: ...
    def __len__(self) -> int: ...

class Pointf:
    """
    Reference class for cv::Point2f objects
    """
    x: float
    y: float
    @overload
    def __init__(self) -> None:
        """
        Create a new point. The coordinates are stored internally as single precision FP32 values
        """
    @overload
    def __init__(self, arg0: float, arg1: float) -> None:
        """
        Create a new point. The coordinates are stored internally as single precision FP32 values
        :param arg0: X coordinate
        :param arg1: Y coordinate
        """
    def __iter__(self) -> Iterator[float]: ...
    def __getitem__(self, item) -> float: ...
    def __setitem__(self, key, value) -> None: ...
    def __len__(self) -> int: ...

class Rect:
    """
    Reference class for cv::Rect2d objects
    """
    height: float
    width: float
    x: float
    y: float
    @overload
    def __init__(self) -> None:
        """
        Create a new empty bounding box
        """
    @overload
    def __init__(self, arg0: float, arg1: float, arg2: float, arg3: float) -> None:
        """
        Create a new bounding box
        :param arg0: X coordinate of top left corner
        :param arg1: Y coordinate of top left corner
        :param arg2: Width of bounding box
        :param arg3: Height of bounding box
        """
    @overload
    def __init__(self, arg0: Rect) -> None:
        """
        Create a new bounding box by copying an existing one
        :param arg0: Source bounding box
        """
    def area(self) -> float:
        """
        Get the area of this bounding box
        """
    def br(self) -> Point:
        """Get the bottom right corner of this bounding box"""
    def contains(self, arg0: Point) -> bool:
        """
        Check whether a point is inside this bounding box
        :param arg0: Point to check
        """
    def empty(self) -> bool:
        """True if empty i.e. the box has an area of 0"""
    def size(self) -> tuple[float, float]:
        """Size of bounding box in (width, height)"""
    def tl(self) -> Point:
        """Get the top left corner of this bounding box"""
    def __eq__(self, arg0: Rect) -> bool:
        """Check if the provided bounding box has the same dimensions as this one"""
    def __ne__(self, arg0: Rect) -> bool:
        """Check if the provided bounding box does not have the same dimensions as this one"""

class TagDictionary:
    __members__: ClassVar[dict] = ...  # read-only
    DICT_4X4_100: ClassVar[TagDictionary] = ...
    DICT_4X4_1000: ClassVar[TagDictionary] = ...
    DICT_4X4_250: ClassVar[TagDictionary] = ...
    DICT_4X4_50: ClassVar[TagDictionary] = ...
    DICT_5X5_100: ClassVar[TagDictionary] = ...
    DICT_5X5_1000: ClassVar[TagDictionary] = ...
    DICT_5X5_250: ClassVar[TagDictionary] = ...
    DICT_5X5_50: ClassVar[TagDictionary] = ...
    DICT_6X6_100: ClassVar[TagDictionary] = ...
    DICT_6X6_1000: ClassVar[TagDictionary] = ...
    DICT_6X6_250: ClassVar[TagDictionary] = ...
    DICT_6X6_50: ClassVar[TagDictionary] = ...
    DICT_7X7_100: ClassVar[TagDictionary] = ...
    DICT_7X7_1000: ClassVar[TagDictionary] = ...
    DICT_7X7_250: ClassVar[TagDictionary] = ...
    DICT_7X7_50: ClassVar[TagDictionary] = ...
    DICT_APRILTAG_16h5: ClassVar[TagDictionary] = ...
    DICT_APRILTAG_25h9: ClassVar[TagDictionary] = ...
    DICT_APRILTAG_36h10: ClassVar[TagDictionary] = ...
    DICT_APRILTAG_36h11: ClassVar[TagDictionary] = ...
    DICT_ARUCO_ORIGINAL: ClassVar[TagDictionary] = ...
    __entries: ClassVar[dict] = ...
    def __init__(self, value: int) -> None: ...
    def __eq__(self, other: object) -> bool: ...
    def __hash__(self) -> int: ...
    def __index__(self) -> int: ...
    def __int__(self) -> int: ...
    def __ne__(self, other: object) -> bool: ...
    @property
    def name(self) -> str: ...
    @property
    def value(self) -> int: ...

class future_frame_poses:
    def __init__(self) -> None:
        """__init__(self: MEPython.future_frame_poses) -> None"""
    def get(self) -> List[List[dnn.Pose]]:
        """get(self: MEPython.future_frame_poses) -> List[List[MEPython.dnn.Pose]]"""
    def valid(self) -> bool:
        """valid(self: MEPython.future_frame_poses) -> bool"""
    def wait(self) -> None:
        """wait(self: MEPython.future_frame_poses) -> None"""
    def wait_for(self, arg0: int) -> future_status:
        """wait_for(self: MEPython.future_frame_poses, arg0: int) -> MEPython.future_status"""

class future_frames:
    def __init__(self) -> None:
        """__init__(self: MEPython.future_frames) -> None"""
    def get(self) -> Tuple[bool, List[Mat]]:
        """get(self: MEPython.future_frames) -> Tuple[bool, List[MEPython.Mat]]"""
    def valid(self) -> bool:
        """valid(self: MEPython.future_frames) -> bool"""
    def wait(self) -> None:
        """wait(self: MEPython.future_frames) -> None"""
    def wait_for(self, arg0: int) -> future_status:
        """wait_for(self: MEPython.future_frames, arg0: int) -> MEPython.future_status"""

class future_status:
    __members__: ClassVar[dict] = ...  # read-only
    __entries: ClassVar[dict] = ...
    deferred: ClassVar[future_status] = ...
    ready: ClassVar[future_status] = ...
    timeout: ClassVar[future_status] = ...
    def __init__(self, value: int) -> None:
        """__init__(self: MEPython.future_status, value: int) -> None"""
    def __eq__(self, other: object) -> bool:
        """__eq__(self: object, other: object) -> bool"""
    def __hash__(self) -> int:
        """__hash__(self: object) -> int"""
    def __index__(self) -> int:
        """__index__(self: MEPython.future_status) -> int"""
    def __int__(self) -> int:
        """__int__(self: MEPython.future_status) -> int"""
    def __ne__(self, other: object) -> bool:
        """__ne__(self: object, other: object) -> bool"""
    @property
    def name(self) -> str: ...
    @property
    def value(self) -> int: ...

class future_void:
    def __init__(self) -> None:
        """__init__(self: MEPython.future_void) -> None"""
    def get(self) -> None:
        """get(self: MEPython.future_void) -> None"""
    def valid(self) -> bool:
        """valid(self: MEPython.future_void) -> bool"""
    def wait(self) -> None:
        """wait(self: MEPython.future_void) -> None"""
    def wait_for(self, arg0: int) -> future_status:
        """wait_for(self: MEPython.future_void, arg0: int) -> MEPython.future_status"""

def filter_2D(clip: object, measurement_noise_cov: float = 0.01, process_noise_cov: float = 0.0001) -> None:
    """
    DEPRECATED -> To be replaced with functionality from tracking module
    :param clip: bpy MovieClip
    :param measurement_noise_cov: Kalman filter measurement noise covariance scale
    :param process_noise_cov: Kalman filter process noise covariance scale
    """

def filter_3D(bpy_obj: object, measurement_noise_cov: float = 0.01, process_noise_cov: float = 0.0001) -> None:
    """
    DEPRECATED -> To be replaced with functionality from tracking module
    :param bpy_obj: Target object to apply smoothing effect
    :param measurement_noise_cov: Kalman filter measurement noise covariance scale
    :param process_noise_cov: Kalman filter process noise covariance scale
    """

def imread(arg0: str) -> Mat:
    """
    Read image from file
    :param arg0: Path to image
    """
def imshow(arg0: str, arg1: Mat) -> None:
    """
    Display image in named window
    :param arg0: Window name
    :param arg1: Image to display
    """
def imwrite(arg0: str, arg1: Mat) -> None:
    """
    Write image to file
    :param arg0: Output path
    :param arg1: Image to export
    """
def resize_img(arg0: Mat, arg1: Tuple[int, int]) -> Mat:
    """
    Resize image to new dimensions
    :param arg0: Input image
    :param arg1: New dimensions in (width, height)
    """
def solve_cameras(clip_a: object, clip_b: object, pose_tracks_a: object, pose_tracks_b: object, cam_mtx_a: object, cam_mtx_b: object, dist_coeffs_a: object, dist_coeffs_b: object, conf_thresh: float = ...) -> object:
    """
    DEPRECATED! To be removed following separation of bpy calls from core module
    :param clip_a: bpy MovieClip to use as anchor
    :param clip_b: bpy MovieClip to use as target
    :param pose_tracks_a: bpy property tree for anchor poses
    :param pose_tracks_b: bpy property tree for target poses
    :param cam_mtx_a: Camera matrix for anchor view
    :param cam_mtx_b: Camera matrix for target view
    :param dist_coeffs_a: Distortion coefficients for anchor view
    :param dist_coeffs_b: Distortion coefficients for target view
    :param conf_thresh: Confidence threshold for pose joints
    """
def triangulate_points(clips: object, pose_tracks_list: object, cam_matrices: object, dist_vectors: object, bpy_scene: object, bpy_data: object, use_all_tracks: bool = ..., non_destructive: bool = ...) -> None:
    """
    DEPRECATED! To be removed following separation of bpy calls from core module
    :param clips: All clips used in calculations
    :param pose_tracks_list: All pose property trees used in calculations
    :param cam_matrices: All camera matrices used in calculations
    :param dist_vectors: All distortion coefficient vectors used in calculations
    :param bpy_scene: Scene to write data to
    :param bpy_data: blend file data tree
    :param use_all_tracks: True if all tracks are to be triangulated
    :param non_destructive: True if existing keyframes are to be kept
    """
def waitKey(arg0: int) -> int:
    """
    Wait for a given period of time or until a key is pressed, whichever happens first
    :param arg0: Time to wait in milliseconds. If 0 will wait indefinitely.
    """
def z_thresh_2D(clip: object, max_z: float = ..., max_t_diff: int = ...) -> None:
    """
    DEPRECATED! To be removed following separation of bpy calls from core module.
    Not entirely sure what this does. I kinda forgot.
    """

def model_path(model_file: str) -> str:
    """
    Path shortcut to model directory located within MotionEngine package
    :param model_file: Path to model relative to model directory root
    """
