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

from typing import Union, ClassVar, List, Tuple, Iterator, Dict, overload
from . import crypto
from . import data
from . import dnn
from . import io
from . import tracking
import numpy as np

DICT_4X4: TagDictionary
DICT_5X5: TagDictionary
DICT_6X6: TagDictionary
DICT_7X7: TagDictionary
DICT_APRILTAG_16h5: TagDictionary
DICT_APRILTAG_25h9: TagDictionary
DICT_APRILTAG_36h10: TagDictionary
DICT_APRILTAG_36h11: TagDictionary
DICT_ARUCO_ORIGINAL: TagDictionary

VER_2_93_0: BlenderVersion
VER_2_93_4: BlenderVersion
VER_3_0_0: BlenderVersion
VER_3_1_0: BlenderVersion
VER_3_2_0: BlenderVersion
VER_3_3_0: BlenderVersion
VER_3_4_0: BlenderVersion
VER_3_5_0: BlenderVersion
VER_3_6_0: BlenderVersion
VER_3_6_8: BlenderVersion
VER_4_0_0: BlenderVersion
VER_4_1_0: BlenderVersion
VER_4_1_1: BlenderVersion
VER_4_2_0: BlenderVersion

models: Dict[str, Dict[str, Dict[str, ...]]]
"""
Dictionary of models bundled with module. Includes various performance metrics.
"""

class BlenderVersion:
    __members__: ClassVar[dict] = ...  # read-only
    VER_2_93_0: ClassVar[TagDictionary] = ...
    VER_2_93_4: ClassVar[TagDictionary] = ...
    VER_3_0_0: ClassVar[TagDictionary] = ...
    VER_3_1_0: ClassVar[TagDictionary] = ...
    VER_3_2_0: ClassVar[TagDictionary] = ...
    VER_3_3_0: ClassVar[TagDictionary] = ...
    VER_3_4_0: ClassVar[TagDictionary] = ...
    VER_3_5_0: ClassVar[TagDictionary] = ...
    VER_3_6_0: ClassVar[TagDictionary] = ...
    VER_3_6_8: ClassVar[TagDictionary] = ...
    VER_4_0_0: ClassVar[TagDictionary] = ...
    VER_4_1_0: ClassVar[TagDictionary] = ...
    VER_4_1_1: ClassVar[TagDictionary] = ...
    VER_4_2_0: ClassVar[TagDictionary] = ...
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

def clip_tracking_data(clip, joint_conf_thresh: float = 0, filter_locked: bool = False) -> tracking.TrackingData:
    """
    Retrieve tracking data from the provided movie clip
    :param joint_conf_thresh: Minimum joint confidence score
    :param filter_locked: If true, will ignore tracks that are not locked
    :return: MotionEngine compatible tracking data object
    """

class Mat:
    """
    Reference class for cv::Mat objects
    """
    def __init__(self) -> None:
        """
        Create a new empty matrix
        """
    def from_array(self, im_data: np.ndarray) -> None:
        """
        Copy numpy array data into mat
        :param im_data: Source data
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
    def __init__(self, x: float, y: float) -> None:
        """
        Create a new point
        :param x: X coordinate
        :param y: Y coordinate
        """
	@overload
    def __init__(self, coords: Tuple[float, float]) -> None:
        """
        Create a new point
        :param coords: Tuple with X, Y coordinates
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
    def __init__(self, x: float, y: float) -> None:
        """
        Create a new point. The coordinates are stored internally as single precision FP32 values
        :param x: X coordinate
        :param x: Y coordinate
        """
	@overload
    def __init__(self, coords: Tuple[float, float]) -> None:
        """
        Create a new point. The coordinates are stored internally as single precision FP32 values
        :param coords: Tuple with X, Y coordinates
        """
    def __iter__(self) -> Iterator[float]: ...
    def __getitem__(self, item) -> float: ...
    def __setitem__(self, key, value) -> None: ...
    def __len__(self) -> int: ...
	
class Point3D:
    """
    Reference class for cv::Point3d objects
    """
    x: float
    y: float
	z: float
    @overload
    def __init__(self) -> None:
        """
        Create a new point
        """
    @overload
    def __init__(self, x: float, y: float, z: float) -> None:
        """
        Create a new point
        :param x: X coordinate
        :param y: Y coordinate
		:param z: Z coordinate
        """
    def __iter__(self) -> Iterator[float]: ...
    def __getitem__(self, item) -> float: ...
    def __setitem__(self, key, value) -> None: ...
    def __len__(self) -> int: ...

class Pointf3D:
    """
    Reference class for cv::Point3f objects
    """
    x: float
    y: float
	z: float
    @overload
    def __init__(self) -> None:
        """
        Create a new point. The coordinates are stored internally as single precision FP32 values
        """
    @overload
    def __init__(self, x: float, y: float, z: float) -> None:
        """
        Create a new point. The coordinates are stored internally as single precision FP32 values
        :param x: X coordinate
        :param y: Y coordinate
		:param z: Z coordinate
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
    def __init__(self, x: float, y: float, width: float, height: float) -> None:
        """
        Create a new bounding box
        :param x: X coordinate of top left corner
        :param y: Y coordinate of top left corner
        :param width: Width of bounding box
        :param height: Height of bounding box
        """
    @overload
    def __init__(self, src: Rect) -> None:
        """
        Create a new bounding box by copying an existing one
        :param src: Source bounding box
        """
    def area(self) -> float:
        """
        Get the area of this bounding box
        """
    def br(self) -> Point:
        """Get the bottom right corner of this bounding box"""
    def contains(self, pt: Point) -> bool:
        """
        Check whether a point is inside this bounding box
        :param pt: Point to check
        """
    def empty(self) -> bool:
        """True if empty i.e. the box has an area of 0"""
    def size(self) -> tuple[float, float]:
        """Size of bounding box in (width, height)"""
    def tl(self) -> Point:
        """Get the top left corner of this bounding box"""
    def __eq__(self, other: Rect) -> bool:
        """Check if the provided bounding box has the same dimensions as this one"""
    def __ne__(self, other: Rect) -> bool:
        """Check if the provided bounding box does not have the same dimensions as this one"""

class TagDictionary:
    __members__: ClassVar[dict] = ...  # read-only
    DICT_4X4: ClassVar[TagDictionary] = ...
    DICT_5X5: ClassVar[TagDictionary] = ...
    DICT_6X6: ClassVar[TagDictionary] = ...
    DICT_7X7: ClassVar[TagDictionary] = ...
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

def get_models(model_category: str, attributes: Union[str, List[str], None] = None,
               values: Union[Dict[str, ...], None] = None,
               sorting_criteria: Union[List[str], None] = None) -> Union[List[Tuple[str, Dict[str, ...]]], None]:
    """
    Function used to access the standard model collection included with MotionEngine

    Attributes filter:
    Searches for all models that contain the attribute or attributes specified.
    If None, will not filter for attributes.

    Values filter:
    If a specified value is a list, will search for all models with the specified
    attribute that contain AT LEAST these values.
    If None, will not filter for values.

    Sorting criteria:
    Specified attributes must be fully present in the filtered model list,
    else that attribute is ignored. If None, will not sort the output.

    :param model_category: Which model category to access
    :param attributes: Which attributes to filter for.
    :param values:  Which attribute value pairs to filter for.
    :param sorting_criteria: Which numerical attributes to use as sorting criteria for the output.
    :return: A list of tuples containing a model name and its attributes, or None if no models matched the specified filters
    """

def imread(path: str) -> Mat:
    """
    Read image from file
    :param path: Path to image
    """
def imshow(win_name: str, img: Mat) -> None:
    """
    Display image in named window
    :param win_name: Window name
    :param img: Image to display
    """
def imwrite(path: str, img: Mat) -> None:
    """
    Write image to file
    :param path: Output path
    :param img: Image to export
    """
def resize_img(img_src: Mat, dims: Tuple[int, int]) -> Mat:
    """
    Resize image to new dimensions
    :param img_src: Input image
    :param dims: New dimensions in (width, height)
    """
def waitKey(msec: int) -> int:
    """
    Wait for a given period of time or until a key is pressed, whichever happens first
    :param msec: Time to wait in milliseconds. If 0 will wait indefinitely.
    """
def model_path(model_file: str) -> str:
    """
    Path shortcut to model directory located within MotionEngine package
    :param model_file: Path to model relative to model directory root
    """

def rand_img_rgb(size: Tuple[int, int]) -> Mat:
    """
    Returns a randomly generated RGB image
    :param size: Size of the image
    """

def rand_img_gray(size: Tuple[int, int]) -> Mat:
    """
    Returns a randomly generated grayscale image
    :param size: Size of the image
    """

def set_compatibility_mode(ver: BlenderVersion) -> None:
    """
    Set the internal compatibility version for the core module
    """

def set_pose_sources(sources: List[str]) -> None:
    """
    Set the list of pose source names for the core module's name filter
    """

def set_tag_sources(sources: List[str]) -> None:
    """
    Set the list of tag source names for the core module's name filter
    """
