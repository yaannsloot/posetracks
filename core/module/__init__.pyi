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
from . import threading
from . import tracking
import numpy as np

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

models: Dict[str, Dict[str, Dict[str, ...]]]
"""
Dictionary of models bundled with module. Includes various performance metrics.
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
def waitKey(arg0: int) -> int:
    """
    Wait for a given period of time or until a key is pressed, whichever happens first
    :param arg0: Time to wait in milliseconds. If 0 will wait indefinitely.
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
