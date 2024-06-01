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

from . import dnn, Pointf, Point3D
from typing import Dict, List, Tuple, overload

class Mat3x1:
    def __init__(self) -> None:
        """
        Create a new 3x1 column matrix
        """
    def __getitem__(self, row: int) -> float: ...
    def __setitem__(self, row: int, value: float) -> None: ...
    def __str__(self) -> str: ...

class Mat1x3:
    def __init__(self) -> None:
        """
        Create a new 1x3 row matrix
        """
    def __getitem__(self, row: int) -> float: ...
    def __setitem__(self, row: int, value: float) -> None: ...
    def __str__(self) -> str: ...

class Mat3x3:
    def __init__(self) -> None:
        """
        Create a new 3x3 matrix
        """
    def __getitem__(self, pos: Tuple[int, int]) -> float: ...
    def __setitem__(self, pos: Tuple[int, int], value: float) -> None: ...
    def __str__(self) -> str: ...

class Mat4x4:
    def __init__(self) -> None:
        """
        Create a new 4x4 matrix
        """
    def __getitem__(self, pos: Tuple[int, int]) -> float: ...
    def __setitem__(self, pos: Tuple[int, int], value: float) -> None: ...
    def __str__(self) -> str: ...

class Rt:
    R: Mat3x3
    t: Mat3x1
    def __init__(self):
        """
        Create a new Rt transformation pair
        """
    def invert(self) -> None:
        """
        Invert this transformation
        """
    def to4x4(self) -> Mat4x4:
        """
        Compose a 4x4 transformation matrix from this Rt pair
        """
    def from4x4(self, src: Mat4x4) -> None:
        """
        Decomposes a 4x4 transformation matrix and assigns the resulting components to this Rt pair
        :param src: The source 4x4 matrix
        """
    def is_identity(self) -> bool:
        """
        Checks if this Rt pair is identical to the identity transformation.
        If true, this means that the transformation has not been modified,
        which can indicate a failure in certain operations.
        :return: True if identical to identity transformation
        """

class Kk:
    K: Mat3x3
    k: Mat1x3
    def __init__(self):
        """
        Create a new object for storing camera intrinsic information
        """

class Tag3D:
    conf: float
    id: int
    @overload
    def __init__(self) -> None:
        """
        Create a new empty tag
        """
    @overload
    def __init__(self, arg0: int) -> None:
        """
        Create a new tag with a given tag id
        :param arg0: Tag ID
        """
    @overload
    def __init__(self, arg0: Union[Point, Tuple[float, float]], arg1: Union[Point, Tuple[float, float]],
                 arg2: Union[Point, Tuple[float, float]], arg3: Union[Point, Tuple[float, float]]) -> None:
        """
        Create a new tag with a given set of corner points.

        Corner points are ordered clockwise starting from the top left corner.
        :param arg0: First corner point
        :param arg1: Second corner point
        :param arg2: Third corner point
        :param arg3: Fourth corner point
        """
    @overload
    def __init__(self, arg0: int, arg1: Union[Point, Tuple[float, float]], arg2: Union[Point, Tuple[float, float]],
                 arg3: Union[Point, Tuple[float, float]], arg4: Union[Point, Tuple[float, float]]) -> None:
        """
        Create a new tag with a given tag id and set of corner points.

        Corner points are ordered clockwise starting from the top left corner.
        :param arg0: Tag ID
        :param arg1: First corner point
        :param arg2: Second corner point
        :param arg3: Third corner point
        :param arg4: Fourth corner point
        """
    def __getitem__(self, arg0: int) -> Point: ...
    def __iter__(self) -> typing.Iterator[Point]: ...
    def __setitem__(self, arg0: int, arg1: Union[Point, Tuple[float, float]]) -> None: ...

class TrackingData:
    detections: Dict[int, Dict[str, dnn.Detection]]
    poses: Dict[int, Dict[str, dnn.Pose]]
    tags: Dict[int, Dict[int, dnn.Tag]]
    def __init__(self) -> None:
        """
        Create a new tracking data block
        """
    def to_points(self, reduce_boxes: bool = True, reduce_tags: bool = False) -> List[Pointf]:
        """
        Convert all tracking data to a list of points
        :param reduce_boxes: If true, use center point for detections
        :param reduce_tags: Tf true, use center point for tags
        :return:
        """
		
class TrackingData3D:
    detections: Dict[int, Dict[str, Tuple[int, Point3D]]]
    poses: Dict[int, Dict[str, Dict[int, Point3D]]]
    tags: Dict[int, Dict[int, Tag3D]]
    def __init__(self) -> None:
        """
        Create a new 3D tracking data block
        """

def find_common_data(arg0: TrackingData, arg1: TrackingData) -> Tuple[TrackingData, TrackingData]:
    """
    Create a pair of tracking data blocks that contain common keys\
    :param arg0: First data block
    :param arg1: Second data block
    """

@overload
def solve_static_pair(t_data_a: TrackingData, t_data_b: TrackingData, cam_Kk_a: Kk, cam_Kk_b: Kk) -> Rt:
    """
    Solve the position of a camera relative to another using tracked point correspondences
    :param t_data_a: Tracking data from the origin camera
    :param t_data_b: Tracking data from the target camera
    :param cam_Kk_a: Camera matrix and distortion data from the origin camera
    :param cam_Kk_b: Camera matrix and distortion data from the target camera
    :return: Transformation of the target camera relative to the origin camera
    """

@overload
def solve_static_pair(t_points_a: List[Pointf], t_points_b: List[Pointf], cam_Kk_a: Kk, cam_Kk_b: Kk) -> Rt:
    """
    Solve the position of a camera relative to another using tracked point correspondences
    :param t_points_a: Tracked points from the origin camera
    :param t_points_b: Tracked points from the target camera
    :param cam_Kk_a: Camera matrix and distortion data from the origin camera
    :param cam_Kk_b: Camera matrix and distortion data from the target camera
    :return: Transformation of the target camera relative to the origin camera
    """

def solve_static_set(t_data: List[TrackingData], cam_Kk: List[Kk]) -> List[Rt]:
    """
    Solve the positions for a set of cameras relative to an anchor view using tracked point correspondences
    :param t_data: List of tracking data. The first element in the list is treated as tracking data from the anchor
    :param cam_Kk: List of camera intrinsic information. Must be the same size as t_data
    :return: A list of transformations for each camera relative to the first.
    """
	
def solve_camera_with_tag(observed_tag: dnn.Tag, cam_Kk: Kk, square_length: double=1.0) -> Rt:
	"""
	Solve a camera's position using an observed tag as the origin
	:param observed_tag: The observed reference tag
	:param cam_Kk: Camera matrix and distortion data from the target camera
	:param square_length: Side length of the observed tag
	:return: The estimated 3D position of the camera relative to the observed tag
	"""
	
def triangulate_static(t_data: List[TrackingData], cam_Kk: List[Kk], cam_Rt: List[Rt]) -> TrackingData3D:
	"""
	Triangulate points for a set of cameras. 
	The transformations of these cameras must be calculated ahead of time for the solver to work properly.
	:param t_data: List of tracking data. The first element in the list is treated as tracking data from the anchor
    :param cam_Kk: List of camera intrinsic information. Must be the same size as t_data
    :return: Triangulated tracking data
	"""
