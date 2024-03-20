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

from . import dnn, Pointf
from typing import Dict, List, Tuple

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

def find_common_data(arg0: TrackingData, arg1: TrackingData) -> Tuple[TrackingData, TrackingData]:
    """
    Create a pair of tracking data blocks that contain common keys\
    :param arg0: First data block
    :param arg1: Second data block
    """
