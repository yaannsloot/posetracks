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

from . import model_load
from . import prune_data
from . import pose_estimation
from . import create_tracks
from . import calculate_statistics
from . import report_status
from . import clear_temp_tracks
from . import clear_all_tracks

ALL_CLASSES = (model_load.CLASSES + prune_data.CLASSES + pose_estimation.CLASSES + create_tracks.CLASSES
               + calculate_statistics.CLASSES + report_status.CLASSES + clear_temp_tracks.CLASSES
               + clear_all_tracks.CLASSES)
