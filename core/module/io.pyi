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

from . import Mat
class FrameProvider:
    """
    A managed decoder that can retrieve images from a variety of source media
    """
    def __init__(self) -> None:
        """
        Create a new frame provider instance
        """
    def close(self) -> None:
        """
        Release the files loaded by this frame provider
        """
    def current_frame(self) -> int:
        """
        Get the current frame number of this frame provider
        """
    def fps(self) -> float:
        """
        Get the fps of the source media
        """
    def frame_count(self) -> int:
        """
        Get the total number of frames in the source media
        """
    def frame_size(self) -> tuple:
        """
        Get the frame dimensions of the source media
        """
    def get_fourcc(self) -> int:
        """
        Get the FourCC code of the source media
        """
    def get_fourcc_str(self) -> str:
        """
        Get the FourCC code of the source media as a string
        """
    def grab_frame(self, frame: Mat, frame_id: int, retry_count: int = 100) -> bool:
        """
        Grab a specific frame from the source media
        :param frame: Destination frame object
        :param frame_id: Source frame id
        :param retry_count: Number of retry attempts in case the first decode fails
        :return: True if the read was successful
        """
    def is_open(self) -> bool:
        """
        Check if this frame provider has media that is loaded and available
        :return:
        """
    def load(self, path: str, use_hw_accel: bool = False) -> bool:
        """
        Load source media into frame provider
        :param path: Path to source media
        :param use_hw_accel: Use hardware accelerated decoding if applicable
        """
    def next_frame(self, frame: Mat, retry_count: int = 100) -> bool:
        """
        Read the next available frame from the source media
        :param frame: Destination frame object
        :param retry_count: Number of retry attempts in case the first decode fails
        :return: True if the read was successful
        """
    def set_frame(self, frame_id: int) -> bool:
        """
        Set the frame position
        :param frame_id: New frame position
        :return: True if the position was changed successfully
        """

class ImageList(FrameProvider):
    """
    Frame provider that reads images from a directory that contains numbered image files
    """
    def __init__(self) -> None:
        """
        Create a new image list instance
        """

class Transcoder(FrameProvider):
    """
    Frame provider that reads images from a video file
    """
    def __init__(self) -> None:
        """
        Create a new transcoder instance
        """
