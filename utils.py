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
import math
import mathutils


def set_lock_on_tracks(tracks, lock):
    for track in tracks:
        track.lock = lock


def set_hidden_on_tracks(tracks, hide):
    for track in tracks:
        track.hide = hide


def set_select_tracks(tracks, select):
    for track in tracks:
        track.select = select


def check_list(item, collection):
    for i in collection:
        if item == i:
            return True
    return False


def get_cv_camera_intrinsics(bpy_movie_clip):
    clip_cam_settings = bpy_movie_clip.tracking.camera
    clip_size = bpy_movie_clip.size

    k1 = clip_cam_settings.k1
    k2 = clip_cam_settings.k2
    k3 = clip_cam_settings.k3

    # Calculate focal length in pixels
    fx = clip_cam_settings.focal_length * clip_size[0] / clip_cam_settings.sensor_width
    fy = fx

    # Calculate the principal point coordinates (in pixels)
    cx = clip_size[0] / 2.0
    cy = clip_size[1] / 2.0

    # Create the camera matrix
    camera_matrix = [[fx, 0, cx],
                     [0, fy, cy],
                     [0, 0, 1]]

    distortion_vector = [k1, k2, 0, 0, k3]

    return camera_matrix, distortion_vector


def flip_rotation_matrix(matrix):
    flip = mathutils.Matrix.Rotation(math.radians(180), 4, 'Z') @ mathutils.Matrix.Rotation(math.radians(180), 4, 'Y')
    return matrix @ flip


def get_relative_location(parent_object, child_object):
    # Get the world transformation matrices of the parent and child objects
    parent_matrix_world = parent_object.matrix_world
    child_matrix_world = child_object.matrix_world

    # Compute the child object's transformation matrix relative to the parent
    relative_matrix = parent_matrix_world.inverted() @ child_matrix_world

    # Extract the relative location from the transformation matrix
    relative_location = relative_matrix.to_translation()

    return relative_location
