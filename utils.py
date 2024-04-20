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
import bpy
import math
import mathutils
from . import global_vars
from . import MotionEngine as me


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


def prepare_camera_for_clip(movie_clip: bpy.types.MovieClip, context: bpy.types.Context):
    """
    Ensure scene cameras exist for movie clip
    :param movie_clip: Reference clip
    :param context: Context from operator
    :return: A camera object with matching settings for sensor size and focal length.
    The source clip has been set as a background image.
    """
    scene = context.scene
    scene_root = scene.collection
    cam_name = movie_clip.name

    cam_data = bpy.data.cameras.get(cam_name)
    if cam_data is None:
        cam_data = bpy.data.cameras.new(cam_name)

    cam_obj = None
    for obj in scene.objects:
        if obj.data is cam_data:
            cam_obj = obj
            break
    if cam_obj is None:
        cam_obj = bpy.data.objects.new(cam_name, object_data=cam_data)

    scene_root.objects.link(cam_obj)

    cam_data.background_images.clear()

    cam_bg_img = cam_data.background_images.new()
    cam_bg_img.clip = movie_clip
    cam_bg_img.display_depth = 'FRONT'
    cam_bg_img.source = 'MOVIE_CLIP'
    cam_bg_img.alpha = 0.75
    cam_bg_img.clip_user.use_render_undistorted = True
    cam_bg_img.show_expanded = False

    cam_data.sensor_width = movie_clip.tracking.camera.sensor_width
    cam_data.lens = movie_clip.tracking.camera.focal_length

    return cam_obj


def get_joint_tracks(movie_clip: bpy.types.MovieClip):
    """
    Create a dictionary of all tracks that qualify as pose joints

    as defined by new spec

    Src_Det.Joint.pose_type.ID

    Src_Det can be any string, including strings that have . separators

    The only requirement is that the last two elements of the string equal

    [Joint, pose_type, ID]

    :param movie_clip: Clip to scan for tracks
    :return: Dictionary that maps Src_Det name to a dictionary of joint ids and joints
     as well as a list of qualifying tracks
    """
    track_dict = {}
    tracks = []

    for track in movie_clip.tracking.tracks:
        split_name = track.name.split('.')
        if len(split_name) < 4 or split_name[-3] != 'Joint' or split_name[-2] not in global_vars.pose_types:
            continue
        try:
            joint_id = int(split_name[-1])
        except ValueError:
            continue
        pose_name = ''
        for i in range(len(split_name) - 3):
            if pose_name != '':
                pose_name += '.' + split_name[i]
            else:
                pose_name = split_name[i]
        track_dict[pose_name][split_name[-2]][joint_id] = track
        tracks.append(track)

    return track_dict, tracks


def get_clip_poses(movie_clip: bpy.types.MovieClip, joint_conf_thresh=0.9):
    """
    Get all named poses on the provided clip
    :param movie_clip: Clip to extract data from
    :param joint_conf_thresh: Joint confidence threshold
    :return: Nested dictionary with mappings to frames and named poses
    """
    return {}


def get_clip_detections(movie_clip: bpy.types.MovieClip, include_user_tracks=False):
    """
    Get all named object detections on the provided clip
    :param movie_clip: Clip to extract data from
    :param include_user_tracks: If true, will convert user defined tracks to detections
    :return: Nested dictionary with mappings to frames and named detections
    """
    return {}


def get_clip_tags(movie_clip: bpy.types.MovieClip):
    """
    Get all tag detections on the provided clip
    :param movie_clip: Clip to extract data from
    :return: Nested dictionary with mappings to frames and tags
    """
    return {}


def get_clip_tracking_data(movie_clip: bpy.types.MovieClip, pose_joint_conf=0.9, include_user_tracks=False,
                           include_poses=True, include_detections=True, include_tags=True):
    """
    Get all clip tracking data and prepare it for use with MEPython
    :param movie_clip: Clip to extract data from
    :param pose_joint_conf: Pose joint confidence threshold
    :param include_user_tracks: If true, will convert user defined tracks to detections
    :param include_poses: If true, extract pose data
    :param include_detections: If true, extract object detections
    :param include_tags: If true, extract tag detections
    :return: MEPython TrackingData object
    """
    result = me.tracking.TrackingData()
    if include_poses:
        result.poses = get_clip_poses(movie_clip, pose_joint_conf)
    if include_detections:
        result.detections = get_clip_detections(movie_clip, include_user_tracks)
    if include_tags:
        result.tags = get_clip_tags(movie_clip)
    return result


def get_clip_Kk(movie_clip: bpy.types.MovieClip):
    """
    Create an MEPython camera intrinsics Kk object from camera information in a blender movie clip
    :param movie_clip: Clip to retrieve camera information from
    :return: MEPython Kk camera intrinsics object
    """
    cam_Kk = me.tracking.Kk()
    clip_cam_settings = movie_clip.tracking.camera
    clip_size = movie_clip.size

    # Set distortion coefficients
    cam_Kk.k[0] = clip_cam_settings.k1
    cam_Kk.k[1] = clip_cam_settings.k2
    cam_Kk.k[2] = clip_cam_settings.k3

    # Calculate focal length in pixels
    fx = clip_cam_settings.focal_length * clip_size[0] / clip_cam_settings.sensor_width
    fy = fx

    # Calculate the principal point coordinates (in pixels)
    cx = clip_size[0] / 2.0
    cy = clip_size[1] / 2.0

    # Copy camera matrix values
    cam_Kk.K[0, 0] = fx
    cam_Kk.K[1, 1] = fy
    cam_Kk.K[0, 2] = cx
    cam_Kk.K[1, 2] = cy

    return cam_Kk
