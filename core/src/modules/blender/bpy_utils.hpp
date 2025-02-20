/* Copyright (C) 2025 Ian Sloat
* Licensed under the GNU GPLv3 or later. See <https://www.gnu.org/licenses/>. */

#pragma once

#include "bpy_types.hpp"
#include "bpy_data.hpp"
#include "../dnn/dnn.hpp"
#include "../tracking/data.hpp"

const Mat4x4 flip_mtx = {
	1, -1, -1, 1,
	-1, 1, 1, -1,
	-1, 1, 1, -1,
	1, 1, 1, 1
};

std::vector<std::string> split_str(const std::string& str, const char delim = '.');

const std::string join_string(std::vector<std::string>::iterator begin, std::vector<std::string>::iterator end, const char delim = '.');

void set_pose_sources(const std::vector<std::string>& sources);

void set_tag_sources(const std::vector<std::string>& sources);

Tag marker_to_tag(const MovieTrackingMarker marker, const int width, const int height);

Detection marker_to_detection(const MovieTrackingMarker marker, const int width, const int height);

Joint marker_to_joint(const MovieTrackingMarker marker, const int width, const int height);

TrackingData clip_tracking_data(const MovieClip clip, const double joint_conf_thresh = 0, const bool filter_locked = false, const bool filter_selected = false);

std::vector<MovieTrackingTrack> get_selected_tracks(MovieTrackingObject object);

Kk get_clip_Kk(MovieClip clip);

Rt get_obj_Rt(PyBObject obj, bool apply_flip = false, bool invert = true);

PyBCollection resolve_collection_path(const std::vector<std::string>& collection_path, bool make_collections = true);

PyBObject get_empty(const std::string& name, const std::vector<std::string>& collection_path);

PyBObject prepare_camera_for_clip(const std::string& clip_name);
