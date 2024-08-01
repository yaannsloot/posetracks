/*
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
*/

#pragma once

#include "bpy_types.hpp"
#include "../dnn/dnn.hpp"
#include "../tracking/data.hpp"

const me::tracking::Mat4x4 flip_mtx = {
	1, -1, -1, 1,
	-1, 1, 1, -1,
	-1, 1, 1, -1,
	1, 1, 1, 1
};

void set_pose_sources(const std::vector<std::string>& sources);

void set_tag_sources(const std::vector<std::string>& sources);

me::dnn::Tag marker_to_tag(const MovieTrackingMarker marker, const int width, const int height);

me::dnn::Detection marker_to_detection(const MovieTrackingMarker marker, const int width, const int height);

me::dnn::Joint marker_to_joint(const MovieTrackingMarker marker, const int width, const int height);

me::tracking::TrackingData clip_tracking_data(const MovieClip clip, const double joint_conf_thresh = 0, const bool filter_locked = false);

std::vector<MovieTrackingTrack> get_selected_tracks(MovieTrackingObject object);

me::tracking::Kk get_clip_Kk(MovieClip clip);

me::tracking::Rt get_obj_Rt(Object obj, bool apply_flip = false, bool invert = true);
