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

#include "bpy_utils.hpp"

std::vector<std::string> pose_sources;
std::vector<std::string> tag_sources;

void set_pose_sources(const std::vector<std::string>& sources)
{
	pose_sources = sources;
}

void set_tag_sources(const std::vector<std::string>& sources)
{
	tag_sources = sources;
}

me::dnn::Tag marker_to_tag(const MovieTrackingMarker marker, const int width, const int height)
{
	me::dnn::Tag output;
	const Corners corners = marker.pattern_corners();
	const float* pos = marker.pos();
	for (size_t i = 0; i < 4; ++i) {
		output[3 - i].x = (corners[i][0] + pos[0]) * width;
		output[3 - i].y = height - (corners[i][1] + pos[1]) * height;
	}
	return output;
}

struct MarkerCorners {
	float corners[4][2];
	float min[2];
	float max[2];
};

MarkerCorners get_corners(const MovieTrackingMarker marker, const int width, const int height) {
	MarkerCorners ret;
	const Corners corners = marker.pattern_corners();
	const float* pos = marker.pos();
	ret.corners[0][0] = (corners[0][0] + pos[0]) * width;
	ret.corners[0][1] = (corners[0][1] + pos[1]) * height;
	ret.min[0] = ret.corners[0][0];
	ret.min[1] = ret.corners[0][1];
	ret.max[0] = ret.corners[0][0];
	ret.max[1] = ret.corners[0][1];
	for (size_t i = 1; i < 4; ++i) {
		ret.corners[i][0] = (corners[i][0] + pos[0]) * width;
		ret.corners[i][1] = (corners[i][1] + pos[1]) * height;
		ret.min[0] = std::min(ret.corners[i][0], ret.min[0]);
		ret.min[1] = std::min(ret.corners[i][1], ret.min[1]);
		ret.max[0] = std::max(ret.corners[i][0], ret.max[0]);
		ret.max[1] = std::max(ret.corners[i][1], ret.max[1]);
	}
	return ret;
}

me::dnn::Detection marker_to_detection(const MovieTrackingMarker marker, const int width, const int height)
{
	MarkerCorners corners = get_corners(marker, width, height);
	float bbox_width = corners.max[0] - corners.min[0];
	float bbox_height = corners.max[1] - corners.min[1];
	float bbox_center_x = corners.min[0] + bbox_width / 2;
	float bbox_center_y = height - (corners.min[1] + bbox_height / 2);
	float bbox_tl_x = bbox_center_x - bbox_width / 2;
	float bbox_tl_y = bbox_center_y - bbox_height / 2;
	return me::dnn::Detection(0, cv::Rect2d(bbox_tl_x, bbox_tl_y, bbox_width, bbox_height), 1);
}

me::dnn::Joint marker_to_joint(const MovieTrackingMarker marker, const int width, const int height)
{
	MarkerCorners c = get_corners(marker, width, height);
	float area = 0;
	for (size_t i = 0; i < 4; ++i) {
		size_t i_a = i % 4;
		size_t i_b = (i + 1) % 4;
		float width = c.corners[i_b][0] - c.corners[i_a][0];
		float height = ((c.corners[i_a][1] - c.min[1]) + (c.corners[i_b][1] - c.min[1])) / 2;
		area += width * height;
	}
	area = std::min(100.0f, std::max(std::abs(area), 0.0f)) / 100.0f;
	const float* pos = marker.pos();
	float x = pos[0] * width;
	float y = height - pos[1] * height;
	return me::dnn::Joint(x, y, area);
}

std::vector<std::string> split_str(const std::string& str, const char delim = '.') {
	std::vector<std::string> strings;
	std::string last;
	for (const char& c : str) {
		if (c == delim) {
			if (!last.empty())
				strings.push_back(last);
			last.clear();
			continue;
		}
		last += c;
	}
	if (!last.empty())
		strings.push_back(last);
	return strings;
}

const std::string join_string(std::vector<std::string>::iterator begin, std::vector<std::string>::iterator end, const char delim = '.') {
	std::string out;
	auto last_it = std::prev(end);
	for (; begin != end; ++begin) {
		out += *begin;
		if (begin != last_it)
			out += delim;
	}
	return out;
}

struct TrackPoseInfo {
	std::string pose;
	int id;
	bool valid = false;
};

TrackPoseInfo get_track_pose_info(const MovieTrackingTrack track) {
	auto split_name = split_str(track.name());
	TrackPoseInfo info;
	if (split_name.size() < 3 || std::find(pose_sources.begin(), pose_sources.end(), split_name.rbegin()[1]) == pose_sources.end())
		return info;
	try {
		info.id = std::stoi(split_name.back());
	}
	catch (...) {
		return info;
	}
	info.pose = join_string(split_name.begin(), std::prev(split_name.end(), 2));
	info.pose = info.pose + '.' + split_name.rbegin()[1];
	info.valid = true;
	return info;
}

struct TrackTagInfo {
	int id;
	int source_id;
	bool valid = false;
};

TrackTagInfo get_track_tag_info(const MovieTrackingTrack track) {
	TrackTagInfo info;
	auto split_name = split_str(track.name());
	auto source_it = std::find(tag_sources.begin(), tag_sources.end(), split_name[1]);
	if (split_name.size() != 3 || split_name[0] != "Tag" || source_it == tag_sources.end())
		return info;
	try {
		info.id = std::stoi(split_name.back());
	}
	catch (...) {
		return info;
	}
	info.valid = true;
	info.source_id = static_cast<int>(std::distance(tag_sources.begin(), source_it)) * 1000 + info.id;
	return info;
}

me::tracking::TrackingData clip_tracking_data(const MovieClip clip, const double joint_conf_thresh, const bool filter_locked)
{
	me::tracking::TrackingData data;
	const int* lastsize = clip.last_size();
	const int start_frame = clip.start_frame();
	const MovieTracking tracking = clip.tracking();
	const auto tracks = tracking.objects().first().tracks(); // First object will always be for camera tracking
	for (MovieTrackingTrack track = tracks.first(); !track.is_null(); track = track.next()) {
		std::string name = track.name();
		if (filter_locked && !(track.flag() & TRACK_LOCKED))
			continue;
		TrackPoseInfo pose_info = get_track_pose_info(track);
		TrackTagInfo tag_info = get_track_tag_info(track);
		int markersnr = track.markersnr();
		for (int m = 0; m < markersnr; ++m) {
			MovieTrackingMarker marker = track.marker(m);
			int framenr = marker.framenr();
			if (pose_info.valid) {
				auto joint = marker_to_joint(marker, lastsize[0], lastsize[1]);
				if (joint.prob < joint_conf_thresh)
					continue;
				data.set_joint(framenr + start_frame - 1, pose_info.pose, pose_info.id, joint);
			}
			else if (tag_info.valid) {
				auto tag = marker_to_tag(marker, lastsize[0], lastsize[1]);
				tag.id = tag_info.id;
				data.set_tag(framenr + start_frame - 1, tag_info.source_id, tag);
			}
			else {
				data.set_detection(framenr + start_frame - 1, name, marker_to_detection(marker, lastsize[0], lastsize[1]));
			}
		}
	}
	return data;
}

std::vector<MovieTrackingTrack> get_selected_tracks(MovieTrackingObject object) {
	auto tracks = object.tracks();
	std::vector<MovieTrackingTrack> out;
	for (MovieTrackingTrack track = tracks.first(); !track.is_null(); track = track.next()) {
		if (track.flag() & TRACK_SELECT)
			out.push_back(track);
	}
	return out;
}

me::tracking::Kk get_clip_Kk(MovieClip clip) {
	me::tracking::Kk result;
	MovieTrackingCamera settings = clip.tracking().camera();
	const int* size = clip.last_size();
	result.k(0) = static_cast<double>(settings.k1());
	result.k(1) = static_cast<double>(settings.k2());
	result.k(2) = static_cast<double>(settings.k3());
	double f = static_cast<double>(settings.focal() * size[0] / settings.sensor_width());
	result.K(0, 0) = f;
	result.K(1, 1) = f;
	result.K(0, 2) = static_cast<double>(size[0]) / 2;
	result.K(1, 2) = static_cast<double>(size[1]) / 2;
	return result;
}

me::tracking::Rt get_obj_Rt(Object obj, bool apply_flip, bool invert) {
	me::tracking::Mat4x4 obj_mat;
	Mat obj_world = obj.obmat();
	for (int i = 0; i < 4; ++i) {
		for (int j = 0; j < 4; ++j) {
			obj_mat(i, j) = static_cast<double>(obj_world[i][j]);
		}
	}
	if (apply_flip)
		obj_mat = obj_mat.mul(flip_mtx);
	me::tracking::Rt result;
	result.from4x4(obj_mat);
	if (invert)
		result.invert();
	return result;
}