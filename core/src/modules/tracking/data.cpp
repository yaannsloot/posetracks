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

#include "data.hpp"
#include <numeric>

TDataPair find_common_data(const TrackingData& data_a, const TrackingData& data_b) {
	TrackingData output_a;
	TrackingData output_b;

	// Find pose intersections
	auto common_pose_frames = findCommonKeys(data_a.poses, data_b.poses);
	for (auto& f_id : common_pose_frames) {
		auto& id_map_a = data_a.poses.at(f_id);
		auto& id_map_b = data_b.poses.at(f_id);
		auto common_ids = findCommonKeys(id_map_a, id_map_b);
		PoseIDMap new_map_a;
		PoseIDMap new_map_b;
		for (auto& p_id : common_ids) {
			auto& pose_a = id_map_a.at(p_id);
			auto& pose_b = id_map_b.at(p_id);
			auto common_joints = findCommonKeys(pose_a.joints, pose_b.joints);
			Pose new_pose_a;
			Pose new_pose_b;
			for (auto& j_id : common_joints) {
				new_pose_a[j_id] = pose_a.joints.at(j_id);
				new_pose_b[j_id] = pose_b.joints.at(j_id);
			}
			if (!common_joints.empty()) {
				new_map_a[p_id] = new_pose_a;
				new_map_b[p_id] = new_pose_b;
			}
		}
		if (!new_map_a.empty() && !new_map_b.empty()) {
			output_a.poses[f_id] = new_map_a;
			output_b.poses[f_id] = new_map_b;
		}
	}

	// Find detection_intersections
	auto common_detection_frames = findCommonKeys(data_a.detections, data_b.detections);
	for (auto& f_id : common_detection_frames) {
		auto& id_map_a = data_a.detections.at(f_id);
		auto& id_map_b = data_b.detections.at(f_id);
		auto common_ids = findCommonKeys(id_map_a, id_map_b);
		DetectionIDMap new_map_a;
		DetectionIDMap new_map_b;
		for (auto& d_id : common_ids) {
			new_map_a[d_id] = id_map_a.at(d_id);
			new_map_b[d_id] = id_map_b.at(d_id);
		}
		if (!new_map_a.empty() && !new_map_b.empty()) {
			output_a.detections[f_id] = new_map_a;
			output_b.detections[f_id] = new_map_b;
		}
	}

	// Find tag intersections
	auto common_tag_frames = findCommonKeys(data_a.tags, data_b.tags);
	for (auto& f_id : common_tag_frames) {
		auto& id_map_a = data_a.tags.at(f_id);
		auto& id_map_b = data_b.tags.at(f_id);
		auto common_ids = findCommonKeys(id_map_a, id_map_b);
		TagIDMap new_map_a;
		TagIDMap new_map_b;
		for (auto& t_id : common_ids) {
			new_map_a[t_id] = id_map_a.at(t_id);
			new_map_b[t_id] = id_map_b.at(t_id);
		}
		if (!new_map_a.empty() && !new_map_b.empty()) {
			output_a.tags[f_id] = new_map_a;
			output_b.tags[f_id] = new_map_b;
		}
	}

	return TDataPair(output_a, output_b);
}

TrackedPoints TrackingData::to_points(bool reduce_tags) {
	TrackedPoints result;

	// Pose points
	for (auto& ppp : this->poses) {
		for (auto& pp : ppp.second) {
			auto& p = pp.second;
			for (auto& jj : p.joints) {
				auto& j = jj.second;
				result.push_back(cv::Point2f(static_cast<float>(j.pt.x), static_cast<float>(j.pt.y)));
			}
		}
	}

	// Detection points
	for (auto& ddd : this->detections) {
		for (auto& dd : ddd.second) {
			auto& d = dd.second;
			result.push_back(cv::Point2f(
				static_cast<float>(d.bbox.x + d.bbox.width / 2),
				static_cast<float>(d.bbox.y + d.bbox.height / 2)
			));
		}
	}

	// Tag points
	for (auto& ttt : this->tags) {
		for (auto& tt : ttt.second) {
			auto& t = tt.second;
			if (reduce_tags) {
				auto center = t[0] + t[1] + t[2] + t[3];
				center /= 4;
				result.push_back(cv::Point2f(static_cast<float>(center.x), static_cast<float>(center.y)));
			}
			else {
				for (int i = 0; i < 4; ++i) {
					result.push_back(cv::Point2f(static_cast<float>(t[i].x), static_cast<float>(t[i].y)));
				}
			}
		}
	}

	return result;
}

void TrackingData::set_pose(const int frame, const std::string name, const Pose& pose)
{
	this->poses[frame][name] = pose;
}

void TrackingData::set_joint(const int frame, const std::string name, const int id, const Joint& joint)
{
	this->poses[frame][name][id] = joint;
}

void TrackingData::set_joint(const int frame, const std::string name, const int id, const cv::Point2d& pt, const double prob)
{
	this->poses[frame][name][id] = Joint(pt, prob);
}

void TrackingData::set_joint(const int frame, const std::string name, const int id, const double x, const double y, const double prob)
{
	this->poses[frame][name][id] = Joint(x, y, prob);
}

void TrackingData::set_detection(const int frame, const std::string name, const Detection& det)
{
	this->detections[frame][name] = det;
}

void TrackingData::set_tag(const int frame, const int id, const Tag& tag)
{
	this->tags[frame][id] = tag;
}

void Rt::invert()
{
	from4x4(to4x4().inv(cv::DECOMP_SVD));
}

Mat4x4 Rt::to4x4()
{
	Mat4x4 T_out = Mat4x4::eye();
	for (int i = 0; i < 3; ++i) {
		T_out(i, 3) = t(i);
		for (int j = 0; j < 3; ++j) {
			T_out(i, j) = R(i, j);
		}
	}
	return T_out;
}

void Rt::from4x4(const Mat4x4& src) {
	for (int i = 0; i < 3; ++i) {
		t(i) = src(i, 3);
		for (int j = 0; j < 3; ++j) {
			R(i, j) = src(i, j);
		}
	}
}

bool Rt::is_identity() {
	Mat4x4 T_orig = to4x4();
	Mat4x4 T_identity = Rt().to4x4();
	std::vector<int> diff(16, 0);
	for (int i = 0; i < 4; ++i) {
		for (int j = 0; j < 4; ++j) {
			if (T_orig(i, j) != T_identity(i, j))
				diff[(size_t)i * 4 + (size_t)j] = 1;
		}
	}
	return std::accumulate(diff.begin(), diff.end(), 0) == 0;
}

std::vector<float> Kk::dist_vector() const {
	std::vector<float> out(5, 0);
	out[0] = static_cast<float>(k(0));
	out[1] = static_cast<float>(k(1));
	out[4] = static_cast<float>(k(2));
	return out;
}
