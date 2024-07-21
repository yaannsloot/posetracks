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

#include "triangulation.hpp"
#include <opencv2/sfm/triangulation.hpp>
#include <opencv2/sfm/projection.hpp>
#include <algorithm>
#include <numeric>
#include <execution>

template<typename T>
using ViewMapping = std::unordered_map<T, std::vector<size_t>>;

namespace me::tracking {

	TrackingData3D triangulateStatic(const std::vector<TrackingData>& t_data, const std::vector<Kk>& cam_Kk, const std::vector<Rt>& cam_Rt) 
	{
		if (t_data.size() != cam_Kk.size())
			throw std::invalid_argument("t_data.size() != cam_Kk.size()");

		if (t_data.size() != cam_Rt.size())
			throw std::invalid_argument("t_data.size() != cam_Rt.size()");

		TrackingData3D output;

		const size_t n_views = t_data.size();

		std::vector<cv::Mat> cam_P(n_views);
		std::vector<std::vector<float>> cam_dist_vectors(n_views);

		for (size_t i = 0; i < n_views; ++i) {
			cv::sfm::projectionFromKRt(cv::Mat::eye(3, 3, CV_64FC1), cam_Rt[i].R, cam_Rt[i].t, cam_P[i]);
			cam_dist_vectors[i] = cam_Kk[i].dist_vector();
		}

		std::set<int> target_frames;

		for (auto& c_dat : t_data) {
			for (const auto& ele : c_dat.poses) {
				target_frames.insert(ele.first);
			}
			for (const auto& ele : c_dat.detections) {
				target_frames.insert(ele.first);
			}
			for (const auto& ele : c_dat.tags) {
				target_frames.insert(ele.first);
			}
		}

		std::mutex lock;

		std::for_each(std::execution::par_unseq, target_frames.begin(), target_frames.end(), [&](int frame) {
			PoseIDMap3D frame_poses3D;
			DetectionIDMap3D frame_detections3D;
			TagIDMap3D frame_tags3D;

			ViewMapping<std::string> pose_mappings;
			ViewMapping<std::string> detection_mappings;
			ViewMapping<int> tag_mappings;

			for (size_t v = 0; v < n_views; ++v) {
				const auto& view_data = t_data[v];
				if (view_data.poses.find(frame) != view_data.poses.end()) {
					for (const auto& ele : view_data.poses.at(frame)) {
						pose_mappings[ele.first].push_back(v);
					}
				}
				if (view_data.detections.find(frame) != view_data.detections.end()) {
					for (const auto& ele : view_data.detections.at(frame)) {
						detection_mappings[ele.first].push_back(v);
					}
				}
				if (view_data.tags.find(frame) != view_data.tags.end()) {
					for (const auto& ele : view_data.tags.at(frame)) {
						tag_mappings[ele.first].push_back(v);
					}
				}
			}

			for (const auto& mapping : pose_mappings) {
				const auto& pose_id = mapping.first;
				const auto& pose_views = mapping.second;
				if (pose_views.size() < 2)
					continue;
				ViewMapping<int> joint_mappings;
				for (const auto& v : pose_views) {
					const auto& view_pose = t_data[v].poses.at(frame).at(pose_id);
					for (const auto& joint : view_pose.joints) {
						joint_mappings[joint.first].push_back(v);
					}
				}
				Pose3D triangulated_pose;
				for (const auto& j_mapping : joint_mappings) {
					const auto& joint_id = j_mapping.first;
					const auto& joint_views = j_mapping.second;
					if (joint_views.size() < 2)
						continue;
					std::vector<cv::Mat> joint_points;
					std::vector<cv::Mat> proj_matrices;
					for (const auto& v : joint_views) {
						const auto& view_joint = t_data[v].poses.at(frame).at(pose_id).joints.at(joint_id);
						const auto& view_K = cam_Kk[v].K;
						const auto& view_k = cam_dist_vectors[v];
						std::vector<cv::Point2f> src = {
							cv::Point2f(
								static_cast<float>(view_joint.pt.x),
								static_cast<float>(view_joint.pt.y)
							)
						};
						cv::Mat dst;
						cv::undistortPoints(src, dst, view_K, view_k);
						cv::Mat dst_mat(dst);
						dst_mat = dst_mat.reshape(1, 2);
						joint_points.push_back(dst_mat);
						proj_matrices.push_back(cam_P[v]);
					}
					cv::Mat points3d;
					cv::sfm::triangulatePoints(joint_points, proj_matrices, points3d);
					triangulated_pose[joint_id] = cv::Point3d(
						points3d.at<double>(0),
						points3d.at<double>(1),
						points3d.at<double>(2)
					);
				}
				if (!triangulated_pose.empty())
					frame_poses3D[pose_id] = triangulated_pose;
			}

			for (const auto& mapping : detection_mappings) {
				const auto& det_id = mapping.first;
				const auto& det_views = mapping.second;
				if (det_views.size() < 2)
					continue;
				std::vector<cv::Mat> det_points;
				std::vector<cv::Mat> proj_matrices;
				int class_id = 0;
				for (const auto& v : det_views) {
					const auto& view_det = t_data[v].detections.at(frame).at(det_id);
					const auto& view_K = cam_Kk[v].K;
					const auto& view_k = cam_dist_vectors[v];
					auto center = (view_det.bbox.tl() + view_det.bbox.br()) / 2;
					std::vector<cv::Point2f> src = {
						cv::Point2f(
							static_cast<float>(center.x),
							static_cast<float>(center.y)
						)
					};
					cv::Mat dst;
					cv::undistortPoints(src, dst, view_K, view_k);
					cv::Mat dst_mat(dst);
					dst_mat = dst_mat.reshape(1, 2);
					det_points.push_back(dst_mat);
					proj_matrices.push_back(cam_P[v]);
					class_id = view_det.class_id;
				}
				cv::Mat points3d;
				cv::sfm::triangulatePoints(det_points, proj_matrices, points3d);
				frame_detections3D[det_id] = Detection3D(class_id, cv::Point3d(
					points3d.at<double>(0),
					points3d.at<double>(1),
					points3d.at<double>(2)
				));
			}

			for (const auto& mapping : tag_mappings) {
				const auto& tag_id = mapping.first;
				const auto& tag_views = mapping.second;
				if (tag_views.size() < 2)
					continue;
				std::vector<cv::Mat> tag_points;
				std::vector<cv::Mat> proj_matrices;
				for (const auto& v : tag_views) {
					const auto& view_tag = t_data[v].tags.at(frame).at(tag_id);
					const auto& view_K = cam_Kk[v].K;
					const auto& view_k = cam_dist_vectors[v];
					for (size_t c = 0; c < 4; ++c) {
						std::vector<cv::Point2f> src = {
							cv::Point2f(
								static_cast<float>(view_tag.corners[c].x),
								static_cast<float>(view_tag.corners[c].y)
							)
						};
						cv::Mat dst;
						cv::undistortPoints(src, dst, view_K, view_k);
						cv::Mat dst_mat(dst);
						dst_mat = dst_mat.reshape(1, 2);
						tag_points.push_back(dst_mat);
					}
					proj_matrices.push_back(cam_P[v]);
				}
				cv::Mat points3d;
				cv::sfm::triangulatePoints(tag_points, proj_matrices, points3d);
				Tag3D triangulated_tag;
				triangulated_tag.id = tag_id;
				for (int c = 0; c < 4; ++c) {
					triangulated_tag.corners[c] = cv::Point3d(
						points3d.at<double>(0, c),
						points3d.at<double>(1, c),
						points3d.at<double>(2, c)
					);
				}
				frame_tags3D[tag_id] = triangulated_tag;
			}

			std::lock_guard<std::mutex> lock_guard(lock);
			if (!frame_poses3D.empty())
				output.poses[frame] = frame_poses3D;
			if (!frame_detections3D.empty())
				output.detections[frame] = frame_detections3D;
			if (!frame_tags3D.empty())
				output.tags[frame] = frame_tags3D;
		});

		return output;

	}

}