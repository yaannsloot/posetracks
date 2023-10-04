/*
me_dnn_detectpose_model.cpp
Special module that does top down model inference for pose estimation

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
*/

#include <me_dnn_detectpose_model.hpp>

namespace me {

	namespace dnn {

		cv::Mat getRoiWithPadding(const cv::Mat& image, cv::Rect roi) {
			// Create rects representing the image and the ROI
			auto image_rect = cv::Rect(0, 0, image.cols, image.rows);

			// Find intersection, i.e. valid crop region
			auto intersection = image_rect & roi;

			// Move intersection to the result coordinate space
			auto inter_roi = intersection - roi.tl();

			// Create gray image and copy intersection
			cv::Mat crop = cv::Mat::ones(roi.size(), image.type());
			image(intersection).copyTo(crop(inter_roi));

			return crop;
		}

		DetectPoseModel::DetectPoseModel() {}

		void DetectPoseModel::unload_all() {
			detection_model.unload();
			pose_model.unload();
		}

		void DetectPoseModel::infer(const cv::Mat& image, std::vector<Pose>& poses, int max_pose_batches, float conf_thresh, float iou_thresh) {
			std::vector<cv::Mat> images{ image };
			std::vector<std::vector<Pose>> poses_;
			infer(images, poses_, 1, max_pose_batches);
			poses = poses_[0];
		}

		void DetectPoseModel::infer(const std::vector<cv::Mat>& images, std::vector<std::vector<Pose>>& poses, int max_detection_batches, int max_pose_batches, float conf_thresh, float iou_thresh) {
			if(!this->is_ready())
				throw std::runtime_error("All models must be loaded prior to inference");
			// Check if images is not empty
			if (images.empty())
				throw std::runtime_error("The source image vector is empty");
			// Check if the images in the vector are valid
			for (const cv::Mat& image : images) {
				if (image.empty())
					throw std::runtime_error("Source image vector contains an empty image");
			}
			if (max_detection_batches < 1)
				max_detection_batches = 1;
			if (max_pose_batches < 1)
				max_pose_batches = 1;

			// infer on detection model for pose regions of interest
			std::vector<std::vector<Detection>> detections;
			for (size_t i = 0; i < images.size(); i = i + max_detection_batches) {
				size_t end = i + max_detection_batches;
				std::vector<cv::Mat> chunk;
				if (end > images.size())
					chunk.assign(images.begin() + i, images.end());
				else
					chunk.assign(images.begin() + i, images.begin() + end);

				// Padding
				size_t diff = max_detection_batches - chunk.size();
				if (diff > 0) {
					auto size_d = detection_model.net_size();
					for (size_t d = 0; d < diff; ++d) {
						cv::Mat zero_img(size_d, CV_8UC3, cv::Scalar(0, 0, 0));
						chunk.push_back(zero_img);
					}
				}

				std::vector<std::vector<Detection>> chunk_detections;
				detection_model.infer(chunk, chunk_detections, conf_thresh, iou_thresh);

				// Post-infer trimming
				chunk_detections.resize(chunk_detections.size() - diff);

				// Add detections to vector
				detections.insert(detections.end(), chunk_detections.begin(), chunk_detections.end());
			}

			// Flatten detections vector, keeping track of original image indices
			std::vector<size_t> img_indices;
			std::vector<Detection> img_detections;
			for (size_t i = 0; i < detections.size(); i++) {
				std::vector<size_t> indices(detections[i].size(), i);
				img_indices.insert(img_indices.end(), indices.begin(), indices.end());
				img_detections.insert(img_detections.end(), detections[i].begin(), detections[i].end());
			}

			poses.clear();
			poses.resize(images.size());

			// infer on pose model. Final pose predictions will be placed into poses vector
			auto det_net_size = detection_model.net_size();
			auto pose_net_size = pose_model.net_size();
			for (size_t i = 0; i < img_detections.size(); i = i + max_pose_batches) {
				size_t end = i + max_pose_batches;
				std::vector<Detection> chunk;
				std::vector<int> chunk_indices;
				if (end > img_detections.size()) {
					chunk.assign(img_detections.begin() + i, img_detections.end());
					chunk_indices.assign(img_indices.begin() + i, img_indices.end());
				}
				else {
					chunk.assign(img_detections.begin() + i, img_detections.begin() + end);
					chunk_indices.assign(img_indices.begin() + i, img_indices.begin() + end);
				}
				std::vector<Pose> chunk_poses;
				std::vector<cv::Mat> ROIs;
				for (size_t j = 0; j < chunk.size(); j++) {
					auto roi_bbox_unscaled = chunk[j].bbox;
					auto tl = roi_bbox_unscaled.tl();
					auto br = roi_bbox_unscaled.br();
					const cv::Mat& frame = images[chunk_indices[j]];

					// Bounding box scaling
					tl.x = tl.x / det_net_size.width;
					br.x = br.x / det_net_size.width;
					tl.y = tl.y / det_net_size.height;
					br.y = br.y / det_net_size.height;
					br.x = br.x * frame.cols;
					br.y = br.y * frame.rows;
					tl.x = tl.x * frame.cols;
					tl.y = tl.y * frame.rows;
					cv::Size2d box_dims(br.x - tl.x, br.y - tl.y);
					cv::Point2d box_center(tl.x + box_dims.width / 2, tl.y + box_dims.height / 2);

					// Aspect ratio adjustment
					double aspect_ratio = (double)pose_net_size.width / (double)pose_net_size.height;
					if (box_dims.width > (aspect_ratio * box_dims.height))
						box_dims.height = box_dims.width / aspect_ratio;
					else if (box_dims.width < (aspect_ratio * box_dims.height))
						box_dims.width = box_dims.height * aspect_ratio;

					box_dims = box_dims * 1.2;

					cv::Point2d new_tl(box_center.x - box_dims.width / 2, box_center.y - box_dims.height / 2);

					cv::Rect2d new_bbox(new_tl, box_dims);

					chunk[j].bbox = new_bbox;
					cv::Mat roi = getRoiWithPadding(frame, new_bbox);
					ROIs.push_back(roi);
				}

				// Padding
				size_t diff = max_pose_batches - ROIs.size();
				if (diff > 0) {
					auto size_d = pose_model.net_size();
					for (size_t d = 0; d < diff; ++d) {
						cv::Mat zero_img(size_d, CV_8UC3, cv::Scalar(0, 0, 0));
						ROIs.push_back(zero_img);
					}
				}

				pose_model.infer(ROIs, chunk_poses);

				// Post-infer trimming
				chunk_poses.resize(chunk_poses.size() - diff);

				// Preallocation

				// Pose adjustments
				for (size_t j = 0; j < chunk.size(); j++) {
					Pose& pose = chunk_poses[j];
					const cv::Mat& frame = images[chunk_indices[j]];
					auto& bbox = chunk[j].bbox;
					auto num_joints = pose.num_joints();

					for (int k = 0; k < num_joints; k++) {
						auto& joint = pose[k];
						double x = joint.pt.x;
						double y = joint.pt.y;
						x = bbox.tl().x + x * bbox.width;
						y = bbox.tl().y + y * bbox.height;
						joint.pt.x = x;
						joint.pt.y = y;
					}
					poses[chunk_indices[j]].push_back(pose);
				}
			}
		}

		bool DetectPoseModel::is_ready() {
			if (detection_model.is_loaded() && pose_model.is_loaded())
				return true;
			return false;
		}

	}

}