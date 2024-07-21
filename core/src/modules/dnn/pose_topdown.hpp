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

#include "models.hpp"

namespace me::dnn::models {

	/// <summary>
	/// Basic ensemble driver for top down pose detection algorithms.
	/// Depends on both an object detector for candidate selection and a pose estimator for the actual pose estimation
	/// </summary>
	class TopDownPoseDetector {
	public:
		/// <summary>
		/// Unload all models used in this detector
		/// </summary>
		virtual void unload_all();

		/// <summary>
		/// Detect poses on the provided image
		/// </summary>
		/// <param name="image">Input image to process</param>
		/// <param name="poses">Output vector of poses with joints adjusted for frame coordinates</param>
		/// <param name="max_pose_batches">Maximum batch size for pose model inferencing</param>
		/// <param name="conf_thresh">Confidence threshold</param>
		/// <param name="iou_thresh">IoU threshold</param>
		virtual void infer(
			const cv::Mat& image,
			std::vector<Pose>& poses,
			int max_pose_batches = 1,
			float conf_thresh = 0.5,
			float iou_thresh = 0.5
		);

		/// <summary>
		/// Detect poses on the provided images
		/// </summary>
		/// <param name="images">Input vector containing images to process</param>
		/// <param name="poses">Output vector of vectors containing the estimated pose for each image with joints adjusted for frame coordinates</param>
		/// <param name="max_detection_batches">Maximum batch size for detection model inferencing</param>
		/// <param name="max_pose_batches">Maximum batch size for pose model inferencing</param>
		/// <param name="conf_thresh">Confidence threshold</param>
		/// <param name="iou_thresh">IoU threshold</param>
		virtual void infer(
			const std::vector<cv::Mat>& images,
			std::vector<std::vector<Pose>>& poses,
			int max_detection_batches = 1,
			int max_pose_batches = 1,
			float conf_thresh = 0.5,
			float iou_thresh = 0.5
		);

		bool is_ready();

		DetectionModel detection_model;
		PoseModel pose_model;

	};

}