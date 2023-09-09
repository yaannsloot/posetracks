/*
me_dnn_detectpose_model.hpp
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

#ifndef ME_DNN_DETECTPOSE_MODEL_HPP
#define ME_DNN_DETECTPOSE_MODEL_HPP

#include <me_dnn.hpp>
#include <me_dnn_pose_model.hpp>
#include <me_dnn_rtdetection_model.hpp>

namespace me {

	namespace dnn {

		// Special class that implements the rtmpose pipeline
		class DetectPoseModel {
		public:
			DetectPoseModel();
			void unload_all();
			void infer(const cv::Mat& image, std::vector<Pose>& poses, int max_pose_batches = 1, float conf_thresh = 0.5, float iou_thresh = 0.5);
			void infer(const std::vector<cv::Mat>& images, std::vector<std::vector<Pose>>& poses, int max_detection_batches = 1, int max_pose_batches = 1, float conf_thresh = 0.5, float iou_thresh = 0.5);
			bool is_ready();
			RTDetectionModel detection_model;
			PoseModel pose_model;
		};

	}

}

#endif
