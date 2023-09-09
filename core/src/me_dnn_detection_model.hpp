/*
me_dnn_detection_model.hpp
Includes driver class for YOLO models

Driver classes are responsible for loading and managing model instances,
as well as performing all of the pre/post processing required for inference

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

#ifndef ME_DNN_DETECTION_MODEL_HPP
#define ME_DNN_DETECTION_MODEL_HPP

#include <me_dnn.hpp>
#include <onnxruntime_cxx_api.h>

namespace me {

	namespace dnn {

		class DetectionModel {
		public:
			DetectionModel();
			~DetectionModel();
			void load(const std::string& model_path, Executor target_executor = Executor::TENSORRT);
			void unload();
			void infer(const cv::Mat& image, std::vector<Detection>& detections, float conf_thresh, float iou_thresh);
			void infer(const std::vector<cv::Mat>& images, std::vector<std::vector<Detection>>& detections, float conf_thresh, float iou_thresh);
			bool is_loaded();
			cv::Size net_size();
			Precision get_precision();
			Executor get_executor();
		private:
			Precision precision = Precision::NONE; // Flag used for model stats when loaded
			Executor executor = Executor::NONE; // Flag used for model stats when loaded
			Ort::Env env;
			Ort::SessionOptions session_options;
			std::shared_ptr<Ort::Session> session; // MUST ensure the session is destroyed when using this smart pointer to prevent memory leaks
		};

	}

}

#endif
