/*
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

#pragma once

#include "dnn.hpp"
#include <onnxruntime_cxx_api.h>

namespace me {
	
	namespace dnn {
		
		namespace models {

			/// <summary>
			/// Generic model class used to define key methods and variables inherited by all model types
			/// </summary>
			class Model {
			public:
				/// <summary>
				/// Load ONNX model and create a new session, unloading the current session if one exists.
				/// </summary>
				/// <param name="model_path">Path to the ONNX model</param>
				/// <param name="target_executor">Target model executor. Certain executors have fallbacks defined if the target is not available.</param>
				virtual void load(const std::string& model_path, Executor target_executor = Executor::TENSORRT);

				/// <summary>
				/// Reset the current ONNX session
				/// <para>This will unload any models associated with this driver object.
				/// If no models were loaded, this function will have no effect.</para>
				/// </summary>
				void unload();

				/// <summary>
				/// Checks if a model is currently loaded and available for inference
				/// </summary>
				/// <returns>True if a model is available, False otherwise</returns>
				bool is_loaded();

				/// <summary>
				/// If a model is currently loaded, this reflects its precision
				/// </summary>
				Precision get_precision();

				/// <summary>
				/// If a model is currently loaded, this reflects the executor assigned to its ONNX session
				/// </summary>
				Executor get_executor();
			protected:
				// Values kept during lifetime of driver
				int inputs = 0;
				int outputs = 0;
				std::string logid = "me_generic_model_driver";
				std::set<std::string> input_names = {};
				std::set<std::string> output_names = {};

				// Modified while an onnx session is running
				Precision precision = Precision::NONE; // Flag used for model stats when loaded
				Executor executor = Executor::NONE; // Flag used for model stats when loaded
				std::shared_ptr<Ort::Env> env;
				std::shared_ptr<Ort::SessionOptions> session_options;
				std::shared_ptr<Ort::Session> session; // MUST ensure the session is destroyed when using this smart pointer to prevent memory leaks
			};

			/// <summary>
			/// Base class for all models that take images as imput
			/// </summary>
			class ImageModel : public Model {
			public:
				/// <summary>
				/// The size of the currently loaded model's input in pixels
				/// </summary>
				/// <returns>A cv::Size object with width and height in pixels</returns>
				virtual cv::Size net_size() = 0;
			};

			/// <summary>
			/// Base class for object detection models
			/// </summary>
			class DetectionModel : public ImageModel{
			public:
				/// <summary>
				/// Run an inference on the currently loaded model
				/// </summary>
				/// <param name="image">Input image to process</param>
				/// <param name="detections">Output vector for bounding box detections</param>
				/// <param name="conf_thresh">Confidence threshold</param>
				/// <param name="iou_thresh">IoU threshold</param>
				virtual void infer(const cv::Mat& image, std::vector<Detection>& detections, float conf_thresh, float iou_thresh) = 0;

				/// <summary>
				/// Run a batch inference on the currently loaded model
				/// </summary>
				/// <param name="images">Input vector containing images to process</param>
				/// <param name="detections">Output vector of vectors containing bounding box detections for each image</param>
				/// <param name="conf_thresh">Confidence threshold</param>
				/// <param name="iou_thresh">IoU threshold</param>
				virtual void infer(const std::vector<cv::Mat>& images, std::vector<std::vector<Detection>>& detections, float conf_thresh, float iou_thresh) = 0;
			};

			class PoseModel : public ImageModel {
			public:
				/// <summary>
				/// Run an inference on the currently loaded model
				/// </summary>
				/// <param name="image">Input image to process</param>
				/// <param name="pose">Output vector for pose estimation</param>
				virtual void infer(const cv::Mat& image, Pose& pose) = 0;

				/// <summary>
				/// Run a batch inference on the currently loaded model
				/// </summary>
				/// <param name="images">Input vector containing images to process</param>
				/// <param name="poses">Output vector of vectors containing the estimated pose for each image</param>
				virtual void infer(const std::vector<cv::Mat>& images, std::vector<Pose>& poses) = 0;
			};

		}	
	
	}
	
}