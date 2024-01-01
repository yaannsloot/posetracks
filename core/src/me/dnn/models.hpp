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

#include "dnn.hpp"
#include <onnxruntime_cxx_api.h>

namespace me {
	
	namespace dnn {
		
		namespace models {

			// Implementation classes

			/// <summary>
			/// Generic model class used to define key methods and variables inherited by all model types
			/// </summary>
			class ModelImpl {
			public:
				virtual void load(const std::string& model_path, Executor target_executor = Executor::TENSORRT);
				void unload();
				bool is_loaded();
				Precision get_precision();
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
			class ImageModelImpl : public ModelImpl {
			public:
				virtual cv::Size net_size() = 0;
			};

			/// <summary>
			/// Base class for object detection models
			/// </summary>
			class DetectionModelImpl : public ImageModelImpl {
			public:
				virtual void infer(const cv::Mat& image, std::vector<Detection>& detections, float conf_thresh, float iou_thresh) = 0;
				virtual void infer(const std::vector<cv::Mat>& images, std::vector<std::vector<Detection>>& detections, float conf_thresh, float iou_thresh) = 0;
			};

			/// <summary>
			/// Base class for pose models
			/// </summary>
			class PoseModelImpl : public ImageModelImpl {
			public:
				virtual void infer(const cv::Mat& image, Pose& pose) = 0;
				virtual void infer(const std::vector<cv::Mat>& images, std::vector<Pose>& poses) = 0;
			};

			/// <summary>
			/// Base class for feature models
			/// </summary>
			class FeatureModelImpl : public ImageModelImpl {
			public:
				virtual void infer(const cv::Mat& image, Feature& feature) = 0;
				virtual void infer(const std::vector<cv::Mat>& images, std::vector<Feature>& features) = 0;
			};



			// Proxy classes

			/// <summary>
			/// Base model class. Has no inference functions.
			/// </summary>
			class Model {
			public:
				/// <summary>
				/// Load ONNX model and create a new session, unloading the current session if one exists.
				/// </summary>
				/// <param name="model_path">Path to the ONNX model</param>
				/// <param name="target_executor">Target model executor. Certain executors have fallbacks defined if the target is not available.</param>
				void load(const std::string& model_path, Executor target_executor = Executor::TENSORRT);

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
				std::shared_ptr<ModelImpl> model_ptr;
			};

			/// <summary>
			/// Image model class. Same as base model class but includes a function for obtaining the input dimensions of the model.
			/// </summary>
			class ImageModel : public Model {
			public:
				/// <summary>
				/// The size of the currently loaded model's input in pixels
				/// </summary>
				/// <returns>A cv::Size object with width and height in pixels</returns>
				cv::Size net_size();
			};

			/// <summary>
			/// Detection model class
			/// </summary>
			class DetectionModel : public ImageModel {
			public:
				/// <summary>
				/// Run an inference on the currently loaded model
				/// </summary>
				/// <param name="image">Input image to process</param>
				/// <param name="detections">Output vector for bounding box detections</param>
				/// <param name="conf_thresh">Confidence threshold</param>
				/// <param name="iou_thresh">IoU threshold</param>
				void infer(const cv::Mat& image, std::vector<Detection>& detections, float conf_thresh, float iou_thresh);

				/// <summary>
				/// Run a batch inference on the currently loaded model
				/// </summary>
				/// <param name="images">Input vector containing images to process</param>
				/// <param name="detections">Output vector of vectors containing bounding box detections for each image</param>
				/// <param name="conf_thresh">Confidence threshold</param>
				/// <param name="iou_thresh">IoU threshold</param>
				void infer(const std::vector<cv::Mat>& images, std::vector<std::vector<Detection>>& detections, float conf_thresh, float iou_thresh);
			};

			/// <summary>
			/// Pose model class
			/// </summary>
			class PoseModel : public ImageModel {
			public:
				/// <summary>
				/// Run an inference on the currently loaded model
				/// </summary>
				/// <param name="image">Input image to process</param>
				/// <param name="pose">Output vector for pose estimation</param>
				void infer(const cv::Mat& image, Pose& pose);

				/// <summary>
				/// Run a batch inference on the currently loaded model
				/// </summary>
				/// <param name="images">Input vector containing images to process</param>
				/// <param name="poses">Output vector containing the estimated pose for each image</param>
				void infer(const std::vector<cv::Mat>& images, std::vector<Pose>& poses);
			};

			/// <summary>
			/// Feature model class
			/// </summary>
			class FeatureModel : public ImageModel {
			public:
				/// <summary>
				/// Run an inference on the currently loaded model
				/// </summary>
				/// <param name="image">Input image to process</param>
				/// <param name="feature">Output feature extracted from the image</param>
				void infer(const cv::Mat& image, Feature& feature);

				/// <summary>
				/// Run a batch inference on the currently loaded model
				/// </summary>
				/// <param name="images">Input vector containing images to process</param>
				/// <param name="features">Output vector containing the feature extracted for each image</param>
				void infer(const std::vector<cv::Mat>& images, std::vector<Feature>& features);
			};

		}	
	
	}
	
}