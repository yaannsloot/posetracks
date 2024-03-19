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

#include "yolox.hpp"
#include "../data/memory.hpp"
#include <opencv2/dnn.hpp>

namespace me {

	namespace dnn {

		namespace models {

			YOLOXModelImpl::YOLOXModelImpl()
			{
				inputs = 1;
				outputs = 1;
				logid = "me_yolox_driver";
				input_names = { "images" };
				output_names = { "output" };
			}

			cv::Size YOLOXModelImpl::net_size()
			{
				cv::Size result(0, 0);
				if (is_loaded()) {
					auto net_shape = this->session->GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
					int net_height = (int)net_shape[2];
					int net_width = (int)net_shape[3];
					result = cv::Size(net_width, net_height);
				}
				return result;
			}

			void YOLOXModelImpl::infer(const cv::Mat& image, std::vector<Detection>& detections, float conf_thresh, float iou_thresh)
			{
				std::vector<cv::Mat> images{ image };
				std::vector<std::vector<Detection>> batch_detections;
				infer(images, batch_detections, conf_thresh, iou_thresh);
				detections = batch_detections[0];
			}

			// Inference post processing functions
			// Shamelessly copied from https://github.com/Megvii-BaseDetection/YOLOX/blob/main/demo/TensorRT/cpp/yolox.cpp

			struct GridAndStride
			{
				int grid0;
				int grid1;
				int stride;
			};

			static void generate_grids_and_stride(std::vector<int>& strides, std::vector<GridAndStride>& grid_strides, int net_height, int net_width)
			{
				for (auto stride : strides)
				{
					int num_grid_y = net_height / stride;
					int num_grid_x = net_width / stride;
					for (int g1 = 0; g1 < num_grid_y; g1++)
					{
						for (int g0 = 0; g0 < num_grid_x; g0++)
						{
							grid_strides.push_back(GridAndStride{ g0, g1, stride });
						}
					}
				}
			}

			void YOLOXModelImpl::infer(const std::vector<cv::Mat>& images, std::vector<std::vector<Detection>>& detections, float conf_thresh, float iou_thresh)
			{
				// Check if session is loaded
				if (this->session == nullptr)
					throw std::runtime_error("Session is not loaded");
				// Check if images is not empty
				if (images.empty())
					throw std::runtime_error("The source image vector is empty");
				// Check if the images in the vector are valid
				for (const cv::Mat& image : images) {
					if (image.empty())
						throw std::runtime_error("Source image vector contains an empty image");
				}

				// Get network input size
				auto net_shape = this->session->GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
				int net_height = (int)net_shape[2];
				int net_width = (int)net_shape[3];

				// Prepare tensor input data
				cv::Mat blob;
				if (images.size() > 1) {
					cv::dnn::blobFromImages(images, blob, 1.0, cv::Size(net_width, net_height));
				}
				else {
					cv::dnn::blobFromImage(images[0], blob, 1.0, cv::Size(net_width, net_height));
				}

				// Convert to ORT tensor
				std::array<int64_t, 4> input_shape{ (int64_t)images.size(), 3, net_height, net_width };
				size_t input_size = input_shape[0] * input_shape[1] * input_shape[2] * input_shape[3];
				auto memory_info = Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU);
				auto input_tensor = Ort::Value::CreateTensor<float>(
					memory_info,
					(float*)blob.data,
					input_size,
					input_shape.data(),
					input_shape.size()
				);

				Ort::IoBinding binding{ *(this->session) };
				binding.BindInput("images", input_tensor);
				Ort::MemoryInfo output_mem_info{ "Cpu", OrtDeviceAllocator, 0, OrtMemTypeDefault };
				binding.BindOutput("output", output_mem_info);

				// Run inference
				this->session->Run(Ort::RunOptions{ nullptr }, binding);

				// Get the output tensors
				std::vector<Ort::Value> output_tensors = binding.GetOutputValues();

				std::vector<int64_t> output_dims = output_tensors[0].GetTensorTypeAndShapeInfo().GetShape();

				assert(output_dims.size() == 3);

				// Get output dims
				size_t batch_size = (size_t)output_dims[0];
				size_t num_dets = (size_t)output_dims[1];
				size_t num_classes = (size_t)output_dims[2] - 5;

				// Convert to accessor
				float* output_ptr = output_tensors[0].GetTensorMutableData<float>();

				detections.clear();
				detections.resize(batch_size);

				// YOLOX SPECIFIC POSTPROCESSING VARS
				std::vector<int> strides = { 8, 16, 32 };
				std::vector<GridAndStride> grid_strides;
				generate_grids_and_stride(strides, grid_strides, net_height, net_width);

				// Convert to accessor
				std::array<size_t, 3> output_shape{ batch_size, num_dets, num_classes + 5 };
				me::data::Accessor<3, float> output(output_ptr, output_shape);

				// Decode and perform per class nms
				// auto start = std::chrono::high_resolution_clock::now();
				for (size_t b = 0; b < batch_size; b++) {
					std::unordered_map<size_t, std::vector<Detection>> class_detections;

					for (size_t i = 0; i < num_dets; ++i) {
						const int grid0 = grid_strides[i].grid0;
						const int grid1 = grid_strides[i].grid1;
						const int stride = grid_strides[i].stride;
						float box_score = output(b, i, 4);
						int class_ = 0;
						float conf = 0;
						for (int c = 0; c < num_classes; ++c) {
							float class_score = output(b, i, 5 + i);
							if (class_score > conf) {
								conf = class_score;
								class_ = c;
							}
						}
						if (box_score > conf_thresh) {
							float x_center = (output(b, i, 0) + grid0) * stride;
							float y_center = (output(b, i, 1) + grid1) * stride;
							float w = exp(output(b, i, 2)) * stride;
							float h = exp(output(b, i, 3)) * stride;
							float x0 = x_center - w * 0.5f;
							float y0 = y_center - h * 0.5f;
							if (class_detections.count(class_) == 0)
								class_detections[class_] = std::vector<Detection>();
							class_detections[class_].push_back(Detection(class_,
								cv::Rect2d(
									x0,
									y0,
									w,
									h
								),
								box_score
							));
						}
					}

					// Per class NMS
					for (auto& pair : class_detections) {
						std::vector<Detection>& class_detections_ref = pair.second;
						if (!class_detections_ref.empty()) {
							std::vector<size_t> indices = nms(class_detections_ref, iou_thresh);
							for (size_t i : indices) {
								detections[b].push_back(class_detections_ref[i]);
							}
						}
					}
				}
				// auto end = std::chrono::high_resolution_clock::now();
				// double conversion_time = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
				// std::cout << "Conversion time: " << conversion_time << "us" << std::endl;
				binding.ClearBoundInputs();
				binding.ClearBoundOutputs();
			}

			YOLOXModel::YOLOXModel()
			{
				model_ptr = std::make_shared<YOLOXModelImpl>();
			}

		}

	}

}