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

#include "rtmpose.hpp"
#include <opencv2/dnn.hpp>

namespace me {

	namespace dnn {

		namespace models {

			RTMPoseModelImpl::RTMPoseModelImpl() {
				inputs = 1;
				outputs = 2;
				logid = "me_rtmpose_driver";
				input_names = { "input" };
				output_names = { "simcc_x", "simcc_y" };
			}

			void RTMPoseModelImpl::infer(const cv::Mat& image, Pose& pose) {
				std::vector<cv::Mat> images = { image };
				std::vector<Pose> poses = { pose };
				infer(images, poses);
				pose = poses[0];
			}

			void RTMPoseModelImpl::infer(const std::vector<cv::Mat>& images, std::vector<Pose>& poses) {

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

				// Warp image to network input size

				cv::Scalar mean(123.675, 116.28, 103.53);

				cv::Mat blob;
				if (images.size() > 1) {
					cv::dnn::blobFromImages(images, blob, 1.0 / 57.63, cv::Size(net_width, net_height), mean);
				}
				else {
					cv::dnn::blobFromImage(images[0], blob, 1.0 / 57.63, cv::Size(net_width, net_height), mean);
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
				binding.BindInput("input", input_tensor);
				Ort::MemoryInfo output_mem_info{ "Cpu", OrtDeviceAllocator, 0, OrtMemTypeDefault };
				binding.BindOutput("simcc_x", output_mem_info);
				binding.BindOutput("simcc_y", output_mem_info);

				// Run inference
				this->session->Run(Ort::RunOptions{ nullptr }, binding);

				// Get the output tensors
				std::vector<Ort::Value> output_tensors = binding.GetOutputValues();

				std::vector<int64_t> simcc_x_dims = output_tensors[0].GetTensorTypeAndShapeInfo().GetShape();
				std::vector<int64_t> simcc_y_dims = output_tensors[1].GetTensorTypeAndShapeInfo().GetShape();

				assert(simcc_x_dims.size() == 3 && simcc_y_dims.size() == 3);

				size_t batch_size = simcc_x_dims[0] == simcc_y_dims[0] ? simcc_x_dims[0] : 0;
				size_t joint_num = simcc_x_dims[1] == simcc_y_dims[1] ? simcc_x_dims[1] : 0;
				size_t extend_width = simcc_x_dims[2];
				size_t extend_height = simcc_y_dims[2];

				float* simcc_x_result = output_tensors[0].GetTensorMutableData<float>();
				float* simcc_y_result = output_tensors[1].GetTensorMutableData<float>();

				// Convert to accessors
				std::array<size_t, 3> simcc_x_shape{ batch_size, joint_num, extend_width };
				std::array<size_t, 3> simcc_y_shape{ batch_size, joint_num, extend_height };
				Accessor<3, float> simcc_x(simcc_x_result, simcc_x_shape);
				Accessor<3, float> simcc_y(simcc_y_result, simcc_y_shape);

				poses.clear();
				poses.resize(batch_size);

				for (size_t b = 0; b < batch_size; b++) {
					for (int j = 0; j < joint_num; j++) {
						int max_x = 0;
						int max_y = 0;
						float max_prob_x = 0.0f;
						float max_prob_y = 0.0f;
						for (int i = 0; i < extend_width; i++) {
							float prob_current = simcc_x(b, j, i);
							if (prob_current > max_prob_x) {
								max_x = i;
								max_prob_x = prob_current;
							}
						}
						for (int i = 0; i < extend_height; i++) {
							float prob_current = simcc_y(b, j, i);
							if (prob_current > max_prob_y) {
								max_y = i;
								max_prob_y = prob_current;
							}
						}
						poses[b].set_joint(j, (float)max_x / 2 / net_width, (float)max_y / 2 / net_height, std::max(max_prob_x, max_prob_y));
					}
				}

				binding.ClearBoundInputs();
				binding.ClearBoundOutputs();

			}

			cv::Size RTMPoseModelImpl::net_size() {
				cv::Size result(0, 0);
				if (is_loaded()) {
					auto net_shape = this->session->GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
					int net_height = (int)net_shape[2];
					int net_width = (int)net_shape[3];
					result = cv::Size(net_width, net_height);
				}
				return result;
			}

			RTMPoseModel::RTMPoseModel() {
				model_ptr = std::make_shared<RTMPoseModelImpl>();
			}

		}

	}

}