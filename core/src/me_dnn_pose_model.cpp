/*
me_dnn_pose_model.cpp
Includes driver class for RTMPose models

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

#include <me_dnn_pose_model.hpp>
#include <cpu_provider_factory.h>
#include <opencv2/dnn.hpp>
#include <filesystem>

namespace me {

	namespace dnn {

		PoseModel::PoseModel() {}

		PoseModel::~PoseModel() {
			unload();
		}

		void PoseModel::load(const std::string& model_path, Executor target_executor) {
			// Unload the model if it is already loaded
			unload();

			this->env = Ort::Env(ORT_LOGGING_LEVEL_ERROR, "me_rtmpose_driver");

			// Start a new onnx session on the specified device.
			// For systems with NVIDIA GPU, the GPU device option will use the TensorRT provider.
			// For systems without an NVIDIA GPU, the GPU device option will default to the CPU provider.
			this->session_options = Ort::SessionOptions();
			if (target_executor == Executor::TENSORRT && checkForProvider("TensorrtExecutionProvider")) {
				std::filesystem::path filePath(model_path);
				auto dir = filePath.parent_path();
				OrtTensorRTProviderOptionsV2* trt_options = nullptr;
				const OrtApi* ort = OrtGetApiBase()->GetApi(12);
				ort->CreateTensorRTProviderOptions(&trt_options);
				std::vector<const char*> keys{
					"device_id",
					"trt_max_workspace_size",
					"trt_max_partition_iterations",
					"trt_min_subgraph_size",
					"trt_fp16_enable",
					"trt_engine_cache_enable",
					"trt_engine_cache_path",
					"trt_dump_subgraphs"
				};
				std::vector<const char*> values{
					"0",
					"2147483648",
					"10",
					"5",
					"1",
					"1",
					dir.string().c_str(),
					"1"
				};
				ort->UpdateTensorRTProviderOptions(trt_options, keys.data(), values.data(), 8);
				this->session_options.AppendExecutionProvider_TensorRT_V2(*trt_options);
				ort->ReleaseTensorRTProviderOptions(trt_options);
				executor = Executor::TENSORRT;
			}
			else if (target_executor == Executor::CUDA && checkForProvider("CUDAExecutionProvider")) {
				OrtCUDAProviderOptionsV2* cuda_options = nullptr;
				const OrtApi* ort = OrtGetApiBase()->GetApi(12);
				ort->CreateCUDAProviderOptions(&cuda_options);
				std::vector<const char*> keys{ "device_id", "gpu_mem_limit", "arena_extend_strategy", "cudnn_conv_algo_search", "do_copy_in_default_stream", "cudnn_conv_use_max_workspace" };
				std::vector<const char*> values{ "0", "2147483648", "kSameAsRequested", "DEFAULT", "1", "1" };
				ort->UpdateCUDAProviderOptions(cuda_options, keys.data(), values.data(), 6);
				this->session_options.AppendExecutionProvider_CUDA_V2(*cuda_options);
				ort->ReleaseCUDAProviderOptions(cuda_options);
				executor = Executor::CUDA;
			}
			else {
				OrtStatusPtr status = OrtSessionOptionsAppendExecutionProvider_CPU(this->session_options, 1);
				executor = Executor::CPU;
				this->session_options.SetIntraOpNumThreads(std::thread::hardware_concurrency());
			}
			this->session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
			this->session_options.SetLogId("me_rtmpose_driver");
			this->session_options.SetLogSeverityLevel(4);
			
			// Create the session
			std::wstring wide_path = std::wstring(model_path.begin(), model_path.end());
			try {
				this->session = std::make_shared<Ort::Session>(this->env, wide_path.c_str(), this->session_options);
				
				// Get the input and output names
				size_t num_input_nodes = this->session->GetInputCount();
				size_t num_output_nodes = this->session->GetOutputCount();
				if (num_input_nodes != 1 || num_output_nodes != 2) {
					unload();
					std::string error = "Pose model expects 1 input node and 2 output nodes but instead found ";
					error += std::to_string(num_input_nodes) + " input nodes and ";
					error += std::to_string(num_output_nodes) + " output nodes";
					throw std::runtime_error(error);
				}
				std::vector<Ort::AllocatedStringPtr> input_names;
				std::vector<Ort::AllocatedStringPtr> output_names;
				Ort::AllocatorWithDefaultOptions allocator;
				for (size_t i = 0; i < num_input_nodes; i++) {
					input_names.push_back(this->session->GetInputNameAllocated(i, allocator));
				}
				for (size_t i = 0; i < num_output_nodes; i++) {
					output_names.push_back(this->session->GetOutputNameAllocated(i, allocator));
				}
				std::string input_name(input_names[0].get());
				std::string output_name1(output_names[0].get());
				std::string output_name2(output_names[1].get());
				// Check if input and output names are valid
				if (input_name.empty() || output_name1.empty() || output_name2.empty()) {
					unload();
					throw std::runtime_error("Failed to get input and output names from onnxruntime");
				}
				// Check if input and output names match rtmpose
				if(input_name != "input" || output_name1 != "simcc_x" || output_name2 != "simcc_y") {
					unload();
					std::string error = "Pose model expects input node name to be 'input' and output node names to be 'simcc_x' and 'simcc_y' but instead found ";
					error += "input node name '" + input_name + "' and output node names '" + output_name1 + "' and '" + output_name2 + "'";
					throw std::runtime_error(error);
				}
				// Get input type
				Ort::TypeInfo input_type_info = this->session->GetInputTypeInfo(0);
				auto input_type = input_type_info.GetTensorTypeAndShapeInfo().GetElementType();
				if (input_type == ONNX_TENSOR_ELEMENT_DATA_TYPE_DOUBLE)
					precision = Precision::FLOAT64;
				else if (input_type == ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT)
					precision = Precision::FLOAT32;
				else if (input_type == ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT16)
					precision = Precision::FLOAT16;
				else if (input_type == ONNX_TENSOR_ELEMENT_DATA_TYPE_INT64 ||
					input_type == ONNX_TENSOR_ELEMENT_DATA_TYPE_UINT64)
					precision = Precision::INT64;
				else if (input_type == ONNX_TENSOR_ELEMENT_DATA_TYPE_INT32 ||
					input_type == ONNX_TENSOR_ELEMENT_DATA_TYPE_UINT32)
					precision = Precision::INT32;
				else if (input_type == ONNX_TENSOR_ELEMENT_DATA_TYPE_INT16 ||
					input_type == ONNX_TENSOR_ELEMENT_DATA_TYPE_UINT16)
					precision = Precision::INT16;
				else if (input_type == ONNX_TENSOR_ELEMENT_DATA_TYPE_INT8 ||
					input_type == ONNX_TENSOR_ELEMENT_DATA_TYPE_UINT8)
					precision = Precision::INT8;
				else
					precision = Precision::UNKNOWN;
			}
			catch (Ort::Exception& e) {
				unload();
				std::string reason = e.what();
				throw std::runtime_error("Failed to create session for onnxruntime: " + reason);
			}
		}

		void PoseModel::unload() {
			this->session.reset();
			precision = Precision::NONE;
			executor = Executor::NONE;
			
		}

		void PoseModel::infer(const cv::Mat& image, Pose& pose) {
			std::vector<cv::Mat> images = { image };
			std::vector<Pose> poses = { pose };
			infer(images, poses);
			pose = poses[0];
		}

		void PoseModel::infer(const std::vector<cv::Mat>& images, std::vector<Pose>& poses) {

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
			std::array<int64_t, 4> input_shape{ (int64_t)images.size(), 3, net_height, net_width};
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
			auto start = std::chrono::high_resolution_clock::now();
			this->session->Run(Ort::RunOptions{ nullptr }, binding);
			auto end = std::chrono::high_resolution_clock::now();
			std::cout << "Runtime: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << "ms" << std::endl;

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

		bool PoseModel::is_loaded() {
			return this->session != nullptr;
		}

		cv::Size PoseModel::net_size() {
			cv::Size result(0, 0);
			if (is_loaded()) {
				auto net_shape = this->session->GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
				int net_height = (int)net_shape[2];
				int net_width = (int)net_shape[3];
				result = cv::Size(net_width, net_height);
			}
			return result;
		}

		Precision PoseModel::get_precision() {
			return precision;
		}

		Executor PoseModel::get_executor() {
			return executor;
		}

	}

}