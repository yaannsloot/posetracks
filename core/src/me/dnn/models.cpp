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

#include "models.hpp"
#include <cpu_provider_factory.h>

namespace me {

	namespace dnn {
		
		namespace models {

			void Model::load(const std::string& model_path, Executor target_executor) {
				// Unload the model if it is already loaded
				unload();

				this->env = std::make_shared<Ort::Env>(ORT_LOGGING_LEVEL_ERROR, this->logid.c_str());

				// Start a new onnx session on the specified device.
				// For systems with NVIDIA GPU, the GPU device option will use the TensorRT provider.
				// For systems without an NVIDIA GPU, the GPU device option will default to the CPU provider.
				this->session_options = std::make_shared<Ort::SessionOptions>();
				if (target_executor == Executor::TENSORRT && checkForProvider("TensorrtExecutionProvider")) {
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
						"motionengine_trt_cache",
						"1"
					};
					ort->UpdateTensorRTProviderOptions(trt_options, keys.data(), values.data(), 8);
					this->session_options->AppendExecutionProvider_TensorRT_V2(*trt_options);
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
					this->session_options->AppendExecutionProvider_CUDA_V2(*cuda_options);
					ort->ReleaseCUDAProviderOptions(cuda_options);
					executor = Executor::CUDA;
				}
				else {
					OrtStatusPtr status = OrtSessionOptionsAppendExecutionProvider_CPU(*this->session_options, 1);
					executor = Executor::CPU;
					this->session_options->SetIntraOpNumThreads(std::thread::hardware_concurrency());
				}
				this->session_options->SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
				this->session_options->SetLogId(this->logid.c_str());
				this->session_options->SetLogSeverityLevel(4);

				// Create the session
				std::wstring wide_path = std::wstring(model_path.begin(), model_path.end());
				try {
					this->session = std::make_shared<Ort::Session>(*this->env, wide_path.c_str(), *this->session_options);

					// Get the input and output names
					size_t num_input_nodes = this->session->GetInputCount();
					size_t num_output_nodes = this->session->GetOutputCount();
					if (num_input_nodes != this->inputs || num_output_nodes != this->outputs) {
						unload();
						std::string error = "Model driver expects ";
						error += std::to_string(inputs) + " input nodes and ";
						error += std::to_string(this->outputs) + " output nodes but instead found ";
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
					// Check if input and output names are valid
					for (auto& name : input_names) {
						if (std::string(name.get()).empty()) {
							unload();
							throw std::runtime_error("Failed to get input and output names from onnxruntime");
						}
					}
					for (auto& name : output_names) {
						if (std::string(name.get()).empty()) {
							unload();
							throw std::runtime_error("Failed to get input and output names from onnxruntime");
						}
					}
					
					// Check if input and output names match what is expected
					for (auto& name : input_names) {
						std::string name_str(name.get());
						if (this->input_names.find(name_str) == this->input_names.end()) {
							unload();
							std::string error = "Input name \"" + name_str + "\" is not present in allowed input names: {";
							for (const auto& element : this->input_names) {
								error += element + ", ";
							}
							error += "}";
							throw std::runtime_error(error);
						}
					}
					for (auto& name : output_names) {
						std::string name_str(name.get());
						if (this->output_names.find(name_str) == this->output_names.end()) {
							unload();
							std::string error = "Output name \"" + name_str + "\" is not present in allowed output names: {";
							for (const auto& element : this->output_names) {
								error += element + ", ";
							}
							error += "}";
							throw std::runtime_error(error);
						}
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

			void Model::unload() {
				this->session.reset();
				precision = Precision::NONE;
				executor = Executor::NONE;
			}

			bool Model::is_loaded() {
				return this->session != nullptr;
			}

			Precision Model::get_precision() {
				return this->precision;
			}

			Executor Model::get_executor() {
				return this->executor;
			}

		}
		
	}
	
}