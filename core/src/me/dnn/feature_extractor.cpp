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

#include "feature_extractor.hpp"
#include "../data/memory.hpp"
#include <opencv2/dnn.hpp>

namespace me::dnn::models {

	GenericFeatureModelImpl::GenericFeatureModelImpl() {
		inputs = 1;
		outputs = 1;
		logid = "me_generic_feature_model_driver";
		input_names = { "input" };
		output_names = { "output" };
	}

	cv::Size GenericFeatureModelImpl::net_size() {
		cv::Size result(0, 0);
		if (is_loaded()) {
			auto net_shape = this->session->GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
			int net_height = (int)net_shape[2];
			int net_width = (int)net_shape[3];
			result = cv::Size(net_width, net_height);
		}
		return result;
	}

	void GenericFeatureModelImpl::infer(const cv::Mat& image, Feature& feature) {
		std::vector<cv::Mat> images{ image };
		std::vector<Feature> features;
		infer(images, features);
		feature = features[0];
	}

	void GenericFeatureModelImpl::infer(const std::vector<cv::Mat>& images, std::vector<Feature>& features) {
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

		cv::Mat blob;
		if (images.size() > 1) {
			cv::dnn::blobFromImages(images, blob, 1.0 / 255, cv::Size(net_width, net_height), true);
		}
		else {
			cv::dnn::blobFromImage(images[0], blob, 1.0 / 255, cv::Size(net_width, net_height), true);
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
		binding.BindOutput("output", output_mem_info);

		// Run inference
		this->session->Run(Ort::RunOptions{ nullptr }, binding);

		// Get the output tensors
		std::vector<Ort::Value> output_tensors = binding.GetOutputValues();

		std::vector<int64_t> result_dims = output_tensors[0].GetTensorTypeAndShapeInfo().GetShape();

		assert(result_dims.size() == 2);

		size_t batch_size = (size_t)result_dims[0];
		size_t feature_length = (size_t)result_dims[1];

		float* result = output_tensors[0].GetTensorMutableData<float>();

		std::array<size_t, 2> result_shape{ batch_size, feature_length };
		me::data::Accessor<2, float> results(result, result_shape);

		features.clear();
		features.resize(batch_size);

		//auto start = std::chrono::high_resolution_clock::now();
				
		for (size_t b = 0; b < batch_size; ++b) {
			for (size_t f = 0; f < feature_length; ++f) {
				features[b].data.push_back(results(b, f));
			}
		}

		//auto end = std::chrono::high_resolution_clock::now();

		//double time = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
		//std::cout << "FE copy time: " << time << "us" << std::endl;

		binding.ClearBoundInputs();
		binding.ClearBoundOutputs();
	}

	GenericFeatureModel::GenericFeatureModel() {
		model_ptr = std::make_shared<GenericFeatureModelImpl>();
	}

}