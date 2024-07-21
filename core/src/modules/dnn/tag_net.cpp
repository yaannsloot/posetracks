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

#include "tag_net.hpp"
#include "../data/memory.hpp"
#include <opencv2/dnn.hpp>

namespace me::dnn::models {

	TagNetModelImpl::TagNetModelImpl()
	{
		inputs = 1;
		outputs = 2;
		logid = "me_tag_net_driver";
		input_names = { "input" };
		output_names = { "corners", "classes" };
	}

	cv::Size TagNetModelImpl::net_size()
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

	void TagNetModelImpl::infer(const cv::Mat& image, std::vector<Tag>& tags)
	{
		std::vector<cv::Mat> images{ image };
		std::vector<std::vector<Tag>> batch_tags;
		infer(images, batch_tags);
		tags = batch_tags[0];
	}

	void TagNetModelImpl::infer(const std::vector<cv::Mat>& images, std::vector<std::vector<Tag>>& tags)
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
		binding.BindOutput("corners", output_mem_info);
		binding.BindOutput("classes", output_mem_info);

		// Run inference
		this->session->Run(Ort::RunOptions{ nullptr }, binding);

		// Get the output tensors
		std::vector<Ort::Value> output_tensors = binding.GetOutputValues();

		std::vector<int64_t> corners_result_dims = output_tensors[0].GetTensorTypeAndShapeInfo().GetShape();
		std::vector<int64_t> classes_result_dims = output_tensors[1].GetTensorTypeAndShapeInfo().GetShape();

		assert(corners_result_dims.size() == 2 && classes_result_dims.size() == 2);

		size_t batch_size = (size_t)corners_result_dims[0] == (size_t)classes_result_dims[0] ? (size_t)corners_result_dims[0] : 0;
		size_t num_classes = (size_t)classes_result_dims[1];
		size_t corners_length = (size_t)corners_result_dims[1];

		float* corners_result = output_tensors[0].GetTensorMutableData<float>();
		float* classes_result = output_tensors[1].GetTensorMutableData<float>();

		std::array<size_t, 2> corners_shape{ batch_size, corners_length };
		std::array<size_t, 2> classes_shape{ batch_size, num_classes };
		me::data::Accessor<2, float> corners(corners_result, corners_shape);
		me::data::Accessor<2, float> classes(classes_result, classes_shape);

		tags.clear();
		tags.resize(batch_size);

		for (size_t b = 0; b < batch_size; ++b) {
			int class_id = 0;
			double conf = 0;
			for (int c = 0; c < num_classes; ++c) {
				double c_conf = (double)classes(b, c);
				if (c_conf > conf) {
					conf = c_conf;
					class_id = c;
				}
			}
			conf = std::min(conf, (double)corners(b, 8));
			cv::Point2d ca((double)corners(b, 0), (double)corners(b, 1));
			cv::Point2d cb((double)corners(b, 2), (double)corners(b, 3));
			cv::Point2d cc((double)corners(b, 4), (double)corners(b, 5));
			cv::Point2d cd((double)corners(b, 6), (double)corners(b, 7));
			tags[b].push_back(Tag(class_id, conf, ca, cb, cc, cd));
		}
		binding.ClearBoundInputs();
		binding.ClearBoundOutputs();
	}

	TagNetModel::TagNetModel()
	{
		model_ptr = std::make_shared<TagNetModelImpl>();
	}

}
