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

#include "rtmdet.hpp"
#include "../data/memory.hpp"
#include <opencv2/dnn.hpp>

RTMDetModelImpl::RTMDetModelImpl() {
	inputs = 1;
	outputs = 2;
	logid = "me_rtmpose_driver";
	input_names = { "input" };
	output_names = { "dets", "labels" };
}

cv::Size RTMDetModelImpl::net_size() {
	cv::Size result(0, 0);
	if (is_loaded()) {
		auto net_shape = this->session->GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
		int net_height = (int)net_shape[2];
		int net_width = (int)net_shape[3];
		result = cv::Size(net_width, net_height);
	}
	return result;
}

void RTMDetModelImpl::infer(const cv::Mat& image, std::vector<Detection>& detections, float conf_thresh, float iou_thresh) {
	std::vector<cv::Mat> images{ image };
	std::vector<std::vector<Detection>> batch_detections;
	infer(images, batch_detections, conf_thresh, iou_thresh);
	detections = batch_detections[0];
}

void RTMDetModelImpl::infer(const std::vector<cv::Mat>& images, std::vector<std::vector<Detection>>& detections, float conf_thresh, float iou_thresh) {

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

	cv::Scalar mean(123.675, 116.28, 103.53);

	// Prepare tensor input data

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
	binding.BindOutput("dets", output_mem_info);
	binding.BindOutput("labels", output_mem_info);

	// Run inference
	// Test runtime
	this->session->Run(Ort::RunOptions{ nullptr }, binding);


	// Get the output tensors
	std::vector<Ort::Value> output_tensors = binding.GetOutputValues();

	std::vector<int64_t> det_result_dims = output_tensors[0].GetTensorTypeAndShapeInfo().GetShape();
	std::vector<int64_t> label_result_dims = output_tensors[1].GetTensorTypeAndShapeInfo().GetShape();

	assert(det_result_dims.size() == 3 && label_result_dims.size() == 2);

	size_t batch_size = (size_t)det_result_dims[0] == (size_t)label_result_dims[0] ? (size_t)det_result_dims[0] : 0;
	size_t num_dets = (size_t)det_result_dims[1] == (size_t)label_result_dims[1] ? (size_t)det_result_dims[1] : 0;
	size_t reshap_dims = (size_t)det_result_dims[2];

	// Get boxes and confs
	float* det_result = output_tensors[0].GetTensorMutableData<float>();
	int* label_result = output_tensors[1].GetTensorMutableData<int>();

	// Convert to accessors
	std::array<size_t, 3> dets_shape{ batch_size, num_dets, reshap_dims };
	std::array<size_t, 2> labels_shape{ batch_size, num_dets };
	Accessor<3, float> dets(det_result, dets_shape); // Access via: dets(batch, box, box_dim). box_dim is 0 for x, 1 for y, 2 for width, 3 for height, 4 for confidence
	Accessor<2, int> labels(label_result, labels_shape); // Access via: labels(batch, box)

	detections.clear();
	detections.resize(batch_size);

	// Perform per class nms
	for (size_t b = 0; b < batch_size; b++) {
		std::unordered_map<int, std::vector<Detection>> class_detections;
		for (size_t i = 0; i < num_dets; i++) {
			// Find the class with the highest confidence
			int class_ = labels(b, i);
			float conf = dets(b, i, 4);
			// Check if the confidence is above the threshold
			if (conf > conf_thresh) {
				// Get the box coordinates
				float x1 = dets(b, i, 0);
				float y1 = dets(b, i, 1);
				float x2 = dets(b, i, 2);
				float y2 = dets(b, i, 3);
				// Initialize the vector for the given class
				if (class_detections.count(class_) == 0)
					class_detections[class_] = std::vector<Detection>();
				// Add the detection to the list
				class_detections[class_].push_back(Detection(class_, cv::Rect2d(cv::Point2d(x1, y1), cv::Point2d(x2, y2)), conf));
			}
		}
		// Perform nms for each class
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
	binding.ClearBoundInputs();
	binding.ClearBoundOutputs();
}

RTMDetModel::RTMDetModel() {
	model_ptr = std::make_shared<RTMDetModelImpl>();
}
