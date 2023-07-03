/*
me_dnn_bindings.cpp
Pybind11 module definitions for me_dnn_module.hpp

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

#include <me_dnn_module.hpp>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>

namespace py = pybind11;

// Python bindings

// Conversion: cv::Mat -> py::array
py::array mat2array(const cv::Mat& mat) {
	int h = mat.rows;
	int w = mat.cols;
	int ch = mat.channels();

	// Check the OpenCV Mat data type
	int dtype;
	std::string format;
	size_t bytes;
	if (mat.depth() == CV_8U) {
		dtype = CV_8U;
		format = py::format_descriptor<uint8_t>::format();
		bytes = sizeof(uint8_t);
	}
	else if (mat.depth() == CV_32F) {
		dtype = CV_32F;
		format = py::format_descriptor<float>::format();
		bytes = sizeof(float);
	}
	else {
		throw std::runtime_error("Unsupported data type");
	}

	// Construct the numpy array
	switch (ch) {
	case 1:
		return py::array(py::buffer_info(
			mat.data,
			bytes,
			format,
			2,
			{ h, w },
			{ bytes * w, bytes }
		));
	case 3:
		return py::array(py::buffer_info(
			mat.data,
			bytes,
			format,
			3,
			{ h, w, ch },
			{ bytes * w * ch, bytes * ch, bytes }
		));
	default:
		throw std::runtime_error("Unsupported channel number");
	}
}


// Conversion: py::array -> cv::Mat
cv::Mat array2mat(py::array array) {
	py::buffer_info info = array.request();
	int h = info.shape[0];
	int w = info.shape[1];
	int ch = (info.ndim == 2) ? 1 : info.shape[2];

	// Check the numpy array data type
	int dtype;
	if (info.format == py::format_descriptor<uint8_t>::format())
		dtype = CV_8U;
	else if (info.format == py::format_descriptor<float>::format())
		dtype = CV_32F;
	else
		throw std::runtime_error("Unsupported data type");

	// Construct the cv::Mat
	if (ch == 1) {
		return cv::Mat(h, w, dtype, info.ptr);
	}
	else if (ch == 3) {
		return cv::Mat(h, w, dtype | CV_8UC3, info.ptr);
	}
	else {
		throw std::runtime_error("Unsupported channel number");
	}
}


PYBIND11_MODULE(MEDNNBase, m) {
	py::class_<me::dnn::MEDNNModuleInstance>(m, "MEDNNModule")
		.def(py::init<const std::string&>(), py::arg("dll_path"))
		.def("GetModuleDetails", &me::dnn::MEDNNModuleInstance::GetModuleDetails)
		.def("forward", [](me::dnn::MEDNNModuleInstance& self, py::list inputs) {

			// Convert inputs to std::vector<std::vector<cv::Mat>>
			std::vector<std::vector<cv::Mat>> mats;
			for (py::handle outer : inputs) {
				std::vector<cv::Mat> inner_mats;
				for (py::handle inner : outer.cast<py::list>()) {
					inner_mats.push_back(array2mat(inner.cast<py::array>()));
				} 
				mats.push_back(inner_mats);
			}

			// Call Forward
			std::vector<std::vector<cv::Mat>> outputs = self.Forward(mats);

			// Convert outputs to Python list of lists of arrays
			py::list py_outputs;
			for (const std::vector<cv::Mat>& inner_mats : outputs) {
				py::list py_inner_outputs;
				for (const cv::Mat& mat : inner_mats) {
					py_inner_outputs.append(mat2array(mat));
				}
				py_outputs.append(py_inner_outputs);
			}

			return py_outputs;

		}, "Forwards data through the model")
		.def("Forward", [](me::dnn::MEDNNModuleInstance& self, py::list inputs) {
			// Convert inputs to std::vector<cv::Mat>
			std::vector<cv::Mat> mats;
			for (py::handle item : inputs) {
				mats.push_back(array2mat(item.cast<py::array>()));
			}

			// Call Forward
			std::vector<cv::Mat> outputs = self.Forward(mats);

			// Convert outputs to Python list of arrays
			py::list py_outputs;
			for (const cv::Mat& mat : outputs) {
				py_outputs.append(mat2array(mat));
			}

			return py_outputs;
		}, "Forwards data through the model")
		.def("Forward", [](me::dnn::MEDNNModuleInstance& self, py::array input) {
			// Convert input to cv::Mat
			cv::Mat mat = array2mat(input);

			// Call Forward
			cv::Mat output = self.Forward(mat);

			// Convert output to Python array
			return mat2array(output);
		}, "Forwards data through the model")
		.def("LoadModel", [](me::dnn::MEDNNModuleInstance& self, py::dict args) {
			// Convert py::dict to std::unordered_map
			std::unordered_map<std::string, std::string> map_args = args.cast<std::unordered_map<std::string, std::string>>();
			return self.LoadModel(map_args);
		}, "Load model with specified arguments")
		.def("SetForwardRules", [](me::dnn::MEDNNModuleInstance& self, py::dict args) {
			// Convert py::dict to std::unordered_map
			std::unordered_map<std::string, std::string> map_args = args.cast<std::unordered_map<std::string, std::string>>();
			return self.SetForwardRules(map_args);
		}, "Set forward rules with specified arguments")
		.def("UnloadModel", &me::dnn::MEDNNModuleInstance::UnloadModel, "Unload the model");
}