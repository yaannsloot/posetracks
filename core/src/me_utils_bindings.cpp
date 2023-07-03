/*
me_utils_bindings.cpp
Pybind11 module definitions for me_utils.hpp
 
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

#include <me_utils.hpp>
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>

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


PYBIND11_MODULE(MEUtils, m) {
	m.def("LetterboxImage", [](py::array src, py::tuple out_size) {
		cv::Mat mat_src = array2mat(src);
		cv::Mat mat_dst;

		cv::Size size_out = cv::Size(out_size[0].cast<int>(), out_size[1].cast<int>());

		std::vector<float> pad_info = me::utility::LetterboxImage(mat_src, mat_dst, size_out);

		return py::make_tuple(mat2array(mat_dst), pad_info);
		}, "Process image and return result as a numpy array with padding information");
	m.def("DoNMSForYOLO", [](py::array input, float conf_thresh, float iou_thresh, py::tuple image_dim) {
		cv::Mat mat_src = array2mat(input);
		cv::Size size = cv::Size(image_dim[0].cast<int>(), image_dim[1].cast<int>());
		cv::Mat mat_dst = me::utility::DoNMSForYOLO(mat_src, conf_thresh, iou_thresh, size);
		return mat2array(mat_dst);
		}, "Converts a raw YOLO detection matrix to OpenCV rect format");
	m.def("DoNMSForBox", [](py::array input, float conf_thresh, float nms_thresh) {
		return mat2array(me::utility::DoNMSForBox(array2mat(input), conf_thresh, nms_thresh));
		}, "Computes nms for a matrix in box detection format");

	py::class_<me::utility::MovieReader>(m, "MovieReader")
		.def(py::init<const std::string&>(), py::arg("movie_path"))
		.def("set_frame", [](me::utility::MovieReader& self, int frame) {
			self.set_frame(frame);
		}, "sets the current frame of the movie clip")
		.def("frame_count", &me::utility::MovieReader::frame_count, "Gets the total number of frames in the movie clip")
		.def("current_frame", &me::utility::MovieReader::current_frame, "Gets the current frame index in the movie clip")
		.def("frame_size", [](me::utility::MovieReader& self) {
			cv::Size size = self.frame_size();
			return py::make_tuple(size.width, size.height);
		}, "Gets the dimensions of the movie clip")
		.def("get_fps", &me::utility::MovieReader::get_fps, "Gets the fps of the movie clip")
		.def("frame", [](me::utility::MovieReader& self) {
			cv::Mat output;
			bool success = self.frame(output);
			return py::make_tuple(mat2array(output), success);
		}, "Retrieves the current frame and moves the frame counter forward")
		.def("grab_frame", [](me::utility::MovieReader& self, int frame) {
			cv::Mat output;
			bool success = self.grab_frame(frame, output);
			return py::make_tuple(mat2array(output), success);
		}, "Seeks to the specified frame and returns its contents")
		.def("close", &me::utility::MovieReader::close, "Closes the movie clip")
		.def("is_open", &me::utility::MovieReader::is_open, "Checks if the movie clip is open")
		.def("get_fourcc", &me::utility::MovieReader::get_fourcc, "Gets the FourCC file type code")
		;
}