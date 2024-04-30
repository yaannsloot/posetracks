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

#define PYBIND11_DETAILED_ERROR_MESSAGES

// Crypto module headers
#include "me/crypto/sha1.hpp"

// DNN module headers
#include "me/dnn/rtmdet.hpp"
#include "me/dnn/yolox.hpp"
#include "me/dnn/rtmpose.hpp"
#include "me/dnn/feature_extractor.hpp"
#include "me/dnn/tag_net.hpp"
#include "me/dnn/cv_tag_detector.hpp"
#include "me/dnn/pose_topdown.hpp"

// IO module headers
#include "me/io/transcoder.hpp"
#include "me/io/imagelist.hpp"

// Tracking module headers
#include "me/tracking/data.hpp"
#include "me/tracking/camera.hpp"

// Dependencies
#include <opencv2/sfm/triangulation.hpp>
#include <opencv2/sfm/projection.hpp>
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <filesystem>
#include <algorithm>
#include <random>
#include <limits>
#include <numeric>

namespace py = pybind11;

PYBIND11_MODULE(MEPython, m)
{
	// Submodules
	auto m_data = m.def_submodule("data");
	auto m_dnn = m.def_submodule("dnn");
	auto m_io = m.def_submodule("io");
	auto m_tracking = m.def_submodule("tracking");
	auto m_crypto = m.def_submodule("crypto");


	// Enum bindings

	// Base
	py::enum_<cv::aruco::PredefinedDictionaryType>(m, "TagDictionary")
		.value("DICT_4X4_50", cv::aruco::PredefinedDictionaryType::DICT_4X4_50)
		.value("DICT_4X4_100", cv::aruco::PredefinedDictionaryType::DICT_4X4_100)
		.value("DICT_4X4_250", cv::aruco::PredefinedDictionaryType::DICT_4X4_250)
		.value("DICT_4X4_1000", cv::aruco::PredefinedDictionaryType::DICT_4X4_1000)
		.value("DICT_5X5_50", cv::aruco::PredefinedDictionaryType::DICT_5X5_50)
		.value("DICT_5X5_100", cv::aruco::PredefinedDictionaryType::DICT_5X5_100)
		.value("DICT_5X5_250", cv::aruco::PredefinedDictionaryType::DICT_5X5_250)
		.value("DICT_5X5_1000", cv::aruco::PredefinedDictionaryType::DICT_5X5_1000)
		.value("DICT_6X6_50", cv::aruco::PredefinedDictionaryType::DICT_6X6_50)
		.value("DICT_6X6_100", cv::aruco::PredefinedDictionaryType::DICT_6X6_100)
		.value("DICT_6X6_250", cv::aruco::PredefinedDictionaryType::DICT_6X6_250)
		.value("DICT_6X6_1000", cv::aruco::PredefinedDictionaryType::DICT_6X6_1000)
		.value("DICT_7X7_50", cv::aruco::PredefinedDictionaryType::DICT_7X7_50)
		.value("DICT_7X7_100", cv::aruco::PredefinedDictionaryType::DICT_7X7_100)
		.value("DICT_7X7_250", cv::aruco::PredefinedDictionaryType::DICT_7X7_250)
		.value("DICT_7X7_1000", cv::aruco::PredefinedDictionaryType::DICT_7X7_1000)
		.value("DICT_ARUCO_ORIGINAL", cv::aruco::PredefinedDictionaryType::DICT_ARUCO_ORIGINAL)
		.value("DICT_APRILTAG_16h5", cv::aruco::PredefinedDictionaryType::DICT_APRILTAG_16h5)
		.value("DICT_APRILTAG_25h9", cv::aruco::PredefinedDictionaryType::DICT_APRILTAG_25h9)
		.value("DICT_APRILTAG_36h10", cv::aruco::PredefinedDictionaryType::DICT_APRILTAG_36h10)
		.value("DICT_APRILTAG_36h11", cv::aruco::PredefinedDictionaryType::DICT_APRILTAG_36h11)
		.export_values();

	// Core

	// DNN
	py::enum_<me::dnn::Precision>(m_dnn, "Precision")
		.value("FLOAT64", me::dnn::Precision::FLOAT64)
		.value("FLOAT32", me::dnn::Precision::FLOAT32)
		.value("FLOAT16", me::dnn::Precision::FLOAT16)
		.value("INT64", me::dnn::Precision::INT64)
		.value("INT32", me::dnn::Precision::INT32)
		.value("INT16", me::dnn::Precision::INT16)
		.value("INT8", me::dnn::Precision::INT8)
		.value("UNKNOWN", me::dnn::Precision::UNKNOWN)
		.value("NONE", me::dnn::Precision::NONE)
		.export_values();

	py::enum_<me::dnn::Executor>(m_dnn, "Executor")
		.value("TENSORRT", me::dnn::Executor::TENSORRT)
		.value("CUDA", me::dnn::Executor::CUDA)
		.value("CPU", me::dnn::Executor::CPU)
		.value("NONE", me::dnn::Executor::NONE)
		.export_values();

	py::enum_<me::dnn::FeatureDistanceType>(m_dnn, "FeatureDistanceType")
		.value("EUCLIDEAN", me::dnn::FeatureDistanceType::EUCLIDEAN)
		.value("NORM_EUCLIDEAN", me::dnn::FeatureDistanceType::NORM_EUCLIDEAN)
		.export_values();

	py::enum_<me::dnn::ScalingMode>(m_dnn, "ScalingMode")
		.value("AUTO", me::dnn::ScalingMode::AUTO)
		.value("NORMALIZE_INPUT", me::dnn::ScalingMode::NORMALIZE_INPUT)
		.value("DIRECT", me::dnn::ScalingMode::DIRECT)
		.export_values();

	// Class bindings

	// Base (used in submodules)
	py::class_<cv::Mat>(m, "Mat")
		.def(py::init<>())
		.def("from_array", [](cv::Mat& self, py::array_t<uint8_t>& im_data) {
			if (im_data.ndim() == 2) {
				cv::Mat mat(im_data.shape(0), im_data.shape(1), CV_8UC1, const_cast<uint8_t*>(im_data.data()), im_data.strides(0));
				self = mat;
			}
			else if (im_data.ndim() == 3) {
				cv::Mat mat(im_data.shape(0), im_data.shape(1), CV_MAKETYPE(CV_8U, im_data.shape(2)), 
					const_cast<uint8_t*>(im_data.data()), im_data.strides(0));
				self = mat;
			}
			else {
				throw std::invalid_argument("im_data.ndim != 2 || 3");
			}
		})
		.def("__repr__", [](cv::Mat& self) {
		std::stringstream ss;
		ss << "<cv::Mat: size=(" << self.cols << ',' << self.rows << "), type=" << self.type() << '>';
		return ss.str();
		});
	py::class_<cv::Point2d>(m, "Point")
		.def(py::init<>())
		.def(py::init<double, double>())
		.def_readwrite("x", &cv::Point2d::x)
		.def_readwrite("y", &cv::Point2d::y)
		.def("__repr__", [](cv::Point2d& self) {
			std::stringstream ss;
			ss << '(' << self.x << ", " << self.y << ')';
			return ss.str();
		});
	py::class_<cv::Point2f>(m, "Pointf")
		.def(py::init<>())
		.def(py::init<float, float>())
		.def_readwrite("x", &cv::Point2f::x)
		.def_readwrite("y", &cv::Point2f::y)
		.def("__repr__", [](cv::Point2f& self) {
			std::stringstream ss;
			ss << '(' << self.x << ", " << self.y << ')';
			return ss.str();
		});
	py::class_<cv::Rect2d>(m, "Rect")
		.def(py::init<>())
		.def(py::init<double, double, double, double>())
		.def(py::init<cv::Rect2d&>())
		.def("area", &cv::Rect2d::area)
		.def("br", &cv::Rect2d::br)
		.def("contains", &cv::Rect2d::contains)
		.def("empty", &cv::Rect2d::empty)
		.def("__eq__", [](const cv::Rect2d& self, const cv::Rect2d& other) {
			return self == other;
		})
		.def("__ne__", [](const cv::Rect2d& self, const cv::Rect2d& other) {
			return self != other;
		})
		.def("size", [](cv::Rect2d& self) {
			cv::Size size = self.size();
			return py::make_tuple(size.width, size.height);
		})
		.def("tl", &cv::Rect2d::tl)
		.def_readwrite("x", &cv::Rect2d::x)
		.def_readwrite("y", &cv::Rect2d::y)
		.def_readwrite("width", &cv::Rect2d::width)
		.def_readwrite("height", &cv::Rect2d::height);

	
	// Data



	// IO
	py::class_<me::io::FrameProvider>(m_io, "FrameProvider")
		.def(py::init<>())
		.def("load", &me::io::FrameProvider::load, py::call_guard<py::gil_scoped_release>(), py::arg("path"), py::arg("use_hw_accel") = false)
		.def("next_frame", &me::io::FrameProvider::next_frame, py::call_guard<py::gil_scoped_release>(), py::arg("frame"), py::arg("retry_count") = 100)
		.def("grab_frame", &me::io::FrameProvider::grab_frame, py::call_guard<py::gil_scoped_release>(), py::arg("frame"), py::arg("frame_id"), py::arg("retry_count") = 100)
		.def("set_frame", &me::io::FrameProvider::set_frame, py::call_guard<py::gil_scoped_release>(), py::arg("frame_id"))
		.def("current_frame", &me::io::FrameProvider::current_frame)
		.def("frame_count", &me::io::FrameProvider::frame_count)
		.def("frame_size", [](me::io::FrameProvider& self) {
			cv::Size size = self.frame_size();
			return py::make_tuple(size.width, size.height);
		})
		.def("fps", &me::io::FrameProvider::fps)
		.def("close", &me::io::FrameProvider::close)
		.def("is_open", &me::io::FrameProvider::is_open)
		.def("get_fourcc_str", &me::io::FrameProvider::get_fourcc_str)
		.def("get_fourcc", &me::io::FrameProvider::get_fourcc);
	py::class_<me::io::ImageList, me::io::FrameProvider>(m_io, "ImageList")
		.def(py::init<>());
	py::class_<me::io::Transcoder, me::io::FrameProvider>(m_io, "Transcoder")
		.def(py::init<>());


	// DNN
	py::class_<me::dnn::Detection>(m_dnn, "Detection")
		.def(py::init<>())
		.def(py::init<int, cv::Rect2d, float>(), py::arg("class_id"), py::arg("bbox"), py::arg("score"))
		.def_readwrite("class_id", &me::dnn::Detection::class_id)
		.def_readwrite("bbox", &me::dnn::Detection::bbox)
		.def_readwrite("score", &me::dnn::Detection::score)
		.def("scale_detection", &me::dnn::Detection::scale_detection);
	py::class_<me::dnn::Joint>(m_dnn, "Joint")
		.def(py::init<>())
		.def(py::init<double, double, double>())
		.def(py::init<cv::Point2d, double>())
		.def_readwrite("prob", &me::dnn::Joint::prob)
		.def_readwrite("pt", &me::dnn::Joint::pt);
	py::class_<me::dnn::Pose>(m_dnn, "Pose")
		.def(py::init<>())
		.def("set_joint", (void (me::dnn::Pose::*)(int, me::dnn::Joint&)) & me::dnn::Pose::set_joint)
		.def("set_joint", (void (me::dnn::Pose::*)(int, cv::Point2d&, double)) & me::dnn::Pose::set_joint)
		.def("set_joint", (void (me::dnn::Pose::*)(int, double, double, double)) & me::dnn::Pose::set_joint)
		.def("get_joint", &me::dnn::Pose::get_joint, py::return_value_policy::reference)
		.def("has_joint", &me::dnn::Pose::has_joint)
		.def("get_joint_ids", &me::dnn::Pose::get_joint_ids)
		.def("num_joints", &me::dnn::Pose::num_joints)
		.def("__getitem__", &me::dnn::Pose::operator[], py::return_value_policy::reference);
	py::class_<me::dnn::Feature>(m_dnn, "Feature")
		.def(py::init<>())
		.def(py::init<std::vector<double>&>())
		.def("norm", &me::dnn::Feature::norm, py::call_guard<py::gil_scoped_release>())
		.def("__truediv__", &me::dnn::Feature::operator/, py::call_guard<py::gil_scoped_release>())
		.def("__sub__", &me::dnn::Feature::operator-, py::call_guard<py::gil_scoped_release>())
		.def("__assign__", &me::dnn::Feature::operator=)
		.def("dist", &me::dnn::Feature::dist, py::call_guard<py::gil_scoped_release>(), py::arg("other"), py::arg("d_type") = me::dnn::FeatureDistanceType::NORM_EUCLIDEAN)
		.def("__len__", &me::dnn::Feature::size)
		.def("__getitem__", [](me::dnn::Feature& self, size_t index) {
			return self.data[index];
		})
		.def("__setitem__", [](me::dnn::Feature& self, size_t index, double val) {
			self.data[index] = val;
		})
		.def("__str__", [](me::dnn::Feature& self) {
			std::stringstream ss;
			ss << "[";
			for (size_t i = 0; i < self.data.size(); ++i) {
				ss << self.data[i];
				if (i < self.data.size() - 1) {
					ss << ", ";
				}
			}
			ss << "]";
			return ss.str();
		}, py::call_guard<py::gil_scoped_release>());
	py::class_<me::dnn::Tag>(m_dnn, "Tag")
		.def(py::init<>())
		.def(py::init<int>())
		.def(py::init<cv::Point2d&, cv::Point2d&, cv::Point2d&, cv::Point2d&>())
		.def(py::init<int, cv::Point2d&, cv::Point2d&, cv::Point2d&, cv::Point2d&>())
		.def(py::init<int, double>())
		.def(py::init<double, cv::Point2d&, cv::Point2d&, cv::Point2d&, cv::Point2d&>())
		.def(py::init<int, double, cv::Point2d&, cv::Point2d&, cv::Point2d&, cv::Point2d&>())
		.def(py::init<const std::pair<double, double>&, const std::pair<double, double>&, const std::pair<double, double>&, const std::pair<double, double>&>())
		.def(py::init<int, const std::pair<double, double>&, const std::pair<double, double>&, const std::pair<double, double>&, const std::pair<double, double>&>())
		.def(py::init<double, const std::pair<double, double>&, const std::pair<double, double>&, const std::pair<double, double>&, const std::pair<double, double>&>())
		.def(py::init<int, double, const std::pair<double, double>&, const std::pair<double, double>&, const std::pair<double, double>&, const std::pair<double, double>&>())
		.def("__getitem__", [](me::dnn::Tag& self, size_t index) {
			if (index >= 4 || index < 0) throw py::index_error();
			return self.corners[index];
			})
		.def("__setitem__", [](me::dnn::Tag& self, size_t index, cv::Point2d& value) {
			if (index >= 4 || index < 0) throw py::index_error();
			self[index] = value;
		})
		.def("__setitem__", [](me::dnn::Tag& self, size_t index, const std::pair<double, double> value) {
			if (index >= 4 || index < 0) throw py::index_error();
			self[index] = cv::Point2d(value.first, value.second);
		})
		.def_readwrite("id", &me::dnn::Tag::id)
		.def_readwrite("conf", &me::dnn::Tag::conf);
	py::class_<me::dnn::FeatureSet>(m_dnn, "FeatureSet")
		.def(py::init<size_t>())
		.def("add", &me::dnn::FeatureSet::add, py::call_guard<py::gil_scoped_release>())
		.def("at", &me::dnn::FeatureSet::at)
		.def("remove", &me::dnn::FeatureSet::remove, py::call_guard<py::gil_scoped_release>())
		.def("mean", &me::dnn::FeatureSet::mean)
		.def("size", &me::dnn::FeatureSet::size)
		.def("length", &me::dnn::FeatureSet::length)
		.def("__getitem__", [](me::dnn::FeatureSet& self, size_t index) {
			return self[index];
		});
	py::class_<me::dnn::FeatureSpace>(m_dnn, "FeatureSapce")
		.def(py::init<size_t>())
		.def("assign", py::overload_cast<me::dnn::Feature&, double, me::dnn::FeatureDistanceType>(&me::dnn::FeatureSpace::assign),
			py::call_guard<py::gil_scoped_release>(),
			py::arg("input"), py::arg("threshold") = 0.4, py::arg("dist_type") = me::dnn::FeatureDistanceType::NORM_EUCLIDEAN)
		.def("assign", py::overload_cast<std::vector<me::dnn::Feature>&, double, me::dnn::FeatureDistanceType, std::vector<int>>(&me::dnn::FeatureSpace::assign),
			py::call_guard<py::gil_scoped_release>(),
			py::arg("input"), py::arg("threshold") = 0.4, py::arg("dist_type") = me::dnn::FeatureDistanceType::NORM_EUCLIDEAN, py::arg("mask") = std::vector<int>())
		.def("size", &me::dnn::FeatureSpace::size)
		.def("length", &me::dnn::FeatureSpace::length)
		.def("clear", &me::dnn::FeatureSpace::clear)
		.def("at", &me::dnn::FeatureSpace::at, py::return_value_policy::reference)
		.def("__getitem__", [](me::dnn::FeatureSpace& self, size_t index) {
			return self[index];
		}, py::return_value_policy::reference);
	py::class_<me::dnn::FeatureTracker>(m_dnn, "FeatureTracker")
		.def(py::init<size_t>())
		.def("assign", &me::dnn::FeatureTracker::assign,
			py::call_guard<py::gil_scoped_release>(),
			py::arg("input_boxes"), py::arg("input_features"), py::arg("score_threshold") = 0.7, py::arg("f_space_threshold") = 0.4,
			py::arg("dist_type") = me::dnn::FeatureDistanceType::NORM_EUCLIDEAN, py::arg("mask") = std::vector<int>());

	
	// DNN Model class bindings

	// Proxy classes
	py::class_<me::dnn::models::Model>(m_dnn, "Model")
		.def(py::init<>())
		.def("load", &me::dnn::models::Model::load, py::call_guard<py::gil_scoped_release>(), py::arg("model_path"), py::arg("target_executor") = me::dnn::Executor::CPU)
		.def("unload", &me::dnn::models::Model::unload)
		.def("is_loaded", &me::dnn::models::Model::is_loaded)
		.def("get_precision", &me::dnn::models::Model::get_precision)
		.def("get_executor", &me::dnn::models::Model::get_executor);
	py::class_<me::dnn::models::ImageModel, me::dnn::models::Model>(m_dnn, "ImageModel")
		.def(py::init<>())
		.def("net_size", [](me::dnn::models::ImageModel& self) {
			cv::Size size = self.net_size();
			return py::make_tuple(size.width, size.height);
		});
	py::class_<me::dnn::models::DetectionModel, me::dnn::models::ImageModel>(m_dnn, "DetectionModel")
		.def(py::init<>())
		.def("infer", [](me::dnn::models::DetectionModel& self, const cv::Mat& image, float conf_thresh, float iou_thresh) {
			std::vector<me::dnn::Detection> result;
			self.infer(image, result, conf_thresh, iou_thresh);
			return result;
		}, py::call_guard<py::gil_scoped_release>(), py::arg("image"), py::arg("conf_thresh") = 0.5, py::arg("iou_thresh") = 0.5)
		.def("infer", [](me::dnn::models::DetectionModel& self, const std::vector<cv::Mat>& images, float conf_thresh, float iou_thresh) {
			std::vector<std::vector<me::dnn::Detection>> result;
			self.infer(images, result, conf_thresh, iou_thresh);
			return result;
		}, py::call_guard<py::gil_scoped_release>(), py::arg("images"), py::arg("conf_thresh") = 0.5, py::arg("iou_thresh") = 0.5);
	py::class_<me::dnn::models::PoseModel, me::dnn::models::ImageModel>(m_dnn, "PoseModel")
		.def(py::init<>())
		.def("infer", [](me::dnn::models::PoseModel& self, const cv::Mat& image) {
			me::dnn::Pose result;
			self.infer(image, result);
			return result;
		}, py::call_guard<py::gil_scoped_release>())
		.def("infer", [](me::dnn::models::PoseModel& self, const std::vector<cv::Mat>& images) {
			std::vector<me::dnn::Pose> result;
			self.infer(images, result);
			return result;
		}, py::call_guard<py::gil_scoped_release>());
	py::class_<me::dnn::models::FeatureModel, me::dnn::models::ImageModel>(m_dnn, "FeatureModel")
		.def(py::init<>())
		.def("infer", [](me::dnn::models::FeatureModel& self, const cv::Mat& image) {
			me::dnn::Feature result;
			self.infer(image, result);
			return result;
		}, py::call_guard<py::gil_scoped_release>())
		.def("infer", [](me::dnn::models::FeatureModel& self, const std::vector<cv::Mat>& images) {
			std::vector<me::dnn::Feature> result;
			self.infer(images, result);
			return result;
		}, py::call_guard<py::gil_scoped_release>());
	py::class_<me::dnn::models::TagModel, me::dnn::models::ImageModel>(m_dnn, "TagModel")
		.def(py::init<>())
		.def("infer", [](me::dnn::models::TagModel& self, const cv::Mat& image) {
			std::vector<me::dnn::Tag> result;
			self.infer(image, result);
			return result;
		}, py::call_guard<py::gil_scoped_release>())
		.def("infer", [](me::dnn::models::TagModel& self, const std::vector<cv::Mat>& images) {
			std::vector<std::vector<me::dnn::Tag>> result;
			self.infer(images, result);
			return result;
		}, py::call_guard<py::gil_scoped_release>());


	// Driver classes
	py::class_<me::dnn::models::RTMDetModel, me::dnn::models::DetectionModel>(m_dnn, "RTMDetModel")
		.def(py::init<>());
	py::class_<me::dnn::models::YOLOXModel, me::dnn::models::DetectionModel>(m_dnn, "YOLOXModel")
		.def(py::init<>());
	py::class_<me::dnn::models::RTMPoseModel, me::dnn::models::PoseModel>(m_dnn, "RTMPoseModel")
		.def(py::init<>());
	py::class_<me::dnn::models::GenericFeatureModel, me::dnn::models::FeatureModel>(m_dnn, "GenericFeatureModel")
		.def(py::init<>());
	py::class_<me::dnn::models::TagNetModel, me::dnn::models::TagModel>(m_dnn, "TagNetModel")
		.def(py::init<>());
	py::class_<me::dnn::models::CVTagDetector, me::dnn::models::TagModel>(m_dnn, "CVTagDetector")
		.def(py::init<>())
		.def("set_dict_type", &me::dnn::models::CVTagDetector::set_dict_type)
		.def("set_preprocess_size", [](me::dnn::models::CVTagDetector& self, std::pair<int, int> new_size) {
			self.set_preprocess_size(cv::Size(new_size.first, new_size.second));
		});

	py::class_<me::dnn::models::TopDownPoseDetector>(m_dnn, "TopDownPoseDetector")
		.def(py::init<>())
		.def("unload_all", &me::dnn::models::TopDownPoseDetector::unload_all)
		.def("infer", [](me::dnn::models::TopDownPoseDetector& self, const cv::Mat& image, int max_pose_batches, float conf_thresh, float iou_thresh) {
		std::vector<me::dnn::Pose> result;
			self.infer(image, result, max_pose_batches, conf_thresh, iou_thresh);
			return result;
		}, py::call_guard<py::gil_scoped_release>(), py::arg("image"), py::arg("max_pose_batches") = 1, py::arg("conf_thresh") = 0.5, py::arg("iou_thresh") = 0.5)
		.def("infer", [](me::dnn::models::TopDownPoseDetector& self, const std::vector<cv::Mat>& images, int max_detection_batches, int max_pose_batches, float conf_thresh, float iou_thresh) {
			std::vector<std::vector<me::dnn::Pose>> result;
			self.infer(images, result, max_detection_batches, max_pose_batches, conf_thresh, iou_thresh);
			return result;
		}, py::call_guard<py::gil_scoped_release>(), py::arg("images"), py::arg("max_detection_batches") = 1, py::arg("max_pose_batches") = 1, py::arg("conf_thresh") = 0.5, py::arg("iou_thresh") = 0.5)
		.def("is_ready", &me::dnn::models::TopDownPoseDetector::is_ready)
		.def_readwrite("detection_model", &me::dnn::models::TopDownPoseDetector::detection_model)
		.def_readwrite("pose_model", &me::dnn::models::TopDownPoseDetector::pose_model);

	// Tracking
	py::class_<me::tracking::Mat3x1>(m_tracking, "Mat3x1")
		.def(py::init<>())
		.def("__getitem__", [](me::tracking::Mat3x1& self, int row) {
			return self(row);
		})
		.def("__setitem__", [](me::tracking::Mat3x1& self, int row, double val) {
			self(row) = val;
		})
		.def("__str__", [](me::tracking::Mat3x1& self) {
			std::stringstream ss;
			ss << cv::format(self, cv::Formatter::FMT_PYTHON);
			return ss.str();
		});
	py::class_<me::tracking::Mat1x3>(m_tracking, "Mat1x3")
		.def(py::init<>())
		.def("__getitem__", [](me::tracking::Mat1x3& self, int col) {
			return self(col);
		})
		.def("__setitem__", [](me::tracking::Mat1x3& self, int col, double val) {
			self(col) = val;
		})
		.def("__str__", [](me::tracking::Mat1x3& self) {
			std::stringstream ss;
			ss << cv::format(self, cv::Formatter::FMT_PYTHON);
			return ss.str();
		});
	py::class_<me::tracking::Mat3x3>(m_tracking, "Mat3x3")
		.def(py::init<>())
		.def("__getitem__", [](me::tracking::Mat3x3& self, std::pair<int, int> pos) {
			return self(pos.first, pos.second);
		})
		.def("__setitem__", [](me::tracking::Mat3x3& self, std::pair<int, int> pos, double val) {
			self(pos.first, pos.second) = val;
		})
		.def("__str__", [](me::tracking::Mat3x3& self) {
			std::stringstream ss;
			ss << cv::format(self, cv::Formatter::FMT_PYTHON);
			return ss.str();
		});
	py::class_<me::tracking::Mat4x4>(m_tracking, "Mat4x4")
		.def(py::init<>())
		.def("__getitem__", [](me::tracking::Mat4x4& self, std::pair<int, int> pos) {
			return self(pos.first, pos.second);
		})
		.def("__setitem__", [](me::tracking::Mat4x4& self, std::pair<int, int> pos, double val) {
			self(pos.first, pos.second) = val;
		})
		.def("__str__", [](me::tracking::Mat4x4& self) {
			std::stringstream ss;
			ss << cv::format(self, cv::Formatter::FMT_PYTHON);
			return ss.str();
		});
	py::class_<me::tracking::Rt>(m_tracking, "Rt")
		.def(py::init<>())
		.def_readwrite("R", &me::tracking::Rt::R)
		.def_readwrite("t", &me::tracking::Rt::t)
		.def("invert", &me::tracking::Rt::invert)
		.def("to4x4", &me::tracking::Rt::to4x4)
		.def("from4x4", &me::tracking::Rt::from4x4)
		.def("is_identity", &me::tracking::Rt::is_identity);
	py::class_<me::tracking::Kk>(m_tracking, "Kk")
		.def(py::init<>())
		.def_readwrite("K", &me::tracking::Kk::K)
		.def_readwrite("k", &me::tracking::Kk::k);
	py::class_<me::tracking::TrackingData>(m_tracking, "TrackingData")
		.def(py::init<>())
		.def_readwrite("poses", &me::tracking::TrackingData::poses)
		.def_readwrite("detections", &me::tracking::TrackingData::detections)
		.def_readwrite("tags", &me::tracking::TrackingData::tags)
		.def("to_points", &me::tracking::TrackingData::to_points, py::call_guard<py::gil_scoped_release>(), py::arg("reduce_tags") = false);

	// Function bindings

	// Base
		m.def("imread", [](std::string filename) {
			cv::Mat image = cv::imread(filename);
			auto return_obj = py::cast(image);
			if (image.data == NULL)
				return_obj = py::none();
			return return_obj;
		}, py::call_guard<py::gil_scoped_release>());
		m.def("imwrite", [](std::string filename, cv::Mat& image) {
			cv::imwrite(filename, image);
		}, py::call_guard<py::gil_scoped_release>());
		m.def("imshow", [](std::string window_name, cv::Mat& image) {
			cv::imshow(window_name, image);
		});
		m.def("resize_img", [](cv::Mat& input, std::pair<int, int> out_size) {
			cv::Mat result;
			cv::resize(input, result, cv::Size(out_size.first, out_size.second));
			return result;
		});
		m.def("waitKey", &cv::waitKey);


	// Needs adjustments future Ian please fix
	m.def("triangulate_points", [](
		py::object clips,
		py::object pose_tracks_list,
		py::object cam_matrices,
		py::object dist_vectors,
		py::object bpy_scene,
		py::object bpy_data,
		bool use_all_tracks,
		bool non_destructive
		) {

			std::chrono::microseconds init(0);
			std::chrono::microseconds blend_read(0);
			std::chrono::microseconds blend_write(0);
			std::chrono::microseconds triangulate(0);

			auto start = std::chrono::high_resolution_clock::now();
			
			// SAME OMEGA NOTE FROM THE SOLVE CAMERAS FUNCTION.
			// This NEEDS to be modified to take into account frame offsets in future versions when that gets implemented
			// It currently DOES NOT DO THAT
			// Also, add pose/non-pose filtering and implement bool flags. Besides that, works as expected. Supports multiple camera views

			// PART 0:
			// Get preliminary information and variables

			// Clip information
			std::vector<py::object> clip_list = clips.cast<std::vector<py::object>>();
			auto clip_count = clip_list.size();
			std::vector<cv::Size> clip_sizes;
			std::vector<std::string> clip_names;
			std::vector<py::object> clip_tracking_tracks;
			std::vector<py::object> clip_pose_tracks_list;

			for (int i = 0; i < clip_count; ++i) {
				py::object clip_obj = clip_list[i];
				py::list clip_size = clip_obj.attr("size");
				clip_sizes.push_back(cv::Size(clip_size[0].cast<int>(), clip_size[1].cast<int>()));
				clip_names.push_back(clip_obj.attr("name").cast<std::string>());
				clip_tracking_tracks.push_back(clip_obj.attr("tracking").attr("tracks"));
				py::object pose_tracks = py::none();
				for (auto& pose_tracks_entry : pose_tracks_list) {
					py::object entry = pose_tracks_entry.cast<py::object>();
					py::object entry_clip = entry.attr("clip");
					if (entry_clip.equal(clip_obj)) {
						pose_tracks = entry.attr("pose_tracks_list");
						break;
					}
				}
				clip_pose_tracks_list.push_back(pose_tracks);
			}

			// Convert camera matrices and distortion vectors to opencv compatible formats
			py::list cam_matrix_list = cam_matrices.cast<py::list>();
			py::list dist_vector_list = dist_vectors.cast<py::list>();
			std::vector<cv::Mat> cv_cam_matrices;
			std::vector<std::vector<float>> cv_dist_vectors;

			for (int i = 0; i < cam_matrix_list.size(); ++i) {
				py::object py_cam_mtx = cam_matrix_list[i];
				py::list py_dist_vector = dist_vector_list[i];
				cv::Mat cv_cam_mtx(3, 3, CV_64FC1);
				std::vector<float> cv_dist_vector;
				for (int r = 0; r < cv_cam_mtx.rows; ++r) {
					for (int c = 0; c < cv_cam_mtx.cols; ++c) {
						cv_cam_mtx.at<double>(r, c) = py_cam_mtx.attr("__getitem__")(r).attr("__getitem__")(c).cast<float>();
					}
				}
				for (int j = 0; j < py_dist_vector.size(); ++j) {
					cv_dist_vector.push_back(py_dist_vector[j].cast<float>());
				}
				cv_cam_matrices.push_back(cv_cam_mtx);
				cv_dist_vectors.push_back(cv_dist_vector);
			}

			// Get scene cameras
			std::vector<py::object> scene_cameras;
			py::object scene_objects = bpy_scene.attr("objects");
			for (auto& obj : scene_objects) {
				py::object object = obj.cast<py::object>();
				if (object.attr("type").cast<std::string>() == "CAMERA")
					scene_cameras.push_back(object);
			}

			// Create a vector of camera objects indexed the same way as clip_names
			// Each index will either contain a camera object with a matching name, or None
			std::vector<py::object> clip_cameras;
			for (auto& name : clip_names) {
				py::object clip_camera = py::none();
				for (auto& cam : scene_cameras) {
					py::object camera = cam.cast<py::object>();
					if (camera.attr("name").cast<std::string>() == name) {
						clip_camera = camera;
						break;
					}
				}
				clip_cameras.push_back(clip_camera);
			}

			// Remove elements from other vectors if their index references a None object in the clip_cameras vector
			for (int i = clip_cameras.size() - 1; i >= 0; --i) {
				if (clip_cameras[i].is_none()) {
					clip_list.erase(clip_list.begin() + i);
					clip_sizes.erase(clip_sizes.begin() + i);
					clip_names.erase(clip_names.begin() + i);
					clip_tracking_tracks.erase(clip_tracking_tracks.begin() + i);
					clip_pose_tracks_list.erase(clip_pose_tracks_list.begin() + i);
					cv_cam_matrices.erase(cv_cam_matrices.begin() + i);
					cv_dist_vectors.erase(cv_dist_vectors.begin() + i);
					clip_cameras.erase(clip_cameras.begin() + i);
				}
			}

			if(clip_cameras.empty())
				throw std::runtime_error("No valid cameras were found in the current scene");

			if(clip_cameras.size() < 2)
				throw std::runtime_error("A minimum of two valid cameras are required for triangulation");

			
			// Coordinate conversion matrix obtained from experimentation with OpenCV <-> Blender transforms
			cv::Mat coordinate_flip = (cv::Mat_<double>(4, 4) <<
				1, -1, -1, 1,
				-1, 1, 1, -1,
				-1, 1, 1, -1,
				1, -1, -1, 1);

			// Finally, obtain projection matrices from cameras
			std::vector<cv::Mat> cv_proj_matrices;
			for (int i = 0; i < clip_cameras.size(); ++i) {
				py::object cam = clip_cameras[i];
				py::object bpy_cam_mtx = cam.attr("matrix_world").attr("normalized")();
				cv::Mat cv_bpy_cam_mtx(4, 4, CV_64FC1);

				for (int r = 0; r < cv_bpy_cam_mtx.rows; ++r) {
					for (int c = 0; c < cv_bpy_cam_mtx.cols; ++c) {
						cv_bpy_cam_mtx.at<double>(r, c) = bpy_cam_mtx.attr("__getitem__")(r).attr("__getitem__")(c).cast<float>();
					}
				}

				cv::multiply(cv_bpy_cam_mtx, coordinate_flip, cv_bpy_cam_mtx);

				cv::Mat R = cv_bpy_cam_mtx(cv::Rect(0, 0, 3, 3)).clone();
				cv::Mat t = cv_bpy_cam_mtx(cv::Rect(3, 0, 1, 3)).clone();

				R = R.t();
				t = -R * t;

				cv::Mat P;
				cv::sfm::projectionFromKRt(cv::Mat::eye(3, 3, CV_64FC1), R, t, P);
				cv_proj_matrices.push_back(P);
			}

			// PART 1:
			// Triangulation

			// Build track map vector. The vector is indexed the same as the other vectors obtained from PART 0.
			// Each map will be index as [trackname] = track_obj
			std::vector<std::unordered_map<std::string, py::object>> track_map;
			for (int i = 0; i < clip_list.size(); ++i) {
				std::unordered_map<std::string, py::object> clip_map;
				py::object clip_tracks = clip_list[i].attr("tracking").attr("tracks");
				for (auto& trk : clip_tracks) {
					py::object track = trk.cast<py::object>();
					clip_map[track.attr("name").cast<std::string>()] = track;
				}
				track_map.push_back(clip_map);
			}

			// List of names that have matching frames in at least two clips
			std::unordered_set<std::string> target_tracks;


			auto end = std::chrono::high_resolution_clock::now();

			init += std::chrono::duration_cast<std::chrono::microseconds>(end - start);

			std::unordered_map<std::string, std::unordered_map<int, cv::Point3d>> animation_data;

			// Loop through each frame in the scene
			int frame_start = bpy_scene.attr("frame_start").cast<int>();
			int frame_end = bpy_scene.attr("frame_end").cast<int>();

			for (int f = frame_start; f <= frame_end; ++f) {

				// Find tracks with matching names on two or more clips and make sure they have markers present on the current frame

				start = std::chrono::high_resolution_clock::now();

				// Subset of track_map that contains only references to tracks on the current frame
				std::vector<std::unordered_map<std::string, py::object>> tracks_on_frame;

				// Fill tracks_on_frame
				// The clip may not actually have any relevant track data for the current frame.
				// If so, it will just be indexed with an empty map
				for (int i = 0; i < track_map.size(); ++i) {
					std::unordered_map<std::string, py::object> clip_tracks_on_frame;
					auto& clip_track_map = track_map[i];
					for (auto& element : clip_track_map) {
						if (!element.second.attr("markers").attr("find_frame")(f).is_none())
							clip_tracks_on_frame.insert(element);
					}
					tracks_on_frame.push_back(clip_tracks_on_frame);
				}

				// Identical to tracks_on_frame, except information is rearranged
				// This is a map indexed by track name that contains a vector of the indices of each clip the track appears on in the current frame
				std::unordered_map<std::string, std::vector<int>> track_buckets;

				// Fill track_buckets
				for (int i = 0; i < tracks_on_frame.size(); ++i) {
					auto& clip_tracks_on_frame = tracks_on_frame[i];
					for (auto& element : clip_tracks_on_frame) {
						if (track_buckets.find(element.first) == track_buckets.end())
							track_buckets[element.first] = std::vector<int>();
						track_buckets[element.first].push_back(i);
					}
				}

				// Trim track_buckets.
				// This is important as some tracks may not have the minimum number of views needed for triangulation
				for (auto it = track_buckets.begin(); it != track_buckets.end(); ) {
					if (it->second.size() < 2) {
						it = track_buckets.erase(it);
					}
					else {
						++it;
					}
				}

				end = std::chrono::high_resolution_clock::now();

				blend_read += std::chrono::duration_cast<std::chrono::microseconds>(end - start);


				// Iterate over each track bucket and triangulate referenced points

				for (auto& element : track_buckets) {
					auto& track_name = element.first;
					auto& clip_indices = element.second;

					std::vector<cv::Mat> track_points;
					std::vector<cv::Mat> proj_matrices;

					// Read the point referenced in each track, 
					// convert its frame coordinates to opencv, 
					// undistort and normalize these coordinates, 
					// and add them to the track_points vector.
					// Also, add the projection matrix for this track's view to the proj_matrices vector
					for (auto& index : clip_indices) {

						start = std::chrono::high_resolution_clock::now();

						auto& track = tracks_on_frame[index][track_name];
						py::object marker = track.attr("markers").attr("find_frame")(f);
						py::object co = marker.attr("co");
						py::object pattern_bbox = marker.attr("pattern_bound_box");
						auto& clip_size = clip_sizes[index];
						float x = co.attr("__getitem__")(0).cast<float>() * clip_size.width;
						float y = clip_size.height - co.attr("__getitem__")(1).cast<float>() * clip_size.height;
						cv::Point2f tl(
							pattern_bbox.attr("__getitem__")(0).attr("__getitem__")(0).cast<float>() * clip_size.width,
							pattern_bbox.attr("__getitem__")(0).attr("__getitem__")(1).cast<float>() * clip_size.height
						);
						cv::Point2f br(
							pattern_bbox.attr("__getitem__")(1).attr("__getitem__")(0).cast<float>()* clip_size.width,
							pattern_bbox.attr("__getitem__")(1).attr("__getitem__")(1).cast<float>()* clip_size.height
						);

						// prob var has no use rn. If no use is decided upon in the future please remove this part
						double prob = std::abs(tl.x - br.x) * std::abs(tl.y - br.y) / 100;

						auto& cam_mtx = cv_cam_matrices[index];
						auto& dist_vector = cv_dist_vectors[index];
						std::vector<cv::Point2f> src = { cv::Point2f(x, y) };
						cv::Mat dst;

						end = std::chrono::high_resolution_clock::now();

						blend_read += std::chrono::duration_cast<std::chrono::microseconds>(end - start);

						start = std::chrono::high_resolution_clock::now();

						cv::undistortPoints(src, dst, cam_mtx, dist_vector);

						cv::Mat dst_mat(dst);
						dst_mat = dst_mat.reshape(1, 2); // reshape to a 2xN matrix
						track_points.push_back(dst_mat);
						proj_matrices.push_back(cv_proj_matrices[index]);

						end = std::chrono::high_resolution_clock::now();

						triangulate += std::chrono::duration_cast<std::chrono::microseconds>(end - start);

					}


					// Triangulate the points using the vectors obtained in the previous step
					
					// TODO: Add a reprojection error check
					// Compare reprojected 3d points to the source 2d points via euclidean distance
					// You can average this or compare individually, idk. Experiment.
					// Remember that the 2d coords are x (u, v, w) = P * X (x, y, z, 1)
					// u/w = x and v/w = y
					
					// TODO: ALSO either here or somewhere else, add an "erroneous change in position" filer.
					// Not sure what this is actually called, but last time this was implemented via
					// taking the delta of each axis over time and filtering for outliers.

					cv::Mat points3d;

					start = std::chrono::high_resolution_clock::now();

					cv::sfm::triangulatePoints(track_points, proj_matrices, points3d);

					end = std::chrono::high_resolution_clock::now();

					triangulate += std::chrono::duration_cast<std::chrono::microseconds>(end - start);

					if (animation_data.find(track_name) == animation_data.end())
						animation_data[track_name] = std::unordered_map<int, cv::Point3d>();

					animation_data[track_name][f] = cv::Point3d(
						points3d.at<double>(0),
						-points3d.at<double>(1),
						-points3d.at<double>(2)
					);

				}

			}

			// PART 2:
			// Write animation data to Blender

			// Set that keeps track of track names that have had some form of initialization performed on them
			std::unordered_set<std::string> track_initialization_list;

			start = std::chrono::high_resolution_clock::now();

			for (auto& track_anim_data : animation_data) {
				auto& track_name = track_anim_data.first;
				auto& track_points = track_anim_data.second;

				// Ensure track object exists in blender & is present in a collection named MotionEngine

				py::object point_collection = bpy_data.attr("collections").attr("get")("MotionEngine");

				if (point_collection.is_none())
					point_collection = bpy_data.attr("collections").attr("new")("MotionEngine");

				if (!bpy_scene.attr("collection").attr("children").contains(point_collection.attr("name")))
					bpy_scene.attr("collection").attr("children").attr("link")(point_collection);

				py::object track_empty = bpy_scene.attr("objects").attr("get")(track_name);

				if (track_empty.is_none()) {
					track_empty = bpy_data.attr("objects").attr("new")(track_name, py::none());
					track_empty.attr("empty_display_type") = "SPHERE";
					bpy_scene.attr("collection").attr("objects").attr("link")(track_empty);
				}

				if (!point_collection.attr("objects").contains(track_empty.attr("name")))
					point_collection.attr("objects").attr("link")(track_empty);

				if (track_initialization_list.find(track_name) == track_initialization_list.end()) {
					track_empty.attr("animation_data_clear")();
					track_initialization_list.insert(track_name);
				}

				// Prepare fcurves

				py::object track_fcurves = track_empty.attr("animation_data");

				if (track_fcurves.is_none())
					track_fcurves = track_empty.attr("animation_data_create")();

				track_fcurves = track_fcurves.attr("action");

				if (track_fcurves.is_none()) {
					py::object new_action = bpy_data.attr("actions").attr("new")(track_name + "Action");
					track_empty.attr("animation_data").attr("action") = new_action;
					track_fcurves = new_action;
				}

				track_fcurves = track_fcurves.attr("fcurves");

				py::object fcurve_x = track_fcurves.attr("find")("location", py::arg("index") = 0);
				py::object fcurve_y = track_fcurves.attr("find")("location", py::arg("index") = 1);
				py::object fcurve_z = track_fcurves.attr("find")("location", py::arg("index") = 2);

				if (fcurve_x.is_none()) {
					fcurve_x = track_fcurves.attr("new")("location", py::arg("index") = 0);
				}
				if (fcurve_y.is_none()) {
					fcurve_y = track_fcurves.attr("new")("location", py::arg("index") = 1);
				}
				if (fcurve_z.is_none()) {
					fcurve_z = track_fcurves.attr("new")("location", py::arg("index") = 2);
				}

				py::object keyframes_x = fcurve_x.attr("keyframe_points");
				py::object keyframes_y = fcurve_y.attr("keyframe_points");
				py::object keyframes_z = fcurve_z.attr("keyframe_points");

				// Write points to fcurves
				keyframes_x.attr("clear")();
				keyframes_y.attr("clear")();
				keyframes_z.attr("clear")();
				keyframes_x.attr("add")(track_points.size());
				keyframes_y.attr("add")(track_points.size());
				keyframes_z.attr("add")(track_points.size());

				auto current_frame = track_points.begin();

				for (int i = 0; i < track_points.size(); ++i) {
					py::object key_x = keyframes_x.attr("__getitem__")(i);
					py::object key_y = keyframes_y.attr("__getitem__")(i);
					py::object key_z = keyframes_z.attr("__getitem__")(i);
					key_x.attr("co") = py::make_tuple(current_frame->first, current_frame->second.x);
					key_y.attr("co") = py::make_tuple(current_frame->first, current_frame->second.y);
					key_z.attr("co") = py::make_tuple(current_frame->first, current_frame->second.z);
					++current_frame;
				}

				keyframes_x.attr("deduplicate")();
				keyframes_y.attr("deduplicate")();
				keyframes_z.attr("deduplicate")();
				fcurve_x.attr("update")();
				fcurve_y.attr("update")();
				fcurve_z.attr("update")();

			}

			end = std::chrono::high_resolution_clock::now();

			blend_write += std::chrono::duration_cast<std::chrono::microseconds>(end - start);

			auto total = blend_read + blend_write + init + triangulate;

			std::cout << "[MotionEngine] triangulate_points -- Total: " << total.count() << "us, init: " << init.count() << "us, blend_read: " << blend_read.count() << "us, blend_write: " << blend_write.count() << "us, triangulate: " << triangulate.count() << "us" << std::endl;

	},
		py::arg("clips"),
		py::arg("pose_tracks_list"),
		py::arg("cam_matrices"),
		py::arg("dist_vectors"),
		py::arg("bpy_scene"),
		py::arg("bpy_data"),
		py::arg("use_all_tracks") = false,
		py::arg("non_destructive") = false
	);


	// Core
	m.def("rand_img_rgb", [](std::pair<int, int> size) {
		cv::Mat img(size.first, size.second, CV_8UC3);
		cv::randu(img, cv::Scalar(0, 0, 0), cv::Scalar(255, 255, 255));
		return img;
	});
	m.def("rand_img_gray", [](std::pair<int, int> size) {
		cv::Mat img(size.first, size.second, CV_8UC1);
		cv::randu(img, 0, 255);
		return img;
	});

	// DNN
	m_dnn.def("letterbox_image", [](const cv::Mat& src, cv::Mat& dst, py::tuple out_size) {
		return me::dnn::LetterboxImage(src, dst, cv::Size(out_size[0].cast<int>(), out_size[1].cast<int>()));
	}, py::call_guard<py::gil_scoped_release>(), py::arg("src"), py::arg("dst"), py::arg("out_size"));
	m_dnn.def("fix_detection_coordinates", [](std::vector<me::dnn::Detection>& detections, std::pair<int, int> src_net_size, std::pair<int, int> target_frame_size, me::dnn::ScalingMode scaling_mode) {
		me::dnn::fixDetectionCoordinates(detections, cv::Size(src_net_size.first, src_net_size.second), cv::Size(target_frame_size.first, target_frame_size.second), scaling_mode);
		return detections;
		}, py::call_guard<py::gil_scoped_release>(), py::arg("detections"), py::arg("src_net_size"), py::arg("target_frame_size"), py::arg("scaling_mode") = me::dnn::ScalingMode::NORMALIZE_INPUT);
	m_dnn.def("fix_detection_coordinates", [](std::vector<std::vector<me::dnn::Detection>>& detections, std::pair<int, int> src_net_size, std::pair<int, int> target_frame_size, me::dnn::ScalingMode scaling_mode) {
		me::dnn::fixDetectionCoordinates(detections, cv::Size(src_net_size.first, src_net_size.second), cv::Size(target_frame_size.first, target_frame_size.second), scaling_mode);
		return detections;
		}, py::call_guard<py::gil_scoped_release>(), py::arg("detections"), py::arg("src_net_size"), py::arg("target_frame_size"), py::arg("scaling_mode") = me::dnn::ScalingMode::NORMALIZE_INPUT);
	m_dnn.def("get_roi_with_padding", [](const cv::Mat& image, cv::Rect2d roi) {
		return me::dnn::getRoiWithPadding(image, roi);
	}, py::call_guard<py::gil_scoped_release>());
	m_dnn.def("get_roi_no_padding", [](const cv::Mat& image, cv::Rect2d roi) {
		return me::dnn::getRoiNoPadding(image, roi);
	}, py::call_guard<py::gil_scoped_release>());
	m_dnn.def("is_roi_outside_image", [](std::pair<int, int> imageSize, cv::Rect2d roi) {
		return me::dnn::isRoiOutsideImage(cv::Size(imageSize.first, imageSize.second), roi);
	});
	m_dnn.def("draw_tags", [](cv::Mat& out_image, std::vector<me::dnn::Tag>& tags) {
		me::dnn::drawTags(out_image, tags);
		return out_image;
	}, py::call_guard<py::gil_scoped_release>());
	m_dnn.def("strict_batch_infer", [](size_t batch_size, me::dnn::models::DetectionModel& model, const std::vector<cv::Mat>& images, float conf_thresh, float iou_thresh) {
		std::vector<std::vector<me::dnn::Detection>> detections;
		me::dnn::models::strict_batch_infer(batch_size, model, images, detections, conf_thresh, iou_thresh);
		return detections;
	}, py::call_guard<py::gil_scoped_release>());
	m_dnn.def("strict_batch_infer", [](size_t batch_size, me::dnn::models::PoseModel& model, const std::vector<cv::Mat>& images) {
		std::vector<me::dnn::Pose> poses;
		me::dnn::models::strict_batch_infer(batch_size, model, images, poses);
		return poses;
	}, py::call_guard<py::gil_scoped_release>());
	m_dnn.def("strict_batch_infer", [](size_t batch_size, me::dnn::models::FeatureModel& model, const std::vector<cv::Mat>& images) {
		std::vector<me::dnn::Feature> features;
		me::dnn::models::strict_batch_infer(batch_size, model, images, features);
		return features;
	}, py::call_guard<py::gil_scoped_release>());
	m_dnn.def("strict_batch_infer", [](size_t batch_size, me::dnn::models::TagModel& model, const std::vector<cv::Mat>& images) {
		std::vector<std::vector<me::dnn::Tag>> tags;
		me::dnn::models::strict_batch_infer(batch_size, model, images, tags);
		return tags;
	}, py::call_guard<py::gil_scoped_release>());

	// Crypto
	m_crypto.def("random_sha1", []() { return me::crypto::generateRandomSHA1().to_string(); });

	// Tracking
	m_tracking.def("find_common_data", &me::tracking::find_common_data, py::call_guard<py::gil_scoped_release>());

	m_tracking.def("solve_static_pair", py::overload_cast<const me::tracking::TrackingData&, const me::tracking::TrackingData&,
		const me::tracking::Kk&, const me::tracking::Kk&>(&me::tracking::solveStaticPair), py::call_guard<py::gil_scoped_release>());

	m_tracking.def("solve_static_pair", py::overload_cast<const me::tracking::TrackedPoints&, const me::tracking::TrackedPoints&,
		const me::tracking::Kk&, const me::tracking::Kk&>(&me::tracking::solveStaticPair), py::call_guard<py::gil_scoped_release>());

	m_tracking.def("solve_static_set", &me::tracking::solveStaticSet, py::call_guard<py::gil_scoped_release>());

}