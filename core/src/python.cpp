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
#include "modules/crypto/sha1.hpp"

// DNN module headers
#include "modules/dnn/rtmdet.hpp"
#include "modules/dnn/yolox.hpp"
#include "modules/dnn/rtmpose.hpp"
#include "modules/dnn/feature_extractor.hpp"
#include "modules/dnn/tag_net.hpp"
#include "modules/dnn/cv_tag_detector.hpp"
#include "modules/dnn/pose_topdown.hpp"

// IO module headers
#include "modules/io/transcoder.hpp"
#include "modules/io/imagelist.hpp"

// Tracking module headers
#include "modules/tracking/data.hpp"
#include "modules/tracking/camera.hpp"
#include "modules/tracking/triangulation.hpp"
#include "modules/tracking/filters.hpp"

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

// Blender compatibility headers
#include "modules/blender/bpy_types.hpp"
#include "modules/blender/bpy_utils.hpp"
#include "modules/blender/bpy_data.hpp"
#include "modules/blender/bpy_ops.hpp"

namespace py = pybind11;

template<typename T>
T bpy_wrap(py::object py_obj) {
	return T(reinterpret_cast<void*>(py_obj.attr("as_pointer")().cast<uintptr_t>()));
}

template<typename T>
T* bpy_ptr(py::object py_obj) {
	return (T*)reinterpret_cast<void*>(py_obj.attr("as_pointer")().cast<uintptr_t>());
}

namespace pybind11::detail {

	template <> struct type_caster<PyMovieClip> {
	public:
		PYBIND11_TYPE_CASTER(PyMovieClip, _("MovieClip"));
		bool load(handle src, bool) {
			object py_obj = reinterpret_steal<object>(src);
			object bpy_types = module::import("bpy.types");
			object bpy_clip = bpy_types.attr("MovieClip");
			if (isinstance(py_obj, bpy_clip)) {
				value = PyMovieClip(&py_obj);
				return true;
			}
			return false;
		}
	};

	template <> struct type_caster<PyBOperator> {
	public:
		PYBIND11_TYPE_CASTER(PyBOperator, _("Operator"));
		bool load(handle src, bool) {
			object py_obj = reinterpret_steal<object>(src);
			object bpy_types = module::import("bpy.types");
			object bpy_clip = bpy_types.attr("Operator");
			if (isinstance(py_obj, bpy_clip)) {
				value = PyBOperator(&py_obj);
				return true;
			}
			return false;
		}
	};

}

PYBIND11_MODULE(posetracks_core, m)
{
	// Submodules
	auto m_data = m.def_submodule("data");
	auto m_dnn = m.def_submodule("dnn");
	auto m_io = m.def_submodule("io");
	auto m_tracking = m.def_submodule("tracking");
	auto m_crypto = m.def_submodule("crypto");


	// Enum bindings

	// Base
	py::enum_<BlenderVersion>(m, "BlenderVersion")
		.value("VER_2_93_0", BlenderVersion::VER_2_93_0)
		.value("VER_2_93_4", BlenderVersion::VER_2_93_4)
		.value("VER_3_0_0", BlenderVersion::VER_3_0_0)
		.value("VER_3_1_0", BlenderVersion::VER_3_1_0)
		.value("VER_3_2_0", BlenderVersion::VER_3_2_0)
		.value("VER_3_3_0", BlenderVersion::VER_3_3_0)
		.value("VER_3_4_0", BlenderVersion::VER_3_4_0)
		.value("VER_3_5_0", BlenderVersion::VER_3_5_0)
		.value("VER_3_6_0", BlenderVersion::VER_3_6_0)
		.value("VER_3_6_8", BlenderVersion::VER_3_6_8)
		.value("VER_4_0_0", BlenderVersion::VER_4_0_0)
		.value("VER_4_1_0", BlenderVersion::VER_4_1_0)
		.value("VER_4_1_1", BlenderVersion::VER_4_1_1)
		.value("VER_4_2_0", BlenderVersion::VER_4_2_0)
		.value("VER_4_2_1", BlenderVersion::VER_4_2_1)
		.export_values();

	py::enum_<cv::aruco::PredefinedDictionaryType>(m, "TagDictionary")
		.value("DICT_4X4", cv::aruco::PredefinedDictionaryType::DICT_4X4_1000)
		.value("DICT_5X5", cv::aruco::PredefinedDictionaryType::DICT_5X5_1000)
		.value("DICT_6X6", cv::aruco::PredefinedDictionaryType::DICT_6X6_1000)
		.value("DICT_7X7", cv::aruco::PredefinedDictionaryType::DICT_7X7_1000)
		.value("DICT_ARUCO_ORIGINAL", cv::aruco::PredefinedDictionaryType::DICT_ARUCO_ORIGINAL)
		.value("DICT_APRILTAG_16h5", cv::aruco::PredefinedDictionaryType::DICT_APRILTAG_16h5)
		.value("DICT_APRILTAG_25h9", cv::aruco::PredefinedDictionaryType::DICT_APRILTAG_25h9)
		.value("DICT_APRILTAG_36h10", cv::aruco::PredefinedDictionaryType::DICT_APRILTAG_36h10)
		.value("DICT_APRILTAG_36h11", cv::aruco::PredefinedDictionaryType::DICT_APRILTAG_36h11)
		.export_values();

	// Core

	// DNN
	py::enum_<Precision>(m_dnn, "Precision")
		.value("FLOAT64", Precision::FLOAT64)
		.value("FLOAT32", Precision::FLOAT32)
		.value("FLOAT16", Precision::FLOAT16)
		.value("INT64", Precision::INT64)
		.value("INT32", Precision::INT32)
		.value("INT16", Precision::INT16)
		.value("INT8", Precision::INT8)
		.value("UNKNOWN", Precision::UNKNOWN)
		.value("NONE", Precision::NONE)
		.export_values();

	py::enum_<Executor>(m_dnn, "Executor")
		.value("TENSORRT", Executor::TENSORRT)
		.value("CUDA", Executor::CUDA)
		.value("CPU", Executor::CPU)
		.value("NONE", Executor::NONE)
		.export_values();

	py::enum_<FeatureDistanceType>(m_dnn, "FeatureDistanceType")
		.value("EUCLIDEAN", FeatureDistanceType::EUCLIDEAN)
		.value("NORM_EUCLIDEAN", FeatureDistanceType::NORM_EUCLIDEAN)
		.export_values();

	py::enum_<ScalingMode>(m_dnn, "ScalingMode")
		.value("AUTO", ScalingMode::AUTO)
		.value("NORMALIZE_INPUT", ScalingMode::NORMALIZE_INPUT)
		.value("DIRECT", ScalingMode::DIRECT)
		.export_values();

	// Class bindings

	// Base (used in submodules)
	py::class_<cv::Mat>(m, "Mat")
		.def(py::init<>())
		.def("from_array", [](cv::Mat& self, py::array_t<uint8_t>& im_data) {
			if (im_data.ndim() == 2) {
				cv::Mat mat(static_cast<int>(im_data.shape(0)), static_cast<int>(im_data.shape(1)), CV_8UC1, 
					const_cast<uint8_t*>(im_data.data()), 
					static_cast<int>(im_data.strides(0)));
				self = mat;
			}
			else if (im_data.ndim() == 3) {
				cv::Mat mat(static_cast<int>(im_data.shape(0)), static_cast<int>(im_data.shape(1)), CV_MAKETYPE(CV_8U, 
					static_cast<int>(im_data.shape(2))),
					const_cast<uint8_t*>(im_data.data()), 
					static_cast<int>(im_data.strides(0)));
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
		.def(py::init([](std::pair<double, double> tuple) {
			return new cv::Point2d(tuple.first, tuple.second);
		}))
		.def_readwrite("x", &cv::Point2d::x)
		.def_readwrite("y", &cv::Point2d::y)
		.def("__repr__", [](cv::Point2d& self) {
			std::stringstream ss;
			ss << '(' << self.x << ", " << self.y << ')';
			return ss.str();
		});
		py::implicitly_convertible<py::tuple, cv::Point2d>();
	py::class_<cv::Point2f>(m, "Pointf")
		.def(py::init<>())
		.def(py::init<float, float>())
		.def(py::init([](std::pair<float, float> tuple) {
			return new cv::Point2f(tuple.first, tuple.second);
		}))
		.def_readwrite("x", &cv::Point2f::x)
		.def_readwrite("y", &cv::Point2f::y)
		.def("__repr__", [](cv::Point2f& self) {
			std::stringstream ss;
			ss << '(' << self.x << ", " << self.y << ')';
			return ss.str();
		});
		py::implicitly_convertible<py::tuple, cv::Point2f>();
	py::class_<cv::Point3d>(m, "Point3D")
		.def(py::init<>())
		.def(py::init<double, double, double>())
		.def(py::init([](std::tuple<double, double, double> tuple) {
			return new cv::Point3d(std::get<0>(tuple), std::get<1>(tuple), std::get<2>(tuple));
		}))
		.def_readwrite("x", &cv::Point3d::x)
		.def_readwrite("y", &cv::Point3d::y)
		.def_readwrite("z", &cv::Point3d::z)
		.def("__repr__", [](cv::Point3d& self) {
			std::stringstream ss;
			ss << '(' << self.x << ", " << self.y << ", " << self.z << ')';
			return ss.str();
		});
		py::implicitly_convertible<py::tuple, cv::Point3d>();
	py::class_<cv::Point3f>(m, "Pointf3D")
		.def(py::init<>())
		.def(py::init<float, float, float>())
		.def(py::init([](std::tuple<float, float, float> tuple) {
			return new cv::Point3f(std::get<0>(tuple), std::get<1>(tuple), std::get<2>(tuple));
		}))
		.def_readwrite("x", &cv::Point3f::x)
		.def_readwrite("y", &cv::Point3f::y)
		.def_readwrite("z", &cv::Point3f::z)
		.def("__repr__", [](cv::Point3f& self) {
			std::stringstream ss;
			ss << '(' << self.x << ", " << self.y << ", " << self.z << ')';
			return ss.str();
		});
		py::implicitly_convertible<py::tuple, cv::Point3f>();
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
	py::class_<FrameProvider>(m_io, "FrameProvider")
		.def(py::init<>())
		.def("load", &FrameProvider::load, py::call_guard<py::gil_scoped_release>(), py::arg("path"), py::arg("use_hw_accel") = false)
		.def("next_frame", &FrameProvider::next_frame, py::call_guard<py::gil_scoped_release>(), py::arg("frame"), py::arg("retry_count") = 100)
		.def("grab_frame", &FrameProvider::grab_frame, py::call_guard<py::gil_scoped_release>(), py::arg("frame"), py::arg("frame_id"), py::arg("retry_count") = 100)
		.def("set_frame", &FrameProvider::set_frame, py::call_guard<py::gil_scoped_release>(), py::arg("frame_id"))
		.def("current_frame", &FrameProvider::current_frame)
		.def("frame_count", &FrameProvider::frame_count)
		.def("frame_size", [](FrameProvider& self) {
			cv::Size size = self.frame_size();
			return py::make_tuple(size.width, size.height);
		})
		.def("fps", &FrameProvider::fps)
		.def("close", &FrameProvider::close)
		.def("is_open", &FrameProvider::is_open)
		.def("get_fourcc_str", &FrameProvider::get_fourcc_str)
		.def("get_fourcc", &FrameProvider::get_fourcc);
	py::class_<ImageList, FrameProvider>(m_io, "ImageList")
		.def(py::init<>());
	py::class_<Transcoder, FrameProvider>(m_io, "Transcoder")
		.def(py::init<>());


	// DNN
	py::class_<Detection>(m_dnn, "Detection")
		.def(py::init<>())
		.def(py::init<int, cv::Rect2d, float>(), py::arg("class_id"), py::arg("bbox"), py::arg("score"))
		.def_readwrite("class_id", &Detection::class_id)
		.def_readwrite("bbox", &Detection::bbox)
		.def_readwrite("score", &Detection::score)
		.def("scale_detection", &Detection::scale_detection);
	py::class_<Joint>(m_dnn, "Joint")
		.def(py::init<>())
		.def(py::init<double, double, double>())
		.def(py::init<cv::Point2d, double>())
		.def_readwrite("prob", &Joint::prob)
		.def_readwrite("pt", &Joint::pt);
	py::class_<Pose>(m_dnn, "Pose")
		.def(py::init<>())
		.def("set_joint", (void (Pose::*)(int, Joint&)) & Pose::set_joint)
		.def("set_joint", (void (Pose::*)(int, cv::Point2d&, double)) & Pose::set_joint)
		.def("set_joint", (void (Pose::*)(int, double, double, double)) & Pose::set_joint)
		.def("get_joint", &Pose::get_joint, py::return_value_policy::reference)
		.def("has_joint", &Pose::has_joint)
		.def("get_joint_ids", &Pose::get_joint_ids)
		.def("num_joints", &Pose::num_joints)
		.def("__getitem__", &Pose::operator[], py::return_value_policy::reference);
	py::class_<Feature>(m_dnn, "Feature")
		.def(py::init<>())
		.def(py::init<std::vector<double>&>())
		.def("norm", &Feature::norm, py::call_guard<py::gil_scoped_release>())
		.def("__truediv__", &Feature::operator/, py::call_guard<py::gil_scoped_release>())
		.def("__sub__", &Feature::operator-, py::call_guard<py::gil_scoped_release>())
		.def("__assign__", &Feature::operator=)
		.def("dist", &Feature::dist, py::call_guard<py::gil_scoped_release>(), py::arg("other"), py::arg("d_type") = FeatureDistanceType::NORM_EUCLIDEAN)
		.def("__len__", &Feature::size)
		.def("__getitem__", [](Feature& self, size_t index) {
			return self.data[index];
		})
		.def("__setitem__", [](Feature& self, size_t index, double val) {
			self.data[index] = val;
		})
		.def("__str__", [](Feature& self) {
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
	py::class_<Tag>(m_dnn, "Tag")
		.def(py::init<>())
		.def(py::init<int>())
		.def(py::init<cv::Point2d&, cv::Point2d&, cv::Point2d&, cv::Point2d&>())
		.def(py::init<int, cv::Point2d&, cv::Point2d&, cv::Point2d&, cv::Point2d&>())
		.def(py::init<int, double>())
		.def(py::init<double, cv::Point2d&, cv::Point2d&, cv::Point2d&, cv::Point2d&>())
		.def(py::init<int, double, cv::Point2d&, cv::Point2d&, cv::Point2d&, cv::Point2d&>())
		.def("__getitem__", [](Tag& self, size_t index) {
			if (index >= 4 || index < 0) throw py::index_error();
			return self.corners[index];
			})
		.def("__setitem__", [](Tag& self, size_t index, cv::Point2d& value) {
			if (index >= 4 || index < 0) throw py::index_error();
			self[index] = value;
		})
		.def_readwrite("id", &Tag::id)
		.def_readwrite("conf", &Tag::conf);
	py::class_<FeatureSet>(m_dnn, "FeatureSet")
		.def(py::init<size_t>())
		.def("add", &FeatureSet::add, py::call_guard<py::gil_scoped_release>())
		.def("at", &FeatureSet::at)
		.def("remove", &FeatureSet::remove, py::call_guard<py::gil_scoped_release>())
		.def("mean", &FeatureSet::mean)
		.def("size", &FeatureSet::size)
		.def("length", &FeatureSet::length)
		.def("__getitem__", [](FeatureSet& self, size_t index) {
			return self[index];
		});
	py::class_<FeatureSpace>(m_dnn, "FeatureSapce")
		.def(py::init<size_t>())
		.def("assign", py::overload_cast<Feature&, double, FeatureDistanceType>(&FeatureSpace::assign),
			py::call_guard<py::gil_scoped_release>(),
			py::arg("input"), py::arg("threshold") = 0.4, py::arg("dist_type") = FeatureDistanceType::NORM_EUCLIDEAN)
		.def("assign", py::overload_cast<std::vector<Feature>&, double, FeatureDistanceType, std::vector<int>>(&FeatureSpace::assign),
			py::call_guard<py::gil_scoped_release>(),
			py::arg("input"), py::arg("threshold") = 0.4, py::arg("dist_type") = FeatureDistanceType::NORM_EUCLIDEAN, py::arg("mask") = std::vector<int>())
		.def("size", &FeatureSpace::size)
		.def("length", &FeatureSpace::length)
		.def("clear", &FeatureSpace::clear)
		.def("at", &FeatureSpace::at, py::return_value_policy::reference)
		.def("__getitem__", [](FeatureSpace& self, size_t index) {
			return self[index];
		}, py::return_value_policy::reference);
	py::class_<FeatureTracker>(m_dnn, "FeatureTracker")
		.def(py::init<size_t>())
		.def("assign", &FeatureTracker::assign,
			py::call_guard<py::gil_scoped_release>(),
			py::arg("input_boxes"), py::arg("input_features"), py::arg("score_threshold") = 0.7, py::arg("f_space_threshold") = 0.4,
			py::arg("dist_type") = FeatureDistanceType::NORM_EUCLIDEAN, py::arg("mask") = std::vector<int>());

	
	// DNN Model class bindings

	// Proxy classes
	py::class_<Model>(m_dnn, "Model")
		.def(py::init<>())
		.def("load", &Model::load, py::call_guard<py::gil_scoped_release>(), py::arg("model_path"), py::arg("target_executor") = Executor::CPU)
		.def("unload", &Model::unload)
		.def("is_loaded", &Model::is_loaded)
		.def("get_precision", &Model::get_precision)
		.def("get_executor", &Model::get_executor);
	py::class_<ImageModel, Model>(m_dnn, "ImageModel")
		.def(py::init<>())
		.def("net_size", [](ImageModel& self) {
			cv::Size size = self.net_size();
			return py::make_tuple(size.width, size.height);
		});
	py::class_<DetectionModel, ImageModel>(m_dnn, "DetectionModel")
		.def(py::init<>())
		.def("infer", [](DetectionModel& self, const cv::Mat& image, float conf_thresh, float iou_thresh) {
			std::vector<Detection> result;
			self.infer(image, result, conf_thresh, iou_thresh);
			return result;
		}, py::call_guard<py::gil_scoped_release>(), py::arg("image"), py::arg("conf_thresh") = 0.5, py::arg("iou_thresh") = 0.5)
		.def("infer", [](DetectionModel& self, const std::vector<cv::Mat>& images, float conf_thresh, float iou_thresh) {
			std::vector<std::vector<Detection>> result;
			self.infer(images, result, conf_thresh, iou_thresh);
			return result;
		}, py::call_guard<py::gil_scoped_release>(), py::arg("images"), py::arg("conf_thresh") = 0.5, py::arg("iou_thresh") = 0.5);
	py::class_<PoseModel, ImageModel>(m_dnn, "PoseModel")
		.def(py::init<>())
		.def("infer", [](PoseModel& self, const cv::Mat& image) {
			Pose result;
			self.infer(image, result);
			return result;
		}, py::call_guard<py::gil_scoped_release>())
		.def("infer", [](PoseModel& self, const std::vector<cv::Mat>& images) {
			std::vector<Pose> result;
			self.infer(images, result);
			return result;
		}, py::call_guard<py::gil_scoped_release>());
	py::class_<FeatureModel, ImageModel>(m_dnn, "FeatureModel")
		.def(py::init<>())
		.def("infer", [](FeatureModel& self, const cv::Mat& image) {
			Feature result;
			self.infer(image, result);
			return result;
		}, py::call_guard<py::gil_scoped_release>())
		.def("infer", [](FeatureModel& self, const std::vector<cv::Mat>& images) {
			std::vector<Feature> result;
			self.infer(images, result);
			return result;
		}, py::call_guard<py::gil_scoped_release>());
	py::class_<TagModel, ImageModel>(m_dnn, "TagModel")
		.def(py::init<>())
		.def("infer", [](TagModel& self, const cv::Mat& image) {
			std::vector<Tag> result;
			self.infer(image, result);
			return result;
		}, py::call_guard<py::gil_scoped_release>())
		.def("infer", [](TagModel& self, const std::vector<cv::Mat>& images) {
			std::vector<std::vector<Tag>> result;
			self.infer(images, result);
			return result;
		}, py::call_guard<py::gil_scoped_release>());


	// Driver classes
	py::class_<RTMDetModel, DetectionModel>(m_dnn, "RTMDetModel")
		.def(py::init<>());
	py::class_<YOLOXModel, DetectionModel>(m_dnn, "YOLOXModel")
		.def(py::init<>());
	py::class_<RTMPoseModel, PoseModel>(m_dnn, "RTMPoseModel")
		.def(py::init<>());
	py::class_<GenericFeatureModel, FeatureModel>(m_dnn, "GenericFeatureModel")
		.def(py::init<>());
	py::class_<TagNetModel, TagModel>(m_dnn, "TagNetModel")
		.def(py::init<>());
	py::class_<CVTagDetector, TagModel>(m_dnn, "CVTagDetector")
		.def(py::init<>())
		.def("set_dict_type", &CVTagDetector::set_dict_type)
		.def("set_preprocess_size", [](CVTagDetector& self, std::pair<int, int> new_size) {
			self.set_preprocess_size(cv::Size(new_size.first, new_size.second));
		});

	py::class_<TopDownPoseDetector>(m_dnn, "TopDownPoseDetector")
		.def(py::init<>())
		.def("unload_all", &TopDownPoseDetector::unload_all)
		.def("infer", [](TopDownPoseDetector& self, const cv::Mat& image, int max_pose_batches, float conf_thresh, float iou_thresh) {
		std::vector<Pose> result;
			self.infer(image, result, max_pose_batches, conf_thresh, iou_thresh);
			return result;
		}, py::call_guard<py::gil_scoped_release>(), py::arg("image"), py::arg("max_pose_batches") = 1, py::arg("conf_thresh") = 0.5, py::arg("iou_thresh") = 0.5)
		.def("infer", [](TopDownPoseDetector& self, const std::vector<cv::Mat>& images, int max_detection_batches, int max_pose_batches, float conf_thresh, float iou_thresh) {
			std::vector<std::vector<Pose>> result;
			self.infer(images, result, max_detection_batches, max_pose_batches, conf_thresh, iou_thresh);
			return result;
		}, py::call_guard<py::gil_scoped_release>(), py::arg("images"), py::arg("max_detection_batches") = 1, py::arg("max_pose_batches") = 1, py::arg("conf_thresh") = 0.5, py::arg("iou_thresh") = 0.5)
		.def("is_ready", &TopDownPoseDetector::is_ready)
		.def_readwrite("detection_model", &TopDownPoseDetector::detection_model)
		.def_readwrite("pose_model", &TopDownPoseDetector::pose_model);

	// Tracking
	py::class_<Mat3x1>(m_tracking, "Mat3x1")
		.def(py::init<>())
		.def("__getitem__", [](Mat3x1& self, int row) {
			return self(row);
		})
		.def("__setitem__", [](Mat3x1& self, int row, double val) {
			self(row) = val;
		})
		.def("__str__", [](Mat3x1& self) {
			std::stringstream ss;
			ss << cv::format(self, cv::Formatter::FMT_PYTHON);
			return ss.str();
		});
	py::class_<Mat1x3>(m_tracking, "Mat1x3")
		.def(py::init<>())
		.def("__getitem__", [](Mat1x3& self, int col) {
			return self(col);
		})
		.def("__setitem__", [](Mat1x3& self, int col, double val) {
			self(col) = val;
		})
		.def("__str__", [](Mat1x3& self) {
			std::stringstream ss;
			ss << cv::format(self, cv::Formatter::FMT_PYTHON);
			return ss.str();
		});
	py::class_<Mat3x3>(m_tracking, "Mat3x3")
		.def(py::init<>())
		.def("__getitem__", [](Mat3x3& self, std::pair<int, int> pos) {
			return self(pos.first, pos.second);
		})
		.def("__setitem__", [](Mat3x3& self, std::pair<int, int> pos, double val) {
			self(pos.first, pos.second) = val;
		})
		.def("__str__", [](Mat3x3& self) {
			std::stringstream ss;
			ss << cv::format(self, cv::Formatter::FMT_PYTHON);
			return ss.str();
		});
	py::class_<Mat4x4>(m_tracking, "Mat4x4")
		.def(py::init<>())
		.def("__getitem__", [](Mat4x4& self, std::pair<int, int> pos) {
			return self(pos.first, pos.second);
		})
		.def("__setitem__", [](Mat4x4& self, std::pair<int, int> pos, double val) {
			self(pos.first, pos.second) = val;
		})
		.def("__str__", [](Mat4x4& self) {
			std::stringstream ss;
			ss << cv::format(self, cv::Formatter::FMT_PYTHON);
			return ss.str();
		});
	py::class_<Rt>(m_tracking, "Rt")
		.def(py::init<>())
		.def_readwrite("R", &Rt::R)
		.def_readwrite("t", &Rt::t)
		.def("invert", &Rt::invert)
		.def("to4x4", &Rt::to4x4)
		.def("from4x4", &Rt::from4x4)
		.def("is_identity", &Rt::is_identity);
	py::class_<Kk>(m_tracking, "Kk")
		.def(py::init<>())
		.def_readwrite("K", &Kk::K)
		.def_readwrite("k", &Kk::k);
	py::class_<Tag3D>(m_tracking, "Tag3D")
		.def(py::init<>())
		.def(py::init<int>())
		.def(py::init<cv::Point3d&, cv::Point3d&, cv::Point3d&, cv::Point3d&>())
		.def(py::init<int, cv::Point3d&, cv::Point3d&, cv::Point3d&, cv::Point3d&>())
		.def("__getitem__", [](Tag3D& self, size_t index) {
			if (index >= 4 || index < 0) throw py::index_error();
			return self.corners[index];
			})
		.def("__setitem__", [](Tag3D& self, size_t index, cv::Point3d& value) {
			if (index >= 4 || index < 0) throw py::index_error();
			self[index] = value;
			})
		.def_readwrite("id", &Tag3D::id);
	py::class_<TrackingData>(m_tracking, "TrackingData")
		.def(py::init<>())
		.def_readwrite("poses", &TrackingData::poses)
		.def_readwrite("detections", &TrackingData::detections)
		.def_readwrite("tags", &TrackingData::tags)
		.def("to_points", &TrackingData::to_points, py::call_guard<py::gil_scoped_release>(), py::arg("reduce_tags") = false)
		.def("set_pose", &TrackingData::set_pose)
		.def("set_joint", py::overload_cast<const int, const std::string, const int, const Joint&>(&TrackingData::set_joint))
		.def("set_joint", py::overload_cast<const int, const std::string, const int, const cv::Point2d&, const double>(&TrackingData::set_joint))
		.def("set_joint", py::overload_cast<const int, const std::string, const int, const double, const double, const double>(&TrackingData::set_joint))
		.def("set_detection", &TrackingData::set_detection)
		.def("set_tag", &TrackingData::set_tag);
	py::class_<TrackingData3D>(m_tracking, "TrackingData3D")
		.def(py::init<>())
		.def_readwrite("poses", &TrackingData3D::poses)
		.def_readwrite("detections", &TrackingData3D::detections)
		.def_readwrite("tags", &TrackingData3D::tags);


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


	// Core
	m.def("set_compatibility_mode", &set_compatibility_mode);

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

	m.def("set_pose_sources", &set_pose_sources, py::call_guard<py::gil_scoped_release>());
	m.def("set_tag_sources", &set_tag_sources, py::call_guard<py::gil_scoped_release>());

	// DNN
	m_dnn.def("letterbox_image", [](const cv::Mat& src, cv::Mat& dst, py::tuple out_size) {
		return LetterboxImage(src, dst, cv::Size(out_size[0].cast<int>(), out_size[1].cast<int>()));
	}, py::call_guard<py::gil_scoped_release>(), py::arg("src"), py::arg("dst"), py::arg("out_size"));
	m_dnn.def("fix_detection_coordinates", [](std::vector<Detection>& detections, std::pair<int, int> src_net_size, std::pair<int, int> target_frame_size, ScalingMode scaling_mode) {
		fixDetectionCoordinates(detections, cv::Size(src_net_size.first, src_net_size.second), cv::Size(target_frame_size.first, target_frame_size.second), scaling_mode);
		return detections;
		}, py::call_guard<py::gil_scoped_release>(), py::arg("detections"), py::arg("src_net_size"), py::arg("target_frame_size"), py::arg("scaling_mode") = ScalingMode::NORMALIZE_INPUT);
	m_dnn.def("fix_detection_coordinates", [](std::vector<std::vector<Detection>>& detections, std::pair<int, int> src_net_size, std::pair<int, int> target_frame_size, ScalingMode scaling_mode) {
		fixDetectionCoordinates(detections, cv::Size(src_net_size.first, src_net_size.second), cv::Size(target_frame_size.first, target_frame_size.second), scaling_mode);
		return detections;
		}, py::call_guard<py::gil_scoped_release>(), py::arg("detections"), py::arg("src_net_size"), py::arg("target_frame_size"), py::arg("scaling_mode") = ScalingMode::NORMALIZE_INPUT);
	m_dnn.def("get_roi_with_padding", [](const cv::Mat& image, cv::Rect2d roi) {
		return getRoiWithPadding(image, roi);
	}, py::call_guard<py::gil_scoped_release>());
	m_dnn.def("get_roi_no_padding", [](const cv::Mat& image, cv::Rect2d roi) {
		return getRoiNoPadding(image, roi);
	}, py::call_guard<py::gil_scoped_release>());
	m_dnn.def("is_roi_outside_image", [](std::pair<int, int> imageSize, cv::Rect2d roi) {
		return isRoiOutsideImage(cv::Size(imageSize.first, imageSize.second), roi);
	});
	m_dnn.def("draw_tags", [](cv::Mat& out_image, std::vector<Tag>& tags) {
		drawTags(out_image, tags);
		return out_image;
	}, py::call_guard<py::gil_scoped_release>());
	m_dnn.def("strict_batch_infer", [](size_t batch_size, DetectionModel& model, const std::vector<cv::Mat>& images, float conf_thresh, float iou_thresh) {
		std::vector<std::vector<Detection>> detections;
		strict_batch_infer(batch_size, model, images, detections, conf_thresh, iou_thresh);
		return detections;
	}, py::call_guard<py::gil_scoped_release>());
	m_dnn.def("strict_batch_infer", [](size_t batch_size, PoseModel& model, const std::vector<cv::Mat>& images) {
		std::vector<Pose> poses;
		strict_batch_infer(batch_size, model, images, poses);
		return poses;
	}, py::call_guard<py::gil_scoped_release>());
	m_dnn.def("strict_batch_infer", [](size_t batch_size, FeatureModel& model, const std::vector<cv::Mat>& images) {
		std::vector<Feature> features;
		strict_batch_infer(batch_size, model, images, features);
		return features;
	}, py::call_guard<py::gil_scoped_release>());
	m_dnn.def("strict_batch_infer", [](size_t batch_size, TagModel& model, const std::vector<cv::Mat>& images) {
		std::vector<std::vector<Tag>> tags;
		strict_batch_infer(batch_size, model, images, tags);
		return tags;
	}, py::call_guard<py::gil_scoped_release>());

	// Crypto
	m_crypto.def("random_sha1", []() { return generateRandomSHA1().to_string(); });

	// Tracking
	m_tracking.def("find_common_data", &find_common_data, py::call_guard<py::gil_scoped_release>());

	m_tracking.def("solve_static_pair", py::overload_cast<const TrackingData&, const TrackingData&,
		const Kk&, const Kk&>(&solveStaticPair), py::call_guard<py::gil_scoped_release>());

	m_tracking.def("solve_static_pair", py::overload_cast<const TrackedPoints&, const TrackedPoints&,
		const Kk&, const Kk&>(&solveStaticPair), py::call_guard<py::gil_scoped_release>());

	m_tracking.def("solve_static_set", &solveStaticSet, py::call_guard<py::gil_scoped_release>());

	m_tracking.def("solve_camera_with_tag", &solveCameraWithTag,
		py::arg("observed_tag"), py::arg("cam_Kk"), py::arg("square_length") = 1.0,
		py::call_guard<py::gil_scoped_release>());

	m_tracking.def("triangulate_static", &triangulateStatic, py::call_guard<py::gil_scoped_release>());

	m_tracking.def("g_kernel_1d", &g_kernel_1d,
		py::arg("width") = 3,
		py::call_guard<py::gil_scoped_release>());

	m_tracking.def("mirror_idx", &mirror_idx, py::call_guard<py::gil_scoped_release>());

	m_tracking.def("g_conv_1d", &g_conv_1d,
		py::arg("input"), py::arg("kernel_radius") = 1,
		py::call_guard<py::gil_scoped_release>());


	// Blender functionality

	auto m_blend = m.def_submodule("blender");

	m.def("clip_tracking_data", [](py::object clip, const double joint_conf_thresh, const bool filter_locked, const bool filter_selected) {
		py::object bpy_types = py::module::import("bpy.types");
		py::object bpy_clip = bpy_types.attr("MovieClip");
		if (!isinstance(clip, bpy_clip))
			throw py::type_error("arg0: clip must be of type bpy.types.MovieClip");
		MovieClip blend_clip = bpy_wrap<MovieClip>(clip);
		py::gil_scoped_release release;
		return clip_tracking_data(blend_clip, joint_conf_thresh, filter_locked, filter_selected);
		}, py::arg("clip"), py::arg("joint_conf_thresh") = 0, py::arg("filter_locked") = false, py::arg("filter_selected") = false);

	m_blend.def("OP_FilterTrackGaussian", &OP_FilterTrackGaussian, py::call_guard<py::gil_scoped_release>());

	m_blend.def("OP_FilterFCurvesGaussian", &OP_FilterFCurvesGaussian, py::call_guard<py::gil_scoped_release>());

	m_blend.def("OP_TriangulatePoints", [](py::object calling_op, const std::string anchor_name) {
		OP_TriangulatePoints(PyBOperator(&calling_op), anchor_name);
	}, py::call_guard<py::gil_scoped_release>());

	m_blend.def("OP_SolveCameras_Invoke", &OP_SolveCameras_Invoke, py::call_guard<py::gil_scoped_release>());

	m_blend.def("OP_SolveCameras_Execute", [](py::object calling_op, const std::string anchor, float solution_scale) {
		OP_SolveCameras_Execute(PyBOperator(&calling_op), anchor, solution_scale);
	}, py::call_guard<py::gil_scoped_release>());
}