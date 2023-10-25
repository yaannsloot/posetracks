/*
me_python.cpp
Pybind11 module definitions

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

#include <me_core.hpp>
#include <me_core_transcoder.hpp>
#include <me_dnn.hpp>
#include <me_dnn_detection_model.hpp>
#include <me_dnn_rtdetection_model.hpp>
#include <me_dnn_pose_model.hpp>
#include <me_dnn_detectpose_model.hpp>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <filesystem>

namespace py = pybind11;

#define PYBIND11_DETAILED_ERROR_MESSAGES

void load_models(
	me::dnn::DetectPoseModel* model,
	const std::string det_model_path,
	me::dnn::Executor det_model_executor,
	const std::string pose_model_path,
	me::dnn::Executor pose_model_executor
) {
	try {
		std::filesystem::path det_path(det_model_path);
		std::filesystem::path pose_path(pose_model_path);
		std::cout << "[MotionEngine] Loading model \"" << det_path.string() << "\"" << std::endl;
		model->detection_model.load(det_model_path, det_model_executor);
		std::cout << "[MotionEngine] Loading model \"" << pose_path.string() << "\"" << std::endl;
		model->pose_model.load(pose_model_path, pose_model_executor);
	}
	catch (const std::exception& ex) {
		std::cout << "An error occurred on asynchronous task while loading models: " << ex.what() << std::endl;
	}
	catch (const std::string& ex) {
		std::cout << "An error occurred on asynchronous task while loading models: " << ex << std::endl;
	}
	catch (...) {
		std::cout << "An unknown error occurred on asynchronous task while loading models" << std::endl;
	}
}

void warmup_models(
	me::dnn::DetectPoseModel* model
) {
	try {
		if (model->is_ready()) {
			cv::Mat wu_det(model->detection_model.net_size(), CV_8UC3);
			cv::Mat wu_pose(model->pose_model.net_size(), CV_8UC3);
			cv::randn(wu_det, cv::Scalar(0, 0, 0), cv::Scalar(255, 255, 255));
			cv::randn(wu_pose, cv::Scalar(0, 0, 0), cv::Scalar(255, 255, 255));
			std::cout << "[MotionEngine] Starting warmup phase of detection model..." << std::endl;
			model->detection_model.infer(wu_det, std::vector<me::dnn::Detection>(), 0, 0);
			std::cout << "[MotionEngine] Starting warmup phase of pose model..." << std::endl;
			model->pose_model.infer(wu_pose, me::dnn::Pose());
			std::cout << "[MotionEngine] Warmup phase complete." << std::endl;
		}
	}
	catch (const std::exception& ex) {
		std::cout << "An error occurred on asyncronous task during model warmup: " << ex.what() << std::endl;
	}
	catch (const std::string& ex) {
		std::cout << "An error occurred on asyncronous task during model warmup: " << ex << std::endl;
	}
	catch (...) {
		std::cout << "An unknown error occurred on asyncronous task during model warmup" << std::endl;
	}
}

void rtm_load_async_func(
	me::dnn::DetectPoseModel* model,
	const std::string det_model_path,
	me::dnn::Executor det_model_executor,
	const std::string pose_model_path,
	me::dnn::Executor pose_model_executor,
	py::function redraw_callback,
	py::function ui_lock_callback,
	py::function display_warmup_callback
) {

	py::gil_scoped_acquire aquire;

	try {

		ui_lock_callback(true);
		display_warmup_callback(true);
		redraw_callback();

		{
			py::gil_scoped_release release;
			load_models(model, det_model_path, det_model_executor, pose_model_path, pose_model_executor);
		}

		redraw_callback();

		{
			py::gil_scoped_release release;
			warmup_models(model);
		}

	}
	catch (const std::exception& ex) {
		std::cout << "An error occurred on asynchronous task: " << ex.what() << std::endl;
	}
	catch (const std::string& ex) {
		std::cout << "An error occurred on asynchronous task: " << ex << std::endl;
	}
	catch (...) {
		std::cout << "An unknown error occurred on asynchronous task" << std::endl;
	}


	// Return state to normal
	ui_lock_callback(false);
	display_warmup_callback(false);
	redraw_callback();

	// Dereference python objects
	ui_lock_callback.dec_ref();
	display_warmup_callback.dec_ref();
	redraw_callback.dec_ref();
	ui_lock_callback.release();
	display_warmup_callback.release();
	redraw_callback.release();

}

void cache_frames(me::core::Transcoder &transcoder, std::vector<cv::Mat>& frames, std::vector<bool>& success, int batch_size) {
	auto futures = transcoder.next_frames(frames, success, batch_size);
	for (auto& future : futures) {
		future.wait();
	}
}

void cache_frames_single_threaded(me::core::Transcoder& transcoder, std::vector<cv::Mat>& frames, std::vector<bool>& success, int batch_size) {

	const int current_frame = transcoder.current_frame();
	const int total_frames = transcoder.frame_count();

	if (current_frame >= total_frames)
		return;

	const int diff = total_frames - current_frame;
	if (diff < batch_size)
		batch_size = diff;

	success.resize(batch_size);
	frames.resize(batch_size);

	for (auto it = success.begin(); it != success.end(); ++it) {
		*it = false;
	}

	for (int f = 0; f < batch_size; ++f) {
		cv::Mat &frame = frames[f];
		bool s = transcoder.next_frame(frame);
		success[f] = s;
		if (!s)
			break;
	}

}

void infer_async_func(
	me::dnn::DetectPoseModel* model, 
	const std::string det_model_path,
	me::dnn::Executor det_model_executor,
	const std::string pose_model_path,
	me::dnn::Executor pose_model_executor,
	int batch_size,
	int det_batch_size,
	int pose_batch_size,
	double conf_thresh,
	double iou_thresh,
	py::object target_clip,
	py::object info_str,
	py::function abspath_func,
	py::function redraw_callback,
	py::function ui_lock_callback,
	py::function display_warmup_callback,
	py::function write_callback
) {

	py::gil_scoped_acquire aquire;

	try {

		ui_lock_callback(true);

		if (!model->is_ready()) {
			display_warmup_callback(true);
			redraw_callback();

			{
				py::gil_scoped_release release;
				load_models(model, det_model_path, det_model_executor, pose_model_path, pose_model_executor);
			}

			redraw_callback();

			{
				py::gil_scoped_release release;
				warmup_models(model);
			}

			display_warmup_callback(false);
			redraw_callback();

		}

		std::string clip_name = py::str(target_clip.attr("name"));
		std::string clip_path = py::str(abspath_func(target_clip.attr("filepath")));
		me::core::Transcoder transcoder;
		transcoder.load(clip_path);

		if (model->is_ready() && transcoder.is_open()) { // This logic is a little weird. Please rearrange it later

			int total_frames = transcoder.frame_count();
			std::vector<std::vector<me::dnn::Pose>> all_poses;

			info_str.attr("msg") = py::str("Detecting poses: 0%");

			while (transcoder.current_frame() < total_frames) {

				int current_frame = 0;
				std::vector<std::vector<me::dnn::Pose>> poses;
				std::vector<cv::Mat> frames;
				std::vector<bool> success;

				{

					py::gil_scoped_release release;

					// Cache frames
					current_frame = transcoder.current_frame();

					std::cout << "[MotionEngine] Reading frames from \"" << clip_name << "\"" << std::endl;

					cache_frames_single_threaded(transcoder, frames, success, batch_size);

					for (const bool& s : success) {
						if (!s)
							throw std::runtime_error("Encountered read failure on file \"" + clip_name + "\"");
					}

					std::cout << "[MotionEngine] Running inference on frames " << current_frame << " to " << current_frame + frames.size() - 1 << std::endl;

					// Run inference
					model->infer(frames, poses, det_batch_size, pose_batch_size, conf_thresh, iou_thresh);

					all_poses.insert(all_poses.end(), poses.begin(), poses.end());

				}

				int percentage = (int)(((double)current_frame + (double)frames.size()) / (double)total_frames * 100);
				std::stringstream ss;
				ss << "Detecting poses: " << percentage << "%";
				info_str.attr("msg") = py::str(ss.str());

			}


			write_callback(all_poses);

		}

	}
	catch (const std::exception& ex) {
		std::cout << "An error occurred on asynchronous task: " << ex.what() << std::endl;
	}
	catch (const std::string& ex) {
		std::cout << "An error occurred on asynchronous task: " << ex << std::endl;
	}
	catch (...) {
		std::cout << "An unknown error occurred on asynchronous task" << std::endl;
	}

	// Return state to normal
	ui_lock_callback(false);
	display_warmup_callback(false);
	redraw_callback();

	// Dereference python objects
	target_clip.dec_ref();
	info_str.dec_ref();
	abspath_func.dec_ref();
	ui_lock_callback.dec_ref();
	display_warmup_callback.dec_ref();
	redraw_callback.dec_ref();
	write_callback.dec_ref();
	target_clip.release();
	info_str.release();
	abspath_func.release();
	ui_lock_callback.release();
	display_warmup_callback.release();
	redraw_callback.release();
	write_callback.release();

}

PYBIND11_MODULE(MEPython, m)
{
	// Submodules
	auto m_core = m.def_submodule("core");
	auto m_dnn = m.def_submodule("dnn");
	auto m_mt = m.def_submodule("mt");


	// Class bindings

	// Base (used in submodules)
	py::class_<cv::Mat>(m, "Mat")
		.def(py::init<>())
		.def("__repr__", [](cv::Mat& self) {
		std::stringstream ss;
		ss << "<cv::Mat: size=(" << self.cols << ',' << self.rows << "), type=" << self.type() << '>';
		return ss.str();});
	py::class_<cv::Point2d>(m, "Point")
		.def(py::init<>())
		.def(py::init<double, double>())
		.def_readwrite("x", &cv::Point2d::x)
		.def_readwrite("y", &cv::Point2d::y);
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
		.def("tl", &cv::Rect2d::br)
		.def_readwrite("x", &cv::Rect2d::x)
		.def_readwrite("y", &cv::Rect2d::y)
		.def_readwrite("width", &cv::Rect2d::width)
		.def_readwrite("height", &cv::Rect2d::height);
	
	// Core
	py::class_<me::core::Transcoder>(m_core, "Transcoder")
		.def(py::init<>())
		.def("load", &me::core::Transcoder::load, py::arg("path"), py::arg("use_hw_accel") = false)
		.def("next_frame", &me::core::Transcoder::next_frame, py::arg("frame"), py::arg("retry_count") = 100)
		.def("grab_frame", &me::core::Transcoder::grab_frame, py::arg("frame"), py::arg("frame_id"), py::arg("retry_count") = 100)
		.def("set_frame", &me::core::Transcoder::set_frame, py::arg("frame_id"))
		.def("current_frame", &me::core::Transcoder::current_frame)
		.def("frame_count", &me::core::Transcoder::frame_count)
		.def("frame_size", [](me::core::Transcoder& self) {
			cv::Size size = self.frame_size();
			return py::make_tuple(size.width, size.height);
		})
		.def("fps", &me::core::Transcoder::fps)
		.def("close", &me::core::Transcoder::close)
		.def("is_open", &me::core::Transcoder::is_open)
		.def("get_fourcc_str", &me::core::Transcoder::get_fourcc_str)
		.def("get_fourcc", &me::core::Transcoder::get_fourcc);


	// DNN
	py::class_<me::dnn::Detection>(m_dnn, "Detection")
		.def(py::init<>())
		.def(py::init<int, cv::Rect2d, float>(), py::arg("class_id"), py::arg("bbox"), py::arg("score"))
		.def_readwrite("class_id", &me::dnn::Detection::class_id)
		.def_readwrite("bbox", &me::dnn::Detection::bbox)
		.def_readwrite("score", &me::dnn::Detection::score);
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
	py::class_<me::dnn::PoseModel>(m_dnn, "RTMPoseModel")
		.def(py::init<>())
		.def("load", &me::dnn::PoseModel::load)
		.def("unload", &me::dnn::PoseModel::unload)
		.def("infer", [](me::dnn::PoseModel& self, const cv::Mat& image) {
			me::dnn::Pose result;
			self.infer(image, result);
			return result;
		})
		.def("infer", [](me::dnn::PoseModel& self, const std::vector<cv::Mat>& images) {
			std::vector<me::dnn::Pose> result;
			self.infer(images, result);
			return result;
		})
		.def("is_loaded", &me::dnn::PoseModel::is_loaded)
		.def("net_size", [](me::dnn::PoseModel& self) {
		cv::Size size = self.net_size();
			return py::make_tuple(size.width, size.height);
		})
		.def("get_precision", &me::dnn::PoseModel::get_precision)
		.def("get_executor", &me::dnn::PoseModel::get_executor);

	/*py::class_<me::dnn::DetectionModel>(m_dnn, "YOLOv4Model") // MUST BE REIMPLEMENTED DUE TO DRIVE CRASH
		.def(py::init<>())
		.def("load", &me::dnn::DetectionModel::load)
		.def("unload", &me::dnn::DetectionModel::unload)
		.def("infer", [](me::dnn::DetectionModel& self, const cv::Mat& image, float conf_thresh = 0.5, float iou_thresh = 0.5) {
			std::vector<me::dnn::Detection> result;
			self.infer(image, result, conf_thresh, iou_thresh);
			return result;
		})
		.def("infer", [](me::dnn::DetectionModel& self, const std::vector<cv::Mat>& images, float conf_thresh = 0.5, float iou_thresh = 0.5) {
			std::vector<std::vector<me::dnn::Detection>> result;
			self.infer(images, result, conf_thresh, iou_thresh);
			return result;
		})
		.def("is_loaded", &me::dnn::DetectionModel::is_loaded)
		.def("net_size", [](me::dnn::DetectionModel& self) {
		cv::Size size = self.net_size();
			return py::make_tuple(size.width, size.height);
		})
		.def("get_precision", &me::dnn::DetectionModel::get_precision)
		.def("get_executor", &me::dnn::DetectionModel::get_executor);*/

	py::class_<me::dnn::RTDetectionModel>(m_dnn, "RTMDetModel")
		.def(py::init<>())
		.def("load", &me::dnn::RTDetectionModel::load)
		.def("unload", &me::dnn::RTDetectionModel::unload)
		.def("infer", [](me::dnn::RTDetectionModel& self, const cv::Mat& image, float conf_thresh = 0.5, float iou_thresh = 0.5) {
			std::vector<me::dnn::Detection> result;
			self.infer(image, result, conf_thresh, iou_thresh);
			return result;
		})
		.def("infer", [](me::dnn::RTDetectionModel& self, const std::vector<cv::Mat>& images, float conf_thresh = 0.5, float iou_thresh = 0.5) {
			std::vector<std::vector<me::dnn::Detection>> result;
			self.infer(images, result, conf_thresh, iou_thresh);
			return result;
		})
		.def("is_loaded", &me::dnn::RTDetectionModel::is_loaded)
		.def("net_size", [](me::dnn::RTDetectionModel& self) {
		cv::Size size = self.net_size();
			return py::make_tuple(size.width, size.height);
		})
		.def("get_precision", &me::dnn::RTDetectionModel::get_precision)
		.def("get_executor", &me::dnn::RTDetectionModel::get_executor);
	py::class_<me::dnn::DetectPoseModel>(m_dnn, "RTMDetPoseBundleModel")
		.def(py::init<>())
		.def("unload_all", &me::dnn::DetectPoseModel::unload_all)
		.def("infer", [](me::dnn::DetectPoseModel& self, const cv::Mat& image, int max_pose_batches = 1, float conf_thresh = 0.5, float iou_thresh = 0.5) {
			std::vector<me::dnn::Pose> result;
			std::cout << "Start" << std::endl;
			self.infer(image, result, max_pose_batches, conf_thresh, iou_thresh);
			std::cout << "End" << std::endl;
			return result;
		})
		.def("infer", [](me::dnn::DetectPoseModel& self, const std::vector<cv::Mat>& images, int max_detection_batches = 1, int max_pose_batches = 1, float conf_thresh = 0.5, float iou_thresh = 0.5) {
			std::vector<std::vector<me::dnn::Pose>> result;
			std::cout << "Start" << std::endl;
			self.infer(images, result, max_detection_batches, max_pose_batches, conf_thresh, iou_thresh);
			std::cout << "End" << std::endl;
			return result;
		})
		.def("is_ready", &me::dnn::DetectPoseModel::is_ready)
		.def_readonly("detection_model", &me::dnn::DetectPoseModel::detection_model)
		.def_readonly("pose_model", &me::dnn::DetectPoseModel::pose_model);


	// Function bindings

	// Base

	// Core

	// DNN
	m_dnn.def("LetterboxImage", [](const cv::Mat& src, cv::Mat& dst, py::tuple out_size) {
		return me::dnn::LetterboxImage(src, dst, cv::Size(out_size[0].cast<int>(), out_size[1].cast<int>()));
	}, py::arg("src"), py::arg("dst"), py::arg("out_size"));

	
	// Enum bindings

	// Base

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


	// Async bindings (MT module)
	py::enum_<std::future_status>(m, "future_status")
		.value("ready", std::future_status::ready)
		.value("timeout", std::future_status::timeout)
		.value("deferred", std::future_status::deferred)
		.export_values();

	py::class_<std::shared_future<std::vector<std::vector<me::dnn::Pose>>>>(m, "future_frame_poses")
		.def(py::init<>())
		.def("get", &std::shared_future<std::vector<std::vector<me::dnn::Pose>>>::get)
		.def("valid", &std::shared_future<std::vector<std::vector<me::dnn::Pose>>>::valid)
		.def("wait", &std::shared_future<std::vector<std::vector<me::dnn::Pose>>>::wait)
		.def("wait_for", [](std::shared_future<std::vector<std::vector<me::dnn::Pose>>>& self, int timeout) {
			return self.wait_for(std::chrono::milliseconds(timeout));
		})
		;

	py::class_<std::shared_future<void>>(m, "future_void")
		.def(py::init<>())
		.def("get", &std::shared_future<void>::get)
		.def("valid", &std::shared_future<void>::valid)
		.def("wait", &std::shared_future<void>::wait)
		.def("wait_for", [](std::shared_future<void>& self, int timeout) {
			return self.wait_for(std::chrono::milliseconds(timeout));
		})
		;

	py::class_<std::shared_future<std::tuple<bool, std::vector<cv::Mat>>>>(m, "future_frames")
		.def(py::init<>())
		.def("get", &std::shared_future<std::tuple<bool, std::vector<cv::Mat>>>::get)
		.def("valid", &std::shared_future<std::tuple<bool, std::vector<cv::Mat>>>::valid)
		.def("wait", &std::shared_future<std::tuple<bool, std::vector<cv::Mat>>>::wait)
		.def("wait_for", [](std::shared_future<std::tuple<bool, std::vector<cv::Mat>>>& self, int timeout) {
		return self.wait_for(std::chrono::milliseconds(timeout));
			})
		;

	// Async functions (MT module)
	m_mt.def("rtm_load_async", [](
		me::dnn::DetectPoseModel& model, 
		const std::string det_model_path, 
		me::dnn::Executor det_model_executor, 
		const std::string pose_model_path, 
		me::dnn::Executor pose_model_executor, 
		py::function redraw_callback,
		py::function ui_lock_callback,
		py::function display_warmup_callback
	) {
			try {
				std::thread thread(rtm_load_async_func,
					&model,
					det_model_path,
					det_model_executor,
					pose_model_path,
					pose_model_executor,
					redraw_callback,
					ui_lock_callback,
					display_warmup_callback
				);
				thread.detach();
			}
			catch (const std::exception& ex) {
				std::cout << "An error occurred while attempting to execute an asynchronous task: " << ex.what() << std::endl;
			}
			catch (const std::string& ex) {
				std::cout << "An error occurred while attempting to execute an asynchronous task: " << ex << std::endl;
			}
			catch (...) {
				std::cout << "An unknown error occurred while attempting to execute an asynchronous task" << std::endl;
			}
	});
	m_mt.def("infer_async", [](
		me::dnn::DetectPoseModel& model,
		const std::string det_model_path,
		me::dnn::Executor det_model_executor,
		const std::string pose_model_path,
		me::dnn::Executor pose_model_executor,
		int batch_size,
		int det_batch_size,
		int pose_batch_size,
		double conf_thresh,
		double iou_thresh,
		py::object target_clip,
		py::object info_str,
		py::function abspath_func,
		py::function redraw_callback,
		py::function ui_lock_callback,
		py::function display_warmup_callback,
		py::function write_callback
		) {
			try {
				std::thread thread(infer_async_func,
					&model,
					det_model_path,
					det_model_executor,
					pose_model_path,
					pose_model_executor,
					batch_size,
					det_batch_size,
					pose_batch_size,
					conf_thresh,
					iou_thresh,
					target_clip,
					info_str,
					abspath_func,
					redraw_callback,
					ui_lock_callback,
					display_warmup_callback,
					write_callback
				);
				thread.detach();
			}
			catch (const std::exception& ex) {
				std::cout << "An error occurred while attempting to execute an asynchronous task: " << ex.what() << std::endl;
			}
			catch (const std::string& ex) {
				std::cout << "An error occurred while attempting to execute an asynchronous task: " << ex << std::endl;
			}
			catch (...) {
				std::cout << "An unknown error occurred while attempting to execute an asynchronous task" << std::endl;
			}
		});
	m_mt.def("RTMDP_infer_async", [](me::dnn::DetectPoseModel& model, const std::vector<cv::Mat>& images, int max_detection_batches = 1, int max_pose_batches = 1, float conf_thresh = 0.5, float iou_thresh = 0.5) {
		if (!me::core::global_pool.Running())
			me::core::global_pool.Start();
		auto future = me::core::global_pool.QueueJob([](me::dnn::DetectPoseModel* model, const std::shared_ptr<std::vector<cv::Mat>> images, int max_detection_batches, int max_pose_batches, float conf_thresh, float iou_thresh) {
			std::vector<std::vector<me::dnn::Pose>> poses;
			if (model->is_ready())
				model->infer(*images, poses, max_detection_batches, max_pose_batches, conf_thresh, iou_thresh);
			return poses;

		}, &model, std::make_shared<std::vector<cv::Mat>>(images), max_detection_batches, max_pose_batches, conf_thresh, iou_thresh);
		return future.share();
	});
	m_mt.def("Transcoder_read_async", [](me::core::Transcoder& transcoder, int num_frames) {
		if (!me::core::global_pool.Running())
			me::core::global_pool.Start();
		auto future = me::core::global_pool.QueueJob([](me::core::Transcoder* transcoder, int num_frames) {
			bool success = true;
			std::vector<cv::Mat> frames;
			for (int i = 0; i < num_frames; i++) {
				cv::Mat frame;
				success = transcoder->next_frame(frame);
				if (!success)
					break;
				frames.push_back(frame);
			}
			return std::make_tuple(success, frames);
		}, &transcoder, num_frames);
		return future.share();
	});


}