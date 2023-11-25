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

#include <me/io/me_core_transcoder.hpp>
#include <me/dnn/rtmdet.hpp>
#include <me/dnn/rtmpose.hpp>
#include <me/dnn/pose_topdown.hpp>
#include <opencv2/sfm/triangulation.hpp>
#include <opencv2/sfm/projection.hpp>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <filesystem>
#include <algorithm>
#include <random>
#include <limits>
#include <numeric>


namespace py = pybind11;

#define PYBIND11_DETAILED_ERROR_MESSAGES

void load_models(
	me::dnn::models::TopDownPoseDetector* model,
	const std::string det_model_path,
	me::dnn::Executor det_model_executor,
	const std::string pose_model_path,
	me::dnn::Executor pose_model_executor
) {
	try {
		std::filesystem::path det_path(det_model_path);
		std::filesystem::path pose_path(pose_model_path);
		model->detection_model(me::dnn::models::RTMDetModel());
		model->pose_model(me::dnn::models::RTMPoseModel());
		std::cout << "[MotionEngine] Loading model \"" << det_path.string() << "\"" << std::endl;
		model->detection_model().load(det_model_path, det_model_executor);
		std::cout << "[MotionEngine] Loading model \"" << pose_path.string() << "\"" << std::endl;
		model->pose_model().load(pose_model_path, pose_model_executor);
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
	me::dnn::models::TopDownPoseDetector* model
) {
	try {
		if (model->is_ready()) {
			cv::Mat wu_det(model->detection_model().net_size(), CV_8UC3);
			cv::Mat wu_pose(model->pose_model().net_size(), CV_8UC3);
			cv::randn(wu_det, cv::Scalar(0, 0, 0), cv::Scalar(255, 255, 255));
			cv::randn(wu_pose, cv::Scalar(0, 0, 0), cv::Scalar(255, 255, 255));
			std::cout << "[MotionEngine] Starting warmup phase of detection model..." << std::endl;
			model->detection_model().infer(wu_det, std::vector<me::dnn::Detection>(), 0, 0);
			std::cout << "[MotionEngine] Starting warmup phase of pose model..." << std::endl;
			model->pose_model().infer(wu_pose, me::dnn::Pose());
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

	py::class_<me::dnn::RTMDetModel>(m_dnn, "RTMDetModel")
		.def(py::init<>())
		.def("load", &me::dnn::RTMDetModel::load)
		.def("unload", &me::dnn::RTMDetModel::unload)
		.def("infer", [](me::dnn::RTMDetModel& self, const cv::Mat& image, float conf_thresh = 0.5, float iou_thresh = 0.5) {
			std::vector<me::dnn::Detection> result;
			self.infer(image, result, conf_thresh, iou_thresh);
			return result;
		})
		.def("infer", [](me::dnn::RTMDetModel& self, const std::vector<cv::Mat>& images, float conf_thresh = 0.5, float iou_thresh = 0.5) {
			std::vector<std::vector<me::dnn::Detection>> result;
			self.infer(images, result, conf_thresh, iou_thresh);
			return result;
		})
		.def("is_loaded", &me::dnn::RTMDetModel::is_loaded)
		.def("net_size", [](me::dnn::RTMDetModel& self) {
		cv::Size size = self.net_size();
			return py::make_tuple(size.width, size.height);
		})
		.def("get_precision", &me::dnn::RTMDetModel::get_precision)
		.def("get_executor", &me::dnn::RTMDetModel::get_executor);
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
	// Needs adjustments future Ian please fix
	m.def("solve_cameras", [](
		py::object clip_a, 
		py::object clip_b, 
		py::object pose_tracks_a, 
		py::object pose_tracks_b, 
		py::object cam_mtx_a, 
		py::object cam_mtx_b, 
		py::object dist_coeffs_a, 
		py::object dist_coeffs_b, 
		const float conf_thresh
		) {

		py::object result = py::none();

		try {

			// OMEGA NOTE!!!
			// This NEEDS to be modified to take into account frame offsets in future versions when that gets implemented
			// It currently DOES NOT DO THAT
			
			// PART 0:
			// Get preliminary information and variables

			// Obtain clip information
			py::list clip_a_size = clip_a.attr("size");
			py::list clip_b_size = clip_b.attr("size");
			int clip_a_width = clip_a_size[0].cast<int>();
			int clip_a_height = clip_a_size[1].cast<int>();
			int clip_b_width = clip_b_size[0].cast<int>();
			int clip_b_height = clip_b_size[1].cast<int>();

			// Obtain pose track information
			py::object tracking_tracks_a = clip_a.attr("tracking").attr("tracks");
			py::object tracking_tracks_b = clip_b.attr("tracking").attr("tracks");
			std::string track_prefix_a = pose_tracks_a.attr("track_prefix").cast<std::string>();
			std::string track_prefix_b = pose_tracks_b.attr("track_prefix").cast<std::string>();
			int track_count_a = pose_tracks_a.attr("tracks").cast<int>();
			int track_count_b = pose_tracks_b.attr("tracks").cast<int>();

			if (track_count_a != track_count_b)
				throw std::runtime_error("Poses must have the same track count");

			// Convert camera matrices and distortion vectors to opencv format
			cv::Mat cv_cam_mtx_a(3, 3, CV_64FC1);
			cv::Mat cv_cam_mtx_b(3, 3, CV_64FC1);
			std::vector<float> cv_dist_coeffs_a;
			std::vector<float> cv_dist_coeffs_b;

			for (int r = 0; r < 3; ++r) {
				for (int c = 0; c < 3; ++c) {
					cv_cam_mtx_a.at<double>(r, c) = cam_mtx_a.attr("__getitem__")(r).attr("__getitem__")(c).cast<float>();
					cv_cam_mtx_b.at<double>(r, c) = cam_mtx_b.attr("__getitem__")(r).attr("__getitem__")(c).cast<float>();
				}
			}

			for (int i = 0; i < py::len(dist_coeffs_a); ++i) {
				cv_dist_coeffs_a.push_back(dist_coeffs_a.attr("__getitem__")(i).cast<float>());
				cv_dist_coeffs_b.push_back(dist_coeffs_b.attr("__getitem__")(i).cast<float>());
			}


			// PART 1:
			// Find matching marker positions based on given threshold value

			std::vector<py::object> common_tracks_a;
			std::vector<py::object> common_tracks_b;

			for (int i = 0; i < track_count_a; ++i) {
				std::stringstream name_a;
				std::stringstream name_b;
				name_a << track_prefix_a << '.' << i;
				name_b << track_prefix_b << '.' << i;
				py::object track_a = tracking_tracks_a.attr("get")(py::str(name_a.str()));
				py::object track_b = tracking_tracks_b.attr("get")(py::str(name_b.str()));
				if (!track_a.is_none() && !track_b.is_none()) {
					std::vector<py::object> track_vector;
					common_tracks_a.push_back(track_a);
					common_tracks_b.push_back(track_b);
				}
			}

			if(common_tracks_a.empty())
				throw std::runtime_error("No common tracks were found for the provided poses");

			std::vector<cv::Point2f> points_a;
			std::vector<cv::Point2f> points_b;

			for (int t = 0; t < common_tracks_a.size(); ++t) {
				py::object markers_a = common_tracks_a[t].attr("markers");
				py::object markers_b = common_tracks_b[t].attr("markers");
				auto marker_count_a = py::len(markers_a);
				auto marker_count_b = py::len(markers_b);
				py::object smallest = markers_a;
				if (marker_count_b < marker_count_a)
					smallest = markers_b;
				for (auto& marker : smallest) {
					py::object marker_a;
					py::object marker_b;

					// Obtaining actual refs for markers, if they exist
					if (marker_count_b < marker_count_a) {
						marker_b = py::cast<py::object>(marker);
						marker_a = markers_a.attr("find_frame")(marker_b.attr("frame"));
					}
					else {
						marker_a = py::cast<py::object>(marker);
						marker_b = markers_b.attr("find_frame")(marker_a.attr("frame"));
					}

					// Skip if there is no match
					if (marker_a.is_none() || marker_b.is_none())
						continue;

					// Convert markers to opencv points and add them to point vectors
					py::object co_a = marker_a.attr("co");
					py::object co_b = marker_b.attr("co");
					py::object pattern_bbox_a = marker_a.attr("pattern_bound_box");
					py::object pattern_bbox_b = marker_b.attr("pattern_bound_box");
					
					float x_a = co_a.attr("__getitem__")(0).cast<float>() * clip_a_width;
					float x_b = co_b.attr("__getitem__")(0).cast<float>() * clip_b_width;
					float y_a = clip_a_height - co_a.attr("__getitem__")(1).cast<float>() * clip_a_height;
					float y_b = clip_b_height - co_b.attr("__getitem__")(1).cast<float>() * clip_b_height;

					cv::Point2f tl_a(
						pattern_bbox_a.attr("__getitem__")(0).attr("__getitem__")(0).cast<float>() * clip_a_width,
						pattern_bbox_a.attr("__getitem__")(0).attr("__getitem__")(1).cast<float>() * clip_a_height
					);
					cv::Point2f tl_b(
						pattern_bbox_b.attr("__getitem__")(0).attr("__getitem__")(0).cast<float>() * clip_b_width,
						pattern_bbox_b.attr("__getitem__")(0).attr("__getitem__")(1).cast<float>() * clip_b_height
					);
					cv::Point2f br_a(
						pattern_bbox_a.attr("__getitem__")(1).attr("__getitem__")(0).cast<float>() * clip_a_width,
						pattern_bbox_a.attr("__getitem__")(1).attr("__getitem__")(1).cast<float>() * clip_a_height
					);
					cv::Point2f br_b(
						pattern_bbox_b.attr("__getitem__")(1).attr("__getitem__")(0).cast<float>() * clip_b_width,
						pattern_bbox_b.attr("__getitem__")(1).attr("__getitem__")(1).cast<float>() * clip_b_height
					);

					double prob_a = std::abs(tl_a.x - br_a.x) * std::abs(tl_a.y - br_a.y) / 100;
					double prob_b = std::abs(tl_b.x - br_b.x) * std::abs(tl_b.y - br_b.y) / 100;

					if (prob_a < conf_thresh || prob_b < conf_thresh)
						continue;

					points_a.push_back(cv::Point2f(x_a, y_a));
					points_b.push_back(cv::Point2f(x_b, y_b));

				}
			}

			// PART 2:
			// Undistort points, calculate essential matrix, and obtain new camera transforms

			std::vector<cv::Point2f> undis_points_a;
			std::vector<cv::Point2f> undis_points_b;

			cv::undistortPoints(points_a, undis_points_a, cv_cam_mtx_a, cv_dist_coeffs_a);
			cv::undistortPoints(points_b, undis_points_b, cv_cam_mtx_b, cv_dist_coeffs_b);

			cv::Mat mask;

			cv::Mat E = cv::findEssentialMat(undis_points_a, undis_points_b, cv::Mat::eye(3, 3, CV_64FC1), cv::RANSAC, 0.999, 0.0001, mask);

			cv::Mat R_relative;
			cv::Mat t_relative;

			int num_inliers = cv::recoverPose(E, undis_points_a, undis_points_b, cv::Mat::eye(3, 3, CV_64FC1), R_relative, t_relative, mask);

			cv::Mat c_transform = cv::Mat::eye(4, 4, CV_64F);
			R_relative.copyTo(c_transform(cv::Rect(0, 0, 3, 3)));
			t_relative.copyTo(c_transform(cv::Rect(3, 0, 1, 3)));

			cv::Mat c_transform_inv;
			cv::invert(c_transform, c_transform_inv, cv::DECOMP_SVD);

			cv::Mat R_inv = c_transform_inv(cv::Rect(0, 0, 3, 3));
			cv::Mat tvec_inv = c_transform_inv(cv::Rect(3, 0, 1, 3));

			cv::Mat transformation_matrix = cv::Mat::eye(4, 4, CV_64FC1);
			R_inv.copyTo(transformation_matrix(cv::Rect(0, 0, 3, 3)));
			tvec_inv.copyTo(transformation_matrix(cv::Rect(3, 0, 1, 3)));

			cv::Mat coordinate_flip = (cv::Mat_<double>(4, 4) <<
				1, -1, -1, 1,
				-1, 1, 1, -1,
				-1, 1, 1, -1,
				1, -1, -1, 1);

			// Converts left handed opencv transform to right handed blender transform
			cv::multiply(transformation_matrix, coordinate_flip, transformation_matrix);

			py::list py_matrix;

			for (int r = 0; r < transformation_matrix.rows; ++r) {
				py::list row;
				for (int c = 0; c < transformation_matrix.cols; ++c) {
					row.append(transformation_matrix.at<double>(r, c));
				}
				py_matrix.append(row);
			}
			
			result = py_matrix;

			return result;

		}
		catch (const std::exception& ex) {
			std::cout << "An error occurred while solving camera positions: " << ex.what() << std::endl;
		}
		catch (const std::string& ex) {
			std::cout << "An error occurred while solving camera positions: " << ex << std::endl;
		}
		catch (...) {
			std::cout << "An unknown error occurred while solving camera positions" << std::endl;
		}

		return result;

	},
		py::arg("clip_a"),
		py::arg("clip_b"),
		py::arg("pose_tracks_a"),
		py::arg("pose_tracks_b"),
		py::arg("cam_mtx_a"),
		py::arg("cam_mtx_b"),
		py::arg("dist_coeffs_a"),
		py::arg("dist_coeffs_b"),
		py::arg("conf_thresh") = 0.9
	);
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

				auto& current_frame = track_points.begin();

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
	m.def("filter_2D", [](
		py::object clip,
		double measurement_noise_cov,
		double process_noise_cov
		) {
			py::object clip_size = clip.attr("size");
			int clip_width = clip_size.attr("__getitem__")(0).cast<int>();
			int clip_height = clip_size.attr("__getitem__")(1).cast<int>();
			py::object clip_tracks = clip.attr("tracking").attr("tracks");
			for (auto& tr : clip_tracks) {
				py::object track = tr.cast<py::object>();

				py::object markers = track.attr("markers");

				if (markers.begin() == markers.end())
					continue;

				// Init filter
				cv::KalmanFilter kalman(4, 2, 0);
				kalman.transitionMatrix = (cv::Mat_<float>(4, 4) <<
					1, 0, 1, 0,
					0, 1, 0, 1,
					0, 0, 1, 0,
					0, 0, 0, 1);
				cv::setIdentity(kalman.measurementMatrix);
				cv::setIdentity(kalman.measurementNoiseCov, cv::Scalar::all(measurement_noise_cov));
				cv::setIdentity(kalman.processNoiseCov, cv::Scalar::all(process_noise_cov));

				py::object marker0 = markers.begin()->cast<py::object>();
				
				py::object co0 = marker0.attr("co");
				float x0 = co0.attr("__getitem__")(0).cast<float>() * clip_width;
				float y0 = clip_height - co0.attr("__getitem__")(1).cast<float>() * clip_height;

				kalman.statePre.at<float>(0) = x0;
				kalman.statePre.at<float>(1) = y0;
				kalman.statePre.at<float>(2) = 0;
				kalman.statePre.at<float>(3) = 0;

				kalman.statePost.at<float>(0) = x0;
				kalman.statePost.at<float>(1) = y0;
				kalman.statePost.at<float>(2) = 0;
				kalman.statePost.at<float>(3) = 0;
				
				for (auto& mkr : markers) {
					py::object marker = mkr.cast<py::object>();

					py::object co = marker.attr("co");
					float x = co.attr("__getitem__")(0).cast<float>() * clip_width;
					float y = clip_height - co.attr("__getitem__")(1).cast<float>() * clip_height;

					cv::Mat prediction = kalman.predict();

					cv::Mat measurement = (cv::Mat_<float>(2, 1) << x, y);
					cv::Mat estimated = kalman.correct(measurement);

					float estimated_x = estimated.at<float>(0);
					float estimated_y = estimated.at<float>(1);

					float bpy_x = estimated_x / clip_width;
					float bpy_y = (clip_height - estimated_y) / clip_height;

					marker.attr("co") = py::make_tuple(bpy_x, bpy_y);
					
				}

			}
		},
		py::arg("clip"),
		py::arg("measurement_noise_cov") = 1e-2,
		py::arg("process_noise_cov") = 1e-4
	);
	m.def("filter_3D", [](
		py::object bpy_obj,
		double measurement_noise_cov,
		double process_noise_cov
		) {
			std::chrono::microseconds blend_read(0);
			std::chrono::microseconds blend_eval(0);
			std::chrono::microseconds blend_write(0);
			std::chrono::microseconds filter_estimations(0);

			auto start = std::chrono::high_resolution_clock::now();

			py::object fcurves = bpy_obj.attr("animation_data").attr("action").attr("fcurves");

			std::map<int, py::object> location_fcurves;

			for (auto& fcrv : fcurves) {
				py::object fcurve = fcrv.cast<py::object>();
				if (fcurve.attr("data_path").contains("location"))
					location_fcurves[fcurve.attr("array_index").cast<int>()] = fcurve;
			}

			std::set<int> target_frames;

			for (auto& fcurve : location_fcurves) {
				py::object keyframe_points = fcurve.second.attr("keyframe_points");
				for (auto& kfrm : keyframe_points) {
					py::object key = kfrm.cast<py::object>();
					target_frames.insert((int)key.attr("co").attr("__getitem__")(0).cast<float>());
				}
			}

			auto end = std::chrono::high_resolution_clock::now();

			blend_read += std::chrono::duration_cast<std::chrono::microseconds>(end - start);

			if (target_frames.empty())
				return;

			cv::KalmanFilter kalman(6, 3, 0);

			kalman.transitionMatrix = (cv::Mat_<float>(6, 6) <<
				1, 0, 0, 1, 0, 0,
				0, 1, 0, 0, 1, 0,
				0, 0, 1, 0, 0, 1,
				0, 0, 0, 1, 0, 0,
				0, 0, 0, 0, 1, 0,
				0, 0, 0, 0, 0, 1);

			start = std::chrono::high_resolution_clock::now();

			cv::setIdentity(kalman.measurementMatrix);
			cv::setIdentity(kalman.measurementNoiseCov, cv::Scalar::all(measurement_noise_cov));
			cv::setIdentity(kalman.processNoiseCov, cv::Scalar::all(process_noise_cov));

			end = std::chrono::high_resolution_clock::now();

			blend_eval += std::chrono::duration_cast<std::chrono::microseconds>(end - start);

			int f0 = *target_frames.begin();

			float x0 = location_fcurves[0].attr("evaluate")(f0).cast<float>();
			float y0 = location_fcurves[1].attr("evaluate")(f0).cast<float>();
			float z0 = location_fcurves[2].attr("evaluate")(f0).cast<float>();

			kalman.statePre.at<float>(0) = x0;
			kalman.statePre.at<float>(1) = y0;
			kalman.statePre.at<float>(2) = z0;
			kalman.statePre.at<float>(3) = 0;
			kalman.statePre.at<float>(4) = 0;
			kalman.statePre.at<float>(5) = 0;

			kalman.statePost.at<float>(0) = x0;
			kalman.statePost.at<float>(1) = y0;
			kalman.statePost.at<float>(2) = z0;
			kalman.statePost.at<float>(3) = 0;
			kalman.statePost.at<float>(4) = 0;
			kalman.statePost.at<float>(5) = 0;

			std::map<int, cv::Point3f> filtered_locations;

			for (int f : target_frames) {

				start = std::chrono::high_resolution_clock::now();

				cv::Mat prediction = kalman.predict();

				end = std::chrono::high_resolution_clock::now();

				filter_estimations += std::chrono::duration_cast<std::chrono::microseconds>(end - start);

				start = std::chrono::high_resolution_clock::now();

				float x = location_fcurves[0].attr("evaluate")(f).cast<float>();
				float y = location_fcurves[1].attr("evaluate")(f).cast<float>();
				float z = location_fcurves[2].attr("evaluate")(f).cast<float>();

				end = std::chrono::high_resolution_clock::now();

				blend_eval += std::chrono::duration_cast<std::chrono::microseconds>(end - start);

				start = std::chrono::high_resolution_clock::now();

				cv::Mat measurement = (cv::Mat_<float>(3, 1) << x, y, z);
				cv::Mat estimated = kalman.correct(measurement);

				end = std::chrono::high_resolution_clock::now();

				filter_estimations += std::chrono::duration_cast<std::chrono::microseconds>(end - start);

				filtered_locations[f] = cv::Point3f(
					estimated.at<float>(0),
					estimated.at<float>(1),
					estimated.at<float>(2)
				);

			}

			start = std::chrono::high_resolution_clock::now();

			for (int i = 0; i < 3; ++i) {
				for (auto& kfrm : location_fcurves[i].attr("keyframe_points")) {
					py::object keyframe = kfrm.cast<py::object>();
					py::object co = keyframe.attr("co");
					int f = (int)co.attr("__getitem__")(0).cast<float>();
					auto& new_loc = filtered_locations[f];
					float val = 0;
					switch (i) {
					case 0:
						val = new_loc.x;
						break;
					case 1:
						val = new_loc.y;
						break;
					default:
						val = new_loc.z;
					}
					co.attr("__setitem__")(1, val);
				}
				location_fcurves[i].attr("update")();
			}

			end = std::chrono::high_resolution_clock::now();

			blend_write += std::chrono::duration_cast<std::chrono::microseconds>(end - start);

			auto total = blend_read + blend_write + blend_eval + filter_estimations;

			std::cout << "[MotionEngine] filter_3D -- Total: " << total.count() << "us, blend_read: " << blend_read.count() << "us, blend_write: " << blend_write.count() << "us, blend_eval: " << blend_eval.count() << "us, filter_estimations: " << filter_estimations.count() << "us" << std::endl;

		},
		py::arg("bpy_obj"),
		py::arg("measurement_noise_cov") = 1e-2,
		py::arg("process_noise_cov") = 1e-4
	);
	m.def("z_thresh_2D", [](
		py::object clip,
		double max_z,
		int max_t_diff
		)
		{
			py::object clip_size = clip.attr("size");
			int clip_width = clip_size.attr("__getitem__")(0).cast<int>();
			int clip_height = clip_size.attr("__getitem__")(1).cast<int>();
			py::object clip_tracks = clip.attr("tracking").attr("tracks");

			max_z = std::abs(max_z);

			max_t_diff = std::max(max_t_diff, 1);

			for (auto& tr : clip_tracks) {
				py::object track = tr.cast<py::object>();

				py::object markers = track.attr("markers");

				if (markers.begin() == markers.end())
					continue;

				std::unordered_map<int, cv::Point2f> points;

				for (auto& mkr : markers) {
					py::object marker = mkr.cast<py::object>();

					int frame = marker.attr("frame").cast<int>();

					py::object co = marker.attr("co");
					float x = co.attr("__getitem__")(0).cast<float>() * clip_width;
					float y = clip_height - co.attr("__getitem__")(1).cast<float>() * clip_height;

					points[frame] = cv::Point2f(x, y);

				}
				
				std::vector<std::unordered_map<int, cv::Point2f>> point_series;
				std::unordered_map<int, cv::Point2f> current_set;

				for (std::unordered_map<int, cv::Point2f>::iterator point = points.begin(); point != points.end(); ++point) {
					current_set.insert(*point);
					if (std::next(point, 1) == points.end() || std::next(point, 1)->first - point->first > max_t_diff) {
						point_series.push_back(current_set);
						current_set = std::unordered_map<int, cv::Point2f>();
					}
				}

				std::vector<double> back_diff;
				std::vector<double> forward_diff;

				cv::Point2f reference_point;
				for (auto& set : point_series) {
					reference_point.x = 0;
					reference_point.y = 0;
					for (auto& point : set) {
						double diff = std::abs(cv::norm(point.second - reference_point));
						reference_point = point.second;
						back_diff.push_back(diff);
					}
				}
				for (auto& set : point_series) {
					for (std::unordered_map<int, cv::Point2f>::iterator point = set.begin(); point != set.end(); ++point) {
						if (std::next(point, 1) != set.end()) {
							double diff = std::abs(cv::norm(std::next(point, 1)->second - point->second));
							forward_diff.push_back(diff);
						}
						else
							forward_diff.push_back(0);
					}
				}

				double back_avg = std::accumulate(back_diff.begin(), back_diff.end(), 0.0) / back_diff.size();
				double forward_avg = std::accumulate(forward_diff.begin(), forward_diff.end(), 0.0) / forward_diff.size();

				double back_sq_sum = std::inner_product(back_diff.begin(), back_diff.end(), back_diff.begin(), 0.0);
				double forward_sq_sum = std::inner_product(forward_diff.begin(), forward_diff.end(), forward_diff.begin(), 0.0);

				double back_std_dev = std::sqrt(back_sq_sum / back_diff.size() - back_avg * back_avg);
				double forward_std_dev = std::sqrt(forward_sq_sum / forward_diff.size() - forward_avg * forward_avg);

				std::set<size_t> first_outliers;

				for (size_t i = 0; i < back_diff.size(); ++i) {
					double z_score = (back_diff[i] - back_avg) / back_std_dev;
					if (std::abs(z_score) > max_z || back_diff[i] == 0) {
						first_outliers.insert(i);
					}
				}
				for (size_t i = 0; i < forward_diff.size(); ++i) {
					double z_score = (forward_diff[i] - forward_avg) / forward_std_dev;
					if (std::abs(z_score) > max_z || forward_diff[i] == 0) {
						first_outliers.insert(i);
					}
				}

				for (auto& f : first_outliers) {
					markers.attr("delete_frame")(std::next(points.begin(), f)->first);
				}

				std::cout << "Filtered " << track.attr("name").cast<std::string>() << std::endl;

			}

		},
		py::arg("clip"),
		py::arg("max_z") = 3,
		py::arg("max_t_diff") = 1
		);


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