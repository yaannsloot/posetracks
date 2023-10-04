/*
me_pose_test.cpp
This file is hot garbage
It will be removed eventually

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

#include <me_dnn_pose_model.hpp>
#include <me_dnn_detection_model.hpp>
#include <me_dnn_rtdetection_model.hpp>
#include <me_dnn_detectpose_model.hpp>
#include <me_core_transcoder.hpp>
#include <opencv2/opencv.hpp>
#include <me_core_simplepool.hpp>

void performance_experiments() {
	// Accessor vs true N dimensionsal heap memory structure performance
	int ***testarray = new int**[100];
	for (int j = 0; j < 100; j++) {
		testarray[j] = new int* [100];
		for (int k = 0; k < 100; k++) {
			testarray[j][k] = new int [100];
		}
	}
	int *flatarray = new int[100 * 100 * 100];
	std::array<size_t, 3> flob_dim{ 100, 100, 100 };
	me::dnn::Accessor<3, int> flob(flatarray, flob_dim);
	auto start = std::chrono::high_resolution_clock::now();
	#pragma omp parallel for
	for (int i = 0; i < 100; i++) {
		for (int j = 0; j < 100; j++) {
			for (int k = 0; k < 100; k++) {
				testarray[i][j][k] = 0;
			}
		}
	}
	auto end = std::chrono::high_resolution_clock::now();
	std::cout << "True array zero fill: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << "ms" << std::endl;
	
	start = std::chrono::high_resolution_clock::now();
#pragma omp parallel for
	for (int i = 0; i < 100; i++) {
		for (int j = 0; j < 100; j++) {
			for (int k = 0; k < 100; k++) {
				testarray[i][j][k] = std::rand();
			}
		}
	}
	end = std::chrono::high_resolution_clock::now();
	std::cout << "True array rand fill: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << "ms" << std::endl;
	for (int j = 0; j < 100; j++) {
		for (int k = 0; k < 100; k++) {
			delete[] testarray[j][k];
		}
	}
	for (int j = 0; j < 100; j++) {
		delete[] testarray[j];
	}
	delete[] testarray;
	start = std::chrono::high_resolution_clock::now();
	#pragma omp parallel for
	for (int i = 0; i < 100; i++) {
		for (int j = 0; j < 100; j++) {
			for (int k = 0; k < 100; k++) {
				flob(i, j, k) = 0;
			}
		}
	}
	end = std::chrono::high_resolution_clock::now();
	std::cout << "FLOB zero fill (me::dnn::Accessor): " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << "ms" << std::endl;
	start = std::chrono::high_resolution_clock::now();
	#pragma omp parallel for
	for (int i = 0; i < 100; i++) {
#pragma omp parallel for
		for (int j = 0; j < 100; j++) {
#pragma omp parallel for
			for (int k = 0; k < 100; k++) {
				flob(i, j, k) = std::rand();
			}
		}
	}
	end = std::chrono::high_resolution_clock::now();
	std::cout << "FLOB rand fill (me::dnn::Accessor): " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << "ms" << std::endl;
	start = std::chrono::high_resolution_clock::now();
#pragma omp parallel for
	for (int i = 0; i < 100; i++) {
#pragma omp parallel for
		for (int j = 0; j < 100; j++) {
#pragma omp parallel for
			for (int k = 0; k < 100; k++) {
				flatarray[i * 100 * 100 + j * 100 + k] = 0;
			}
		}
	}
	end = std::chrono::high_resolution_clock::now();
	std::cout << "FLOB zero fill (math based): " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << "ms" << std::endl;
	start = std::chrono::high_resolution_clock::now();
#pragma omp parallel for
	for (int i = 0; i < 100; i++) {
		for (int j = 0; j < 100; j++) {
			for (int k = 0; k < 100; k++) {
				flatarray[i * 100 * 100 + j * 100 + k] = std::rand();
			}
		}
	}
	end = std::chrono::high_resolution_clock::now();
	std::cout << "FLOB rand fill (math based): " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << "ms" << std::endl;
	delete[] flatarray;

	// Generate vector of images
	std::vector<cv::Mat> images;
	for (int i = 0; i < 10; i++) {
		cv::Mat img(1500, 1500, CV_8UC3);
		cv::randu(img, cv::Scalar(0, 0, 0), cv::Scalar(255, 255, 255));
		images.push_back(img);
	}

	std::cout << "Testing OpenCV blob function[1]...";
	start = std::chrono::high_resolution_clock::now();
	cv::dnn::blobFromImages(images, 1, cv::Size(320, 320));
	end = std::chrono::high_resolution_clock::now();
	std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << "ms" << std::endl;

	std::cout << "Testing OpenCV blob function[2]...";
	start = std::chrono::high_resolution_clock::now();
	cv::dnn::blobFromImages(images, 1, cv::Size(320, 320));
	end = std::chrono::high_resolution_clock::now();
	std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << "ms" << std::endl;

	for (int i = 1; i <= 4; ++i) {
		std::cout << "Testing MotionEngine blob function(letterbox)[" << i << "]...";
		std::vector<float> output_float;
		start = std::chrono::high_resolution_clock::now();
		me::dnn::blobifyImages(images, output_float, 1.0, cv::Scalar(), cv::Scalar(), cv::Size(320, 320), false, false, me::dnn::CropMethod::LETTERBOX);
		end = std::chrono::high_resolution_clock::now();
		std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << "ms" << std::endl;
	}

	for (int i = 1; i <= 4; ++i) {
		std::cout << "Testing MotionEngine blob function(fit)[" << i << "]...";
		std::vector<float> output_float;
		start = std::chrono::high_resolution_clock::now();
		me::dnn::blobifyImages(images, output_float, 1.0, cv::Scalar(), cv::Scalar(), cv::Size(320, 320), false, false, me::dnn::CropMethod::FIT);
		end = std::chrono::high_resolution_clock::now();
		std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << "ms" << std::endl;
	}

	for (int i = 1; i <= 4; ++i) {
		std::cout << "Testing MotionEngine blob function(stretch)[" << i << "]...";
		std::vector<float> output_float;
		start = std::chrono::high_resolution_clock::now();
		me::dnn::blobifyImages(images, output_float, 1.0, cv::Scalar(), cv::Scalar(), cv::Size(320, 320), false, false, me::dnn::CropMethod::NONE);
		end = std::chrono::high_resolution_clock::now();
		std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << "ms" << std::endl;
	}

}


//void primary_tests() {
//	auto pose_model = me::dnn::PoseModel();
//	pose_model.load("end2end.onnx", me::dnn::Executor::CPU);
//	std::cout << "Model loaded: " << pose_model.is_loaded() << std::endl;
//	cv::Mat image = cv::imread("human-pose.jpg");
//	me::dnn::Pose pose_init;
//	cv::Mat pimg_init = cv::Mat::zeros(640, 480, CV_8UC3);
//	pose_model.infer(pimg_init, pose_init);
//	me::dnn::Pose pose;
//	pose_model.infer(image, pose);
//	std::cout << "Pose: " << pose.num_joints() << std::endl;
//	// draw pose
//	for (int i = 0; i < pose.num_joints(); i++) {
//		auto& joint = pose[i];
//		// Scale to image size
//		cv::Point2f pt(joint.pt.x * image.cols, joint.pt.y * image.rows);
//		cv::circle(image, pt, 3, cv::Scalar(0, 255, 0), -1);
//	}
//	// show image
//	cv::imshow("Pose", image);
//	// Also test unloading. GPU mem should be freed
//	pose_model.unload();
//	cv::waitKey(0);
//
//
//	// Test detection model
//	me::dnn::DetectionModel detection_model;
//	detection_model.load("yolov4_-1_3_1280_1280_dynamic.onnx", me::dnn::Executor::TENSORRT);
//	std::cout << "Model loaded: " << detection_model.is_loaded() << std::endl;
//	// Warm up model with random junk
//	cv::Mat image_init = cv::Mat::zeros(640, 480, CV_8UC3);
//	std::vector<me::dnn::Detection> detections;
//	detection_model.infer(image_init, detections, 1, 1);
//	cv::Mat image2 = cv::imread("yaann.jpg");
//	// Test speed
//	auto start = std::chrono::high_resolution_clock::now();
//	detection_model.infer(image2, detections, 0.5, 0.6);
//	auto end = std::chrono::high_resolution_clock::now();
//	std::cout << "Inference time: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << "ms" << std::endl;
//	std::cout << "Detections: " << detections.size() << std::endl;
//	// Draw detections
//	for (auto& detection : detections) {
//		cv::rectangle(image2, detection.bbox, cv::Scalar(0, 255, 0), 2);
//	}
//	cv::imshow("Detections", image2);
//	cv::waitKey(0);
//
//	// Run 100 times
//	for (int i = 0; i < 100; i++) {
//		start = std::chrono::high_resolution_clock::now();
//		detection_model.infer(image2, detections, 0.5, 0.6);
//		end = std::chrono::high_resolution_clock::now();
//		std::cout << "Inference time: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << "ms" << std::endl;
//	}
//
//	detection_model.unload();
//
//	/*
//	detection_model.load("yolov4_-1_3_384_640_dynamic.onnx", me::dnn::Executor::CUDA);
//	auto video_reader = cv::VideoCapture("pexels_videos_4698 (1080p).mp4", cv::CAP_FFMPEG);
//	while (video_reader.isOpened()) {
//		bool success = false;
//		cv::Mat frame;
//		success = video_reader.read(frame);
//		for (int i = 0; i < 100; i++) {
//			if (!success)
//				success = video_reader.read(frame);
//			else
//				break;
//		}
//		if (!success) {
//			std::cout << "Failed to read from video" << std::endl;
//			break;
//		}
//		detection_model.infer(frame, detections, 0.5, 0.5);
//		for (auto& det : detections) {
//			if (det.class_id == 0) {
//				auto br = det.bbox.br();
//				auto tl = det.bbox.tl();
//				auto net_size = detection_model.net_size();
//				br.x = br.x / net_size.width;
//				tl.x = tl.x / net_size.width;
//				br.y = br.y / net_size.height;
//				tl.y = tl.y / net_size.height;
//				br.x = br.x * frame.cols;
//				br.y = br.y * frame.rows;
//				tl.x = tl.x * frame.cols;
//				tl.y = tl.y * frame.rows;
//				cv::Rect2d new_bbox(tl, br);
//				cv::rectangle(frame, new_bbox, cv::Scalar(0, 255, 0), 2);
//			}
//		}
//		cv::imshow("Detections2", frame);
//		cv::waitKey(1);
//
//	}
//	video_reader.release();
//	detection_model.unload();
//	*/
//
//	/*
//	auto video_reader2 = cv::VideoCapture("pexels_videos_4698 (1080p).mp4", cv::CAP_FFMPEG);
//	me::dnn::RTDetectionModel rtdetection_model;
//	rtdetection_model.load("rtmdet-m.onnx", me::dnn::Executor::CUDA);
//	while (video_reader2.isOpened()) {
//		bool success = false;
//		cv::Mat frame;
//		success = video_reader2.read(frame);
//		for (int i = 0; i < 100; i++) {
//			if (!success)
//				success = video_reader2.read(frame);
//			else
//				break;
//		}
//		if (!success) {
//			std::cout << "Failed to read from video" << std::endl;
//			break;
//		}
//		rtdetection_model.infer(frame, detections, 0.5, 0.5);
//		for (auto& det : detections) {
//			if (det.class_id == 0) {
//				auto br = det.bbox.br();
//				auto tl = det.bbox.tl();
//				auto net_size = rtdetection_model.net_size();
//				br.x = br.x / net_size.width;
//				tl.x = tl.x / net_size.width;
//				br.y = br.y / net_size.height;
//				tl.y = tl.y / net_size.height;
//				br.x = br.x * frame.cols;
//				br.y = br.y * frame.rows;
//				tl.x = tl.x * frame.cols;
//				tl.y = tl.y * frame.rows;
//				cv::Rect2d new_bbox(tl, br);
//				cv::rectangle(frame, new_bbox, cv::Scalar(0, 255, 0), 2);
//			}
//		}
//		cv::imshow("Detections3", frame);
//		cv::waitKey(1);
//	}
//	rtdetection_model.unload();
//	video_reader2.release();
//	*/
//
//	std::vector<std::pair<int, int>> joint_mappings = {
//		{1, 2}
//	};
//	auto video_reader2 = cv::VideoCapture("right3.mp4", cv::CAP_FFMPEG);
//	me::dnn::DetectPoseModel detectpose_model;
//	detectpose_model.detection_model.load("rtmdet-m.onnx", me::dnn::Executor::CUDA);
//	detectpose_model.pose_model.load("end2end.onnx", me::dnn::Executor::CUDA);
//	std::vector<me::dnn::Pose> poses;
//	while (video_reader2.isOpened()) {
//		bool success = false;
//		cv::Mat frame;
//		success = video_reader2.read(frame);
//		for (int i = 0; i < 100; i++) {
//			if (!success)
//				success = video_reader2.read(frame);
//			else
//				break;
//		}
//		if (!success) {
//			std::cout << "Failed to read from video" << std::endl;
//			break;
//		}
//		detectpose_model.infer(frame, poses, 2);
//		for (auto& pose : poses) {
//			for (int i = 0; i < pose.num_joints(); i++) {
//				auto& joint = pose[i];
//				if (joint.prob > 0.5)
//					cv::circle(frame, joint.pt, 3, cv::Scalar(0, 255, 0), -1);
//			}
//		}
//		cv::imshow("Detections3", frame);
//		cv::waitKey(1);
//	}
//	detectpose_model.unload_all();
//	video_reader2.release();
//}

void detectpose_test() {
	auto video_reader2 = cv::VideoCapture("right3.mp4");
	cv::VideoCapture cap3("right3.mp4");
	cv::Mat mat;
	cap3.read(mat);
	std::cout << video_reader2.get(cv::CAP_PROP_POS_FRAMES) << " " << cap3.get(cv::CAP_PROP_POS_FRAMES) << std::endl;
	me::dnn::DetectPoseModel detectpose_model;
	detectpose_model.detection_model.load("rtmdet-m.onnx", me::dnn::Executor::CUDA);
	detectpose_model.pose_model.load("end2end.onnx", me::dnn::Executor::TENSORRT);
	std::vector<me::dnn::Pose> poses;
	while (video_reader2.isOpened()) {
		bool success = false;
		cv::Mat frame;
		success = video_reader2.read(frame);
		for (int i = 0; i < 100; i++) {
			if (!success)
				success = video_reader2.read(frame);
			else
				break;
		}
		if (!success) {
			std::cout << "Failed to read from video" << std::endl;
			break;
		}
		detectpose_model.infer(frame, poses, 1);
		for (auto& pose : poses) {
			for (int i = 0; i < pose.num_joints(); i++) {
				auto& joint = pose[i];
				if (joint.prob > 0.5)
					cv::circle(frame, joint.pt, 3, cv::Scalar(0, 255, 0), -1);
			}
		}
		cv::imshow("Detections3", frame);
		cv::waitKey(1);
	}
	detectpose_model.unload_all();
	video_reader2.release();
}

int main() {
	try {
		std::vector<std::string> providers = Ort::GetAvailableProviders();
		for (auto& provider : providers) {
			std::cout << provider << std::endl;
		}
		// primary_tests();
		performance_experiments();
		detectpose_test();
		std::cout << "Starting pool..." << std::endl;
		auto pool = me::core::SimplePool();
		pool.Start();
		auto test_job = pool.QueueJob([]() {
			std::cout << "Hello world!" << std::endl;
			return 1;
			}
		);
		std::cout << "Test job returns " << test_job.get() << std::endl;
		std::cout << "Adding tasks..." << std::endl;
		std::vector<std::future<int>> jobs;
		for (int i = 0; i < 100; ++i) {
			auto task = pool.QueueJob([](int id) {
				for (int j = 0; j < 1000000; ++j) {
					std::rand(); // Somewhat long task
				}
				return id;
				}, i);
			jobs.emplace_back(std::move(task));
		}
		for (auto &task : jobs) {
			std::cout << "Task " << task.get() << " has finished" << std::endl;
		}
		pool.Stop();
		auto start = std::chrono::high_resolution_clock::now();
		auto segments_a = me::core::calculateSegments(100000, std::thread::hardware_concurrency());
		auto end = std::chrono::high_resolution_clock::now();
		std::cout << "Seg calc: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << "ms" << std::endl;
		for (auto& seg : segments_a) {
			std::cout << "segment_a: " << seg.first << ", " << seg.second << std::endl;
		}
		auto segments_b = me::core::calculateSegments(5, std::thread::hardware_concurrency());
		for (auto& seg : segments_b) {
			std::cout << "segment_b: " << seg.first << ", " << seg.second << std::endl;
		}
		auto segments_c = me::core::calculateSegments(1, std::thread::hardware_concurrency());
		for (auto& seg : segments_c) {
			std::cout << "segment_c: " << seg.first << ", " << seg.second << std::endl;
		}

	} catch(std::runtime_error &e) {
		std::cout << e.what() << std::endl;
	}

}