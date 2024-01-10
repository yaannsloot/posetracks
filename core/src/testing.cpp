/*
This file is hot garbage
It will be removed eventually

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

#include <me/dnn/rtmdet.hpp>
#include <me/dnn/rtmpose.hpp>
#include <me/dnn/yolox.hpp>
#include <me/dnn/pose_topdown.hpp>
#include <me/dnn/feature_extractor.hpp>
#include <me/io/imagelist.hpp>
#include <me/io/transcoder.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/sfm/triangulation.hpp>
#include <opencv2/sfm/projection.hpp>
#include <me/threading/simplepool.hpp>
#include <me/data/memory.hpp>
#include <me/crypto/sha1.hpp>
#include <filesystem>
#include <numeric>
#include <random>
#include <arrayfire.h>

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
	me::data::Accessor<3, int> flob(flatarray, flob_dim);
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
	//me::io::FrameProvider cap = me::io::Transcoder();
	//cap.load("right3.mp4");
	me::io::FrameProvider cap = me::io::Transcoder();
	cap.load("shaq.mp4");
	std::cout << cap.frame_size().width << " " << cap.frame_size().height << std::endl;
	me::dnn::models::TopDownPoseDetector detectpose_model;
	detectpose_model.detection_model = me::dnn::models::RTMDetModel();
	detectpose_model.pose_model = me::dnn::models::RTMPoseModel();
	detectpose_model.detection_model.load("redis/models/rtmdet/fullbody_320.onnx", me::dnn::Executor::CUDA);
	detectpose_model.pose_model.load("redis/models/rtmpose/fullbody133-m.onnx", me::dnn::Executor::CUDA);
	std::cout << (int)detectpose_model.detection_model.get_precision() << std::endl;
	std::cout << (int)detectpose_model.pose_model.get_precision() << std::endl;
	std::vector<me::dnn::Pose> poses;

	// Test box detector
	std::vector<cv::Mat> frames(3);
	cap.grab_frame(frames[0], cap.frame_count() * 0.25);
	cap.grab_frame(frames[1], cap.frame_count() * 0.5);
	cap.grab_frame(frames[2], cap.frame_count() * 0.75);
	cap.set_frame(0);
	std::vector<std::vector<me::dnn::Detection>> detections;
	detectpose_model.detection_model.infer(frames, detections, 0.5, 0.5);
	auto net_size = detectpose_model.detection_model.net_size();
	for (int i = 0; i < frames.size(); ++i) {
		auto& dets = detections[i];
		auto& testf = frames[i];
		for (auto& det : dets) {
			cv::rectangle(testf,
				cv::Point(
					det.bbox.tl().x / net_size.width * testf.cols,
					det.bbox.tl().y / net_size.height * testf.rows
				),
				cv::Point(
					det.bbox.br().x / net_size.width * testf.cols,
					det.bbox.br().y / net_size.height * testf.rows
				),
				cv::Scalar(0, 255, 0)
			);
		}
		cv::imshow("Detections3", testf);
		cv::waitKey(0);
	}
	cv::VideoWriter video_out;
	video_out.open("shaq_out.mp4", cv::VideoWriter::fourcc('m', 'p', '4', 'v'), cap.fps(), cap.frame_size());
	while (cap.is_open()) {
		bool success = false;
		cv::Mat frame;
		success = cap.next_frame(frame);
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
		video_out.write(frame);
		cv::imshow("Detections3", frame);
		cv::waitKey(1);
	}
	detectpose_model.unload_all();
	cap.close();
	video_out.release();
}

// Testing class proxy patterns used elsewhere

class A {
public:
	virtual void f() = 0;
};

class B : public A {
public:
	virtual void e() = 0;
};

class C : public B {
public:
	virtual void f() override {
		std::cout << 2 << std::endl;
	}
	virtual void e() override {
		std::cout << 'a' << std::endl;
	}
};

// Stores the actual pointer for the object
class AInstance {
public:
	void f() {
		if (a_instance != nullptr)
			a_instance->f();
		else
			std::cout << "No instance managed" << std::endl;
	}
protected:
	std::shared_ptr<A> a_instance;
};

// Adds a dynamic call to the new virtual function in B. Still no actual instantiation
class BInstance : public AInstance {
public:
	void e() {
		if (a_instance != nullptr) {
			std::shared_ptr<B> b_instance = std::dynamic_pointer_cast<B>(a_instance);
			b_instance->e();
		}
		else
			std::cout << "No instance managed" << std::endl;
	}
};

// Creates an instance because the impl class is not abstract at this level
class CInstance : public BInstance {
public:
	CInstance() {
		a_instance = std::make_shared<C>();
	}
};

template <typename T> // If a vector is already sorted minus the last element, this will rebalance the vector.
void rotate_insert(std::vector<T>& vec) {
	if (vec.size() < 2) return;

	T temp = vec.back();

	size_t i = vec.size() - 1;
	while (i > 0 && vec[i - 1] > temp) {
		vec[i] = vec[i - 1];
		--i;
	}

	vec[i] = temp;
}

// This maintains implicit inheritence rules for type casting.
// Because the managed instance of A cannot be downcasted to B, the dynamic pointer cast should never fail. b_instance will always come from a child type pointer.
// Upcasting works perfectly fine here. THe only thing that gets copied is the instance pointer, which has our actual implementation

int main() {
	try {
		std::cout << me::crypto::generateRandomSHA1().to_string() << std::endl;
		std::cout << me::crypto::generateRandomSHA1().to_string() << std::endl;
		std::cout << me::crypto::generateRandomSHA1().to_string() << std::endl;

		auto start = std::chrono::high_resolution_clock::now();
		auto end = std::chrono::high_resolution_clock::now();

		// Container sorting and RW tests
		std::random_device randy;
		std::mt19937 jerry(randy());
		std::uniform_real_distribution<> steve(0.0, 1.0);
		std::vector<double> random_numbers(1000);
		for (double& val : random_numbers) {
			val = steve(jerry);
		}
		af::array af_rand(random_numbers.size(), random_numbers.data());
		me::dnn::Feature rand_feat(random_numbers);
		std::cout << rand_feat.norm() << ' ' << af::norm(af_rand) << std::endl;

		start = std::chrono::high_resolution_clock::now();
		rand_feat.norm();
		end = std::chrono::high_resolution_clock::now();
		double feat = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
		start = std::chrono::high_resolution_clock::now();
		af::norm(af_rand);
		end = std::chrono::high_resolution_clock::now();
		double fire = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
		std::cout << "ME Feature norm: " << feat << "us, ArrayFire norm: " << fire << "us" << std::endl;

		start = std::chrono::high_resolution_clock::now();
		rand_feat.norm();
		end = std::chrono::high_resolution_clock::now();
		feat = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
		start = std::chrono::high_resolution_clock::now();
		af::norm(af_rand);
		end = std::chrono::high_resolution_clock::now();
		fire = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
		std::cout << "ME Feature norm: " << feat << "us, ArrayFire norm: " << fire << "us" << std::endl;

		start = std::chrono::high_resolution_clock::now();
		rand_feat.norm();
		end = std::chrono::high_resolution_clock::now();
		feat = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
		start = std::chrono::high_resolution_clock::now();
		af::norm(af_rand);
		end = std::chrono::high_resolution_clock::now();
		fire = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
		std::cout << "ME Feature norm: " << feat << "us, ArrayFire norm: " << fire << "us" << std::endl;

		int test_steps = 100;
		std::vector<double> test_vector;
		std::multiset<double> test_set;
		double set_time = 0;
		double vector_time = 0;
		double sort_time = 0;
		double rand_sort_insert = 0;
		double rand_set_insert = 0;
		for (int i = 0; i < test_steps; ++i) {
			test_vector.clear();
			test_set.clear();
			start = std::chrono::high_resolution_clock::now();
			for (double& val : random_numbers) {
				test_vector.push_back(val);
			}
			end = std::chrono::high_resolution_clock::now();
			vector_time += std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
			start = std::chrono::high_resolution_clock::now();
			for (double& val : random_numbers) {
				test_set.insert(val);
			}
			end = std::chrono::high_resolution_clock::now();
			set_time += std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
			start = std::chrono::high_resolution_clock::now();

			std::sort(test_vector.begin(), test_vector.end());

			end = std::chrono::high_resolution_clock::now();
			sort_time += std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
			double new_num = steve(jerry);
			test_vector.push_back(new_num);
			start = std::chrono::high_resolution_clock::now();
			rotate_insert(test_vector);
			end = std::chrono::high_resolution_clock::now();
			rand_sort_insert += std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
			start = std::chrono::high_resolution_clock::now();
			test_set.insert(new_num);
			end = std::chrono::high_resolution_clock::now();
			rand_set_insert += std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
		}
		set_time /= test_steps;
		vector_time /= test_steps;
		sort_time /= test_steps;
		rand_sort_insert /= test_steps;
		rand_set_insert /= test_steps;

		std::cout << "Set_write: " << set_time << "us, Vector write: " << vector_time << "us, Sort time: " << 
			sort_time << "us, Vector total: " << vector_time + sort_time << "us, Resort time: " << rand_sort_insert << "us, Set insert time: "
			<< rand_set_insert << "us" << std::endl;

		me::dnn::Feature feature_a({ 3, 5, 8, 9, 10, 14, 15 });
		me::dnn::Feature feature_b({ 2, 4, 6, 7, 8, 12, 10 });
		me::dnn::Feature feature_c({ 2, 4, 6, 7, 8, 12, 10 });
		std::cout << "www" << std::endl;
		std::cout << feature_a.size() << ", " << feature_b.size() << ", " << feature_c.size() << std::endl;

		me::dnn::FeatureSet feature_set(7);

		feature_set.add(feature_a);
		feature_set.add(feature_b);
		feature_set.add(feature_c);

		std::cout << feature_b.dist(feature_c) << std::endl;

		feature_set.remove(0);
		
		auto feature_mean = feature_set.mean();

		for (auto& d : feature_mean.data)
			std::cout << d << ' ';
		std::cout << std::endl;

		me::dnn::models::FeatureModel feature_extractor = me::dnn::models::GenericFeatureModel();

		feature_extractor.load("BasicConv6_People_64.onnx", me::dnn::Executor::CUDA);

		if(!feature_extractor.is_loaded())
			std::cout << "MODEL NOT LOADED!" << std::endl;

		feature_extractor.infer(cv::imread("1.jpg"), feature_a);

		me::dnn::FeatureSet f_set(feature_a.size());

		std::vector<me::dnn::Feature> features;
		std::vector<cv::Mat> images = { cv::imread("1.jpg"), cv::imread("2.jpg") };

		std::vector<long long> inference_times;
		for (int i = 0; i < 10; ++i) {
			start = std::chrono::high_resolution_clock::now();
			me::dnn::models::strict_batch_infer(1, feature_extractor, images, features);
			end = std::chrono::high_resolution_clock::now();
			inference_times.push_back(std::chrono::duration_cast<std::chrono::microseconds>(end - start).count());
		}
		double avg_time = std::accumulate(inference_times.begin(), inference_times.end(), 0) / inference_times.size();
		
		std::cout << "Inference time: " << avg_time << "us" << std::endl;

		f_set.add(features[0]);
		f_set.add(features[1]);
		
		feature_a.dist(feature_b);

		start = std::chrono::high_resolution_clock::now();
		std::cout << feature_a.size() << std::endl;
		std::cout << feature_a.dist(f_set.mean()) << std::endl;
		end = std::chrono::high_resolution_clock::now();
		std::cout << "Compare time: " << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() << "us" << std::endl;

		std::vector<double> fa(feature_a.size(), 0);
		std::vector<double> fb(feature_a.size(), 1);
		me::dnn::Feature feature_short_a({ me::dnn::Feature(fa).dist(feature_a), me::dnn::Feature(fb).dist(feature_a) });
		me::dnn::Feature feature_short_b({ me::dnn::Feature(fa).dist(feature_b), me::dnn::Feature(fb).dist(feature_b) });
		std::cout << (feature_short_a - feature_short_b).norm() << std::endl;


		// Feature space test
		images.clear();
		std::cout << "Loading images..." << std::endl;
		std::string img_path = "samples";
		std::map<int, std::filesystem::path> target_files;
		for (const auto& entry : std::filesystem::directory_iterator(img_path)) {
			std::string ext = entry.path().extension().string();
			if (ext == ".png" || ext == ".jpg") {
				target_files.emplace(std::stoi(entry.path().stem().string()), entry.path());
			}
		}
		for (auto& path : target_files) {
			images.push_back(cv::imread(path.second.string()));
			std::cout << path.first << ' ';
		}
		std::cout << std::endl;

		std::cout << "Running inference..." << std::endl;
		feature_extractor.infer(images, features);

		std::vector<int> ids;
		me::dnn::FeatureSpace feature_space(features[0].size());

		std::cout << "Estimating ids..." << std::endl;
		bool one_to_one = true;
		if (!one_to_one) {
			for (auto& f : features) {
				auto it = feature_space.assign(f, 0.6);
				ids.push_back(it);
			}
			std::cout << "ids: ";
			for (auto& id : ids) {
				std::cout << id << ' ';
			}
			std::cout << std::endl;
		}
		else {
			double thresh = 0.7;
			auto temp = features;
			features.insert(features.end(), temp.begin(), temp.end());
			for (int i = 0; i < 100; ++i) {
				start = std::chrono::high_resolution_clock::now();
				auto assignments = feature_space.assign(features, thresh);
				end = std::chrono::high_resolution_clock::now();
				std::cout << "Assign time: " << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() << "us" << std::endl;
				for (auto& id : assignments) {
					std::cout << id << ' ';
				}
				std::cout << std::endl;
			}
		}

		bool gen_test = false;

		if (gen_test) {
			std::cout << "Generating test video..." << std::endl;

			int num_elements = 100;
			int num_frames = 4000;
			double frame_rate = 60.0;
			cv::Size frame_size(1024, 1024);
			cv::VideoWriter video_out;
			me::dnn::FeatureSpace f_space(2);
			std::random_device rd;
			std::mt19937 gen(rd());
			std::uniform_int_distribution<int> rand_x(0, frame_size.width);
			std::uniform_int_distribution<int> rand_y(0, frame_size.height);
			std::uniform_int_distribution<int> rand_rgb(0, 255);
			std::vector<cv::Scalar> colors;
			auto f_dist_type = me::dnn::FeatureDistanceType::EUCLIDEAN;
			video_out.open("clustering.mp4", cv::VideoWriter::fourcc('m', 'p', '4', 'v'), frame_rate, frame_size);
			bool init = true;
			bool one_one = false;
			for (int f = 0; f < num_frames; ++f) {
				cv::Mat frame(frame_size, CV_8UC3, cv::Scalar(255, 255, 255));
				std::vector<me::dnn::Feature> frame_points;
				for (int e = 0; e < num_elements; ++e) {
					me::dnn::Feature feature({ (double)rand_x(gen), (double)rand_y(gen) });
					frame_points.push_back(feature);
				}
				if (init || one_one) {
					f_space.assign(frame_points, 2000, f_dist_type);
					init = false;
				}
				else {
					for (auto& rf : frame_points) {
						f_space.assign(rf, 2000, f_dist_type);
					}
				}
				int colors_to_add = f_space.size() - colors.size();
				for (int i = 0; i < colors_to_add; ++i) {
					colors.emplace_back(rand_rgb(gen), rand_rgb(gen), rand_rgb(gen));
				}
				for (int s = 0; s < f_space.size(); ++s) {
					for (auto& feat : f_space[s]) {
						cv::circle(frame, cv::Point(feat.data[0], feat.data[1]), 3, colors[s], -1);
					}
				}
				for (auto& s : f_space) {
					me::dnn::Feature center = s.mean();
					cv::drawMarker(frame, cv::Point(center.data[0], center.data[1]), cv::Scalar(0, 0, 0));
				}

				video_out.write(frame);
			}
			video_out.release();
		}


		// Run functions used in the python module so their dependencies show up on the logs
		std::vector<float> dist_coeffs = { 1, 0, 0, 1, 1 };
		std::vector<cv::Point2f> points = { cv::Point2f(0.1, 0.1) };
		cv::undistortPoints(points, points, cv::Mat::eye(3, 3, CV_64FC1), dist_coeffs);
		cv::Mat E = cv::findEssentialMat(points, points, cv::Mat::eye(3, 3, CV_64FC1), cv::RANSAC, 0.999, 0.0001);
		cv::Mat P;
		cv::sfm::projectionFromKRt(cv::Mat::eye(3, 3, CV_64FC1), cv::Mat::eye(3, 3, CV_64FC1), cv::Mat::ones(3, 1, CV_64FC1), P);
		cv::Mat pointsMat(points);
		pointsMat = pointsMat.reshape(1, 2); // reshape to a 2xN matrix
		std::vector<cv::Mat> points2d = { pointsMat, pointsMat };
		std::vector<cv::Mat> proj_matrices = { P, P };
		cv::Mat points3d;
		cv::sfm::triangulatePoints(points2d, proj_matrices, points3d);
		cv::KalmanFilter kalman(4, 2, 0);
		
		std::cout << "Optimized: " << cv::useOptimized() << std::endl;

		AInstance a;

		BInstance b;

		CInstance c;

		a.f();

		a = b;

		a.f();

		a = c;

		a.f();

		b = c;

		b.e();

		//cv::Mat test_img = cv::imread("left3jpg/0001.jpg");

		//auto new_img = me::dnn::resize_to_net(test_img, cv::Size(1920, 1080));

		//cv::imshow("resized", new_img);

		std::shared_ptr<me::threading::SimplePool> poolp;
		poolp.reset();
		poolp = std::make_shared<me::threading::SimplePool>();
		poolp->Start();
		poolp = std::make_shared<me::threading::SimplePool>();
		poolp->Start();
		poolp.reset();

		std::cout << std::filesystem::temp_directory_path().string() << std::endl;
		std::vector<std::string> providers = Ort::GetAvailableProviders();
		for (auto& provider : providers) {
			std::cout << provider << std::endl;
		}
		// primary_tests();
		performance_experiments();
		detectpose_test();
		std::cout << "Starting pool..." << std::endl;
		auto pool = me::threading::SimplePool();
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
		start = std::chrono::high_resolution_clock::now();
		auto segments_a = me::threading::calculateSegments(100000, std::thread::hardware_concurrency());
		end = std::chrono::high_resolution_clock::now();
		std::cout << "Seg calc: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << "ms" << std::endl;
		for (auto& seg : segments_a) {
			std::cout << "segment_a: " << seg.first << ", " << seg.second << std::endl;
		}
		auto segments_b = me::threading::calculateSegments(5, std::thread::hardware_concurrency());
		for (auto& seg : segments_b) {
			std::cout << "segment_b: " << seg.first << ", " << seg.second << std::endl;
		}
		auto segments_c = me::threading::calculateSegments(1, std::thread::hardware_concurrency());
		for (auto& seg : segments_c) {
			std::cout << "segment_c: " << seg.first << ", " << seg.second << std::endl;
		}

	} catch(std::runtime_error &e) {
		std::cout << e.what() << std::endl;
	}

}