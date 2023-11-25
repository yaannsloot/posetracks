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
#pragma once

#include <opencv2/opencv.hpp>

namespace me {
	
	namespace dnn {

		// GENERAL TODO: Make generic class definitions for model types

		enum class Precision { // Status flag for model classes
			FLOAT64,
			FLOAT32,
			FLOAT16,
			INT64,
			INT32,
			INT16,
			INT8,
			UNKNOWN,
			NONE
		};

		enum class Executor { // Status flag for model classes
			TENSORRT,
			CUDA,
			CPU,
			NONE
		};

		enum class BlobLayout {
			NCHW,
			NHWC,
			CHWN
		};

		enum class CropMethod {
			FIT,
			LETTERBOX,
			NONE
		};

		struct Detection {
			Detection();
			Detection(int class_id, cv::Rect2d bbox, float score);
			int class_id;
			cv::Rect2d bbox;
			float score;
		};

		struct Joint {
			Joint();
			Joint(cv::Point2d pt, double prob);
			Joint(double x, double y, double prob);
			double prob;
			cv::Point2d pt;
		};

		struct Pose {
			Pose();
			void set_joint(int id, Joint& joint);
			void set_joint(int id, cv::Point2d& pt, double prob);
			void set_joint(int id, double x, double y, double prob);
			Joint& get_joint(int id);
			bool has_joint(int id);
			std::set<int> get_joint_ids();
			int num_joints();
			Joint& operator[](int id);
			std::map<int, Joint> joints;
		};

		inline double iou(cv::Rect2d a, cv::Rect2d b);

		std::vector<size_t> nms(std::vector<Detection>& detections, float iou_thresh);

		cv::Mat resize_to_net(const cv::Mat& img, cv::Size new_size);

		void blobifyImages(const std::vector<cv::Mat>& images,
			std::vector<float>& output,
			double scale = 1.0,
			const cv::Scalar& mean = cv::Scalar(),
			const cv::Scalar& std_dev = cv::Scalar(),
			const cv::Size& out_size = cv::Size(),
			bool swapRB = false,
			bool use_cuda = false,
			CropMethod crop_method = CropMethod::NONE,
			BlobLayout layout = BlobLayout::NCHW);

		// Data wrapper for operating on flattened blobs
		template <std::size_t N, typename T>
		class Accessor {
		public:
			Accessor(T* data, const std::array<size_t, N>& dim)
				: data(data), dimensions(dim) {
				calculate_strides();
			}

			template <typename... Args>
			inline T& operator()(Args... args) {
				std::array<size_t, N> indices{ (size_t)args... };
				size_t index = 0;

				for (size_t i = 0; i < N; i++)
					index += indices[i] * strides[i];

				return data[index];
			}

		private:
			T* data;
			std::array<size_t, N> dimensions;
			std::array<size_t, N> strides;

			void calculate_strides() {
				size_t stride = 1;
				for (size_t i = N; i-- > 0; ) {
					strides[i] = stride;
					stride *= dimensions[i];
				}
			}
		};

		std::vector<float> LetterboxImage(const cv::Mat& src, cv::Mat& dst, const cv::Size& out_size, bool use_cuda = true);

		std::vector<float> FitImage(const cv::Mat& src, cv::Mat& dst, const cv::Size& out_size, bool use_cuda = true);

		void StretchImage(const cv::Mat& src, cv::Mat& dst, const cv::Size& out_size, bool use_cuda = true);

		bool checkForProvider(const std::string provider_str);

		cv::Mat getRoiWithPadding(const cv::Mat& image, cv::Rect roi);

	}

}