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

#include "dnn.hpp"
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudawarping.hpp>
#include <opencv2/cudaarithm.hpp>
#include <me/threading/simplepool.hpp>
#include <onnxruntime_cxx_api.h>

namespace me {

	namespace dnn {

		Detection::Detection() {
			this->class_id = 0;
			this->bbox = cv::Rect2d(0, 0, 0, 0);
			this->score = 0;
		}

		Detection::Detection(int class_id, cv::Rect2d bbox, float score) {
			this->class_id = class_id;
			this->bbox = bbox;
			this->score = score;
		}

		Joint::Joint() {
			this->pt = cv::Point2d(0, 0);
			this->prob = 0;
		}

		Joint::Joint(cv::Point2d pt, double prob) {
			this->pt = pt;
			this->prob = prob;
		}

		Joint::Joint(double x, double y, double prob) {
			this->pt = cv::Point2d(x, y);
			this->prob = prob;
		}

		Pose::Pose() {}

		void Pose::set_joint(int id, Joint& joint) {
			this->joints[id] = joint;
		}

		void Pose::set_joint(int id, cv::Point2d& pt, double prob) {
			this->joints[id] = Joint(pt, prob);
		}

		void Pose::set_joint(int id, double x, double y, double prob) {
			this->joints[id] = Joint(x, y, prob);
		}

		Joint& Pose::get_joint(int id) {
			return this->joints[id];
		}

		bool Pose::has_joint(int id) {
			return this->joints.find(id) != this->joints.end();
		}

		std::set<int> Pose::get_joint_ids() {
			std::set<int> ids;
			for (auto& joint : this->joints) {
				ids.insert(joint.first);
			}
			return ids;
		}

		int Pose::num_joints() {
			return (int)this->joints.size();
		}

		Joint& Pose::operator[](int id) {
			return this->joints[id];
		}

		inline double iou(cv::Rect2d a, cv::Rect2d b) {
			auto interArea = (a & b).area();
			auto unionArea = a.area() + b.area() - interArea;
			return static_cast<double>(interArea) / static_cast<double>(unionArea);
		}

		// Return indices of detections that are not suppressed by NMS
		std::vector<size_t> nms(std::vector<Detection>& detections, float iou_thresh) {
			std::vector<size_t> selected_indices;

			// Sort detections by score in descending order
			auto comp = [](const Detection& first, const Detection& second) {
				return first.score > second.score;
			};
			std::sort(detections.begin(), detections.end(), comp);

			for (size_t i = 0; i < detections.size(); ++i) {
				bool keep = true;
				for (size_t idx : selected_indices) {
					if (iou(detections[i].bbox, detections[idx].bbox) > iou_thresh) {
						keep = false;
						break;
					}
				}
				if (keep)
					selected_indices.push_back(i);
			}

			return selected_indices;
		}

		cv::Mat resize_segment(const cv::Mat& img_segment, cv::Size new_segment_size) {
			cv::Mat resized_segment;
			cv::resize(img_segment, resized_segment, new_segment_size);
			return resized_segment;
		}

		std::vector<cv::Mat> split_into_segments(const cv::Mat& img, int num_segments) {
			std::vector<cv::Mat> segments;
			int segment_size = std::sqrt(img.rows * img.cols / num_segments);
			int padding = segment_size * num_segments - img.rows * img.cols;
			cv::Mat padded_img;
			cv::copyMakeBorder(img, padded_img, 0, padding / img.cols, 0, padding % img.cols, cv::BORDER_CONSTANT, cv::Scalar(0));
			for (int i = 0; i < padded_img.rows; i += segment_size) {
				for (int j = 0; j < padded_img.cols; j += segment_size) {
					cv::Mat segment = padded_img(cv::Rect(j, i, segment_size, segment_size));
					segments.push_back(segment);
				}
			}
			return segments;
		}

		cv::Mat combine_segments(const std::vector<cv::Mat>& segments, cv::Size new_size) {
			cv::Mat img(new_size, CV_8UC3);
			for (const auto& segment : segments) {
				cv::Mat resized_segment;
				cv::resize(segment, resized_segment, new_size);
				cv::Rect roi(segment.cols * (&segment - &segments[0]), segment.rows * (&segment - &segments[0]), segment.cols, segment.rows);
				resized_segment.copyTo(img(roi));
			}
			return img;
		}

		cv::Mat resize_to_net(const cv::Mat& img, cv::Size new_size) {
			if (!threading::global_pool.Running())
				threading::global_pool.Start();
			int num_segments = threading::global_pool.NumThreads();
			std::vector<cv::Mat> segments = split_into_segments(img, num_segments);
			std::vector<cv::Mat> resized_segments;
			std::vector<std::shared_future<cv::Mat>> futures;

			for (const auto& segment : segments) {
				futures.push_back(threading::global_pool.QueueJob(resize_segment, segment, new_size));
			}

			for (auto& future : futures) {
				resized_segments.push_back(future.get());
			}

			return combine_segments(resized_segments, new_size);
		}

		void blobifyImages(const std::vector<cv::Mat>& images,
			std::vector<float>& output,
			double scale,
			const cv::Scalar& mean,
			const cv::Scalar& std_dev,
			const cv::Size& out_size,
			bool swapRB,
			bool use_cuda,
			CropMethod crop_method,
			BlobLayout layout) {
			// Guards
			double std_dev_a = (std_dev[0] == 0) ? 1.0 : std_dev[0];
			double std_dev_b = (std_dev[1] == 0) ? 1.0 : std_dev[1];
			double std_dev_c = (std_dev[2] == 0) ? 1.0 : std_dev[2];
			const cv::Scalar new_std(std_dev_a, std_dev_b, std_dev_c); // Prevents divide by 0
			// Setup
			size_t N = images.size();
			size_t C = 3;
			size_t H = out_size.height;
			size_t W = out_size.width;
			size_t stride_a;
			size_t stride_b;
			size_t stride_c;
			switch (layout) {
			case BlobLayout::NCHW:
				stride_a = C * H * W;
				stride_b = H * W;
				stride_c = W;
				break;
			case BlobLayout::NHWC:
				stride_a = H * W * C;
				stride_b = W * C;
				stride_c = C;
				break;
			case BlobLayout::CHWN:
				stride_a = H * W * N;
				stride_b = W * N;
				stride_c = N;
				break;
			default:
				std::stringstream ss;
				ss << "Unknown blob layout: " << (int)layout;
				throw std::runtime_error(ss.str());
			}

			size_t blob_size = N * C * H * W;

			output.resize(blob_size);
			float* output_ptr = output.data();

			auto pool_size = std::thread::hardware_concurrency();
			if (!me::threading::global_pool.Running())
				me::threading::global_pool.Start();

			auto resize_segments = me::threading::calculateSegments(N, pool_size);
			std::vector<cv::Mat> resized_images;
			std::vector<std::future<void>> resize_tasks;
			resized_images.resize(N);

			// Queue segment tasks
			for (auto& seg : resize_segments) {
				auto task = me::threading::global_pool.QueueJob([](const std::pair<size_t, size_t> segment,
					const std::vector<cv::Mat> *src_images,
					std::vector<cv::Mat> *dst_images,
					CropMethod method,
					const cv::Size dst_size,
					bool cuda) {

					for (size_t i = segment.first; i < segment.second; ++i) {
						
						// Preprocess image ops
						switch (method) {
						case CropMethod::FIT:
							FitImage(src_images->at(i), dst_images->at(i), dst_size, cuda);
							break;
						case CropMethod::LETTERBOX:
							LetterboxImage(src_images->at(i), dst_images->at(i), dst_size, cuda);
							break;
						default:
							StretchImage(src_images->at(i), dst_images->at(i), dst_size, cuda);
						}

					}

				}, seg, &images, &resized_images, crop_method, out_size, use_cuda);
				resize_tasks.emplace_back(std::move(task));
			}

			// Wait for tasks to complete
			for (auto& task : resize_tasks) {
				task.wait();
			}

			// Copy tasks
			auto copy_segments = me::threading::calculateSegments(blob_size, pool_size);
			std::vector<std::future<void>> copy_tasks;

			for (auto& seg : copy_segments) {
				auto task = me::threading::global_pool.QueueJob([](const std::pair<size_t, size_t> segment,
					float *blob,
					const std::vector<cv::Mat> *src_images,
					BlobLayout l_type,
					bool swapRB,
					size_t N,
					size_t C,
					size_t H,
					size_t W,
					double scale,
					const cv::Scalar mean,
					const cv::Scalar new_std) {

						for (size_t i = segment.first; i < segment.second; ++i) {

							// Calculate indices
							size_t n, h, w, c;
							switch (l_type) {
							case BlobLayout::NCHW:
								n = i / (C * H * W);
								c = (i / (H * W)) % C;
								h = (i / W) % H;
								w = i % W;
								break;
							case BlobLayout::NHWC:
								n = i / (H * W * C);
								h = (i / (W * C)) % H;
								w = (i / C) % W;
								c = i % C;
								break;
							default:
								c = i / (H * W * N);
								h = (i / (W * N)) % H;
								w = (i / N) % W;
								n = i % N;
							}

							// Channel swap
							if (swapRB) {
								if (c == 0)
									c = 2;
								else if (c == 2)
									c = 0;
							}

							float tmp = src_images->at(n).ptr<uchar>(h)[w * 3 + c];

							blob[i] = scale * ((tmp - mean[c]) / new_std[c]);

						}

				}, seg, output_ptr, &resized_images, layout, swapRB, N, C, H, W, scale, mean, new_std);
				copy_tasks.emplace_back(std::move(task));
			}

			// Wait for tasks to complete
			for (auto& task : copy_tasks) {
				task.wait();
			}

		}

		std::vector<float> LetterboxImage(const cv::Mat& src, cv::Mat& dst, const cv::Size& out_size, bool use_cuda) {
			auto in_h = static_cast<float>(src.rows);
			auto in_w = static_cast<float>(src.cols);
			float out_h = (float)out_size.height;
			float out_w = (float)out_size.width;

			float scale = std::min(out_w / in_w, out_h / in_h);

			int mid_h = static_cast<int>(in_h * scale);
			int mid_w = static_cast<int>(in_w * scale);

			int top = (static_cast<int>(out_h) - mid_h) / 2;
			int down = (static_cast<int>(out_h) - mid_h + 1) / 2;
			int left = (static_cast<int>(out_w) - mid_w) / 2;
			int right = (static_cast<int>(out_w) - mid_w + 1) / 2;

			if (cv::cuda::getCudaEnabledDeviceCount() > 0 && use_cuda) {
				cv::cuda::GpuMat gpuIn;
				cv::cuda::GpuMat gpuOut;

				gpuIn.upload(src);

				cv::cuda::resize(gpuIn, gpuOut, cv::Size(mid_w, mid_h));

				cv::cuda::copyMakeBorder(gpuOut, gpuOut, top, down, left, right, cv::BORDER_CONSTANT, cv::Scalar(114, 114, 114));

				gpuOut.download(dst);
			}
			else {
				cv::resize(src, dst, cv::Size(mid_w, mid_h));
				cv::copyMakeBorder(dst, dst, top, down, left, right, cv::BORDER_CONSTANT, cv::Scalar(114, 114, 114));
			}

			std::vector<float> pad_info{ static_cast<float>(left), static_cast<float>(top), scale };
			return pad_info;
		}

		std::vector<float> FitImage(const cv::Mat& src, cv::Mat& dst, const cv::Size& out_size, bool use_cuda) {
			auto in_h = static_cast<float>(src.rows);
			auto in_w = static_cast<float>(src.cols);
			float out_h = (float)out_size.height;
			float out_w = (float)out_size.width;

			// Calculate scale to fill the output size
			float scale = std::max(out_w / in_w, out_h / in_h);

			int mid_h = static_cast<int>(in_h * scale);
			int mid_w = static_cast<int>(in_w * scale);

			// Calculate the region to crop
			int top = (mid_h - static_cast<int>(out_h)) / 2;
			int left = (mid_w - static_cast<int>(out_w)) / 2;

			if (cv::cuda::getCudaEnabledDeviceCount() > 0 && use_cuda) {
				cv::cuda::GpuMat gpuIn;
				cv::cuda::GpuMat gpuOut;

				gpuIn.upload(src);

				cv::cuda::resize(gpuIn, gpuOut, cv::Size(mid_w, mid_h));

				// Crop the image to the output size
				cv::cuda::GpuMat gpuCrop(gpuOut, cv::Rect(left, top, out_w, out_h));
				gpuCrop.download(dst);
			}
			else {
				cv::resize(src, dst, cv::Size(mid_w, mid_h));
				// Crop the image to the output size
				dst = dst(cv::Rect(left, top, out_w, out_h));
			}

			std::vector<float> pad_info{ static_cast<float>(left), static_cast<float>(top), scale };
			return pad_info;
		}

		void StretchImage(const cv::Mat& src, cv::Mat& dst, const cv::Size& out_size, bool use_cuda) {
			if (cv::cuda::getCudaEnabledDeviceCount() > 0 && use_cuda) {
				cv::cuda::GpuMat gpuIn;
				cv::cuda::GpuMat gpuOut;

				gpuIn.upload(src);

				cv::cuda::resize(gpuIn, gpuOut, out_size);

				gpuOut.download(dst);
			}
			else {
				cv::resize(src, dst, out_size);
			}
		}

		bool checkForProvider(const std::string provider_str) {
			std::vector<std::string> providers = Ort::GetAvailableProviders();
			for (const auto& provider : providers) {
				if (provider_str == provider)
					return true;
			}
			return false;
		}

		cv::Mat getRoiWithPadding(const cv::Mat& image, cv::Rect roi) {
			// Create rects representing the image and the ROI
			auto image_rect = cv::Rect(0, 0, image.cols, image.rows);

			// Find intersection, i.e. valid crop region
			auto intersection = image_rect & roi;

			// Move intersection to the result coordinate space
			auto inter_roi = intersection - roi.tl();

			// Create gray image and copy intersection
			cv::Mat crop = cv::Mat::ones(roi.size(), image.type());
			image(intersection).copyTo(crop(inter_roi));

			return crop;
		}

	}

}