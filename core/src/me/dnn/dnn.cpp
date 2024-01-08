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

#include "dnn.hpp"
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudawarping.hpp>
#include <opencv2/cudaarithm.hpp>
#include <me/threading/simplepool.hpp>
#include <onnxruntime_cxx_api.h>
#include <execution>

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

		double Feature::norm() const {
			const double* ptr_data = this->data.data();
			auto elements = this->data.size();
			double sum_of_squares = 0;
			for (size_t i = 0; i < elements; ++i) {
				sum_of_squares += ptr_data[i] * ptr_data[i];
			}
			return (sum_of_squares > 0) ? std::sqrt(sum_of_squares) : 0;
		}

		Feature Feature::operator/(double val) const {
			if (val > 0 && this->size() > 0) {
				Feature output;
				output.data = this->data;
				double* output_ptr = output.data.data();
				auto elements = output.data.size();
				for (size_t i = 0; i < elements; ++i) {
					output_ptr[i] /= val;
				}
				return output;
			}
			return *this;
		}

		Feature Feature::operator-(const Feature& other) const {
			Feature output;
			if (this->size() > 0 && other.size() > 0 && this->size() == other.size()) {
				auto elements = this->data.size();
				output.data.resize(elements);
				double* output_ptr = output.data.data();
				const double* data_ptr = this->data.data();
				const double* other_ptr = other.data.data();
				for (size_t i = 0; i < elements; ++i) {
					output_ptr[i] = data_ptr[i] - other_ptr[i];
				}
			}
			return output;
		}

		Feature& Feature::operator=(const Feature& other) {
			if (this != &other) {
				this->data = other.data;
			}
			return *this;
		}

		double Feature::dist(const Feature& other, FeatureDistanceType d_type) const {
			if (this->size() != other.size())
				return 0;
			if (d_type == FeatureDistanceType::NORM_EUCLIDEAN) {
				Feature norm_a = *this / this->norm();
				Feature norm_b = other / other.norm();
				return (norm_a - norm_b).norm();
			}
			else {
				return (*this - other).norm();
			}
		}

		size_t Feature::size() const {
			return data.size();
		}

		void FeatureSet::add(Feature& f) {
			if (f.size() != this->feature_length) {
				std::stringstream ss;
				ss << "Mismatch in expected feature length (" << this->feature_length << ") and provided length (" << f.size() << ").";
				throw std::runtime_error(ss.str());
			}
			if (this->features.size() == 0)
				this->mean_feature = f;
			else {
				size_t current_size = this->features.size();
				size_t new_size = current_size + 1;
				double scale_a = (double)current_size / (double)new_size;
				double scale_b = 1.0 / (double)new_size;
				std::transform(this->mean_feature.data.begin(), this->mean_feature.data.end(),
					f.data.begin(), mean_feature.data.begin(), 
					[scale_a, scale_b](double& a, double& b) -> double { return (a * scale_a) + (b * scale_b); });
			}
			this->features.push_back(f);
		}

		const Feature& FeatureSet::at(size_t index) const {
			return this->features[index];
		}

		void FeatureSet::remove(size_t index) {
			auto t_it = this->features.begin() + index;
			this->erase(t_it);
		}

		void FeatureSet::erase(std::vector<Feature>::iterator position) {
			this->features.erase(position);
			this->mean_feature.data.resize(this->feature_length);
			std::fill(std::execution::par, this->mean_feature.data.begin(), this->mean_feature.data.end(), 0);
			if (!this->features.empty()) {
				for (const auto& vec : this->features) {
					std::transform(this->mean_feature.data.begin(),
						this->mean_feature.data.end(), vec.data.begin(), this->mean_feature.data.begin(),
						std::plus<double>());
				}
				auto div = this->features.size();
				std::transform(this->mean_feature.data.begin(), this->mean_feature.data.end(), this->mean_feature.data.begin(),
					[div](double& a) -> double { return a / div; });
			}
		}

		const Feature& FeatureSet::mean() const {
			return this->mean_feature;
		}

		std::vector<Feature>::iterator FeatureSet::begin() {
			return this->features.begin();
		}

		std::vector<Feature>::iterator FeatureSet::end() {
			return this->features.end();
		}

		size_t FeatureSet::size() const {
			return this->features.size();
		}

		size_t FeatureSet::length() const {
			return this->feature_length;
		}

		const Feature& FeatureSet::operator[](size_t index) const {
			return this->at(index);
		}

		double dist_thread(const Feature& a, const Feature& mean, FeatureDistanceType d_type) {
			return a.dist(mean, d_type);
		}

		size_t FeatureSpace::assign(Feature& input, double threshold, FeatureDistanceType dist_type) {
			if (input.size() != this->feature_length) {
				std::stringstream ss;
				ss << "Mismatch in expected feature length (" << this->feature_length << ") and provided length (" << input.size() << ").";
				throw std::runtime_error(ss.str());
			}
			double lowest = std::numeric_limits<double>::infinity();
			bool thresh_passed = false;
			size_t lowest_index = 0;
			size_t num_sets = this->feature_sets.size();
			FeatureSet* f_set_ptr = this->feature_sets.data();
			std::vector<size_t> f_set_indices(num_sets);
			std::iota(f_set_indices.begin(), f_set_indices.end(), 0);
			std::mutex lock;
			std::for_each(std::execution::par_unseq, f_set_indices.begin(), f_set_indices.end(), [&] (size_t i) {
				double dist = input.dist(f_set_ptr[i].mean(), dist_type);
				std::lock_guard<std::mutex> lock_guard(lock);
				if (dist < threshold && dist < lowest) {
					lowest_index = i;
					lowest = dist;
					thresh_passed = true;
				}
			});
			auto it = this->feature_sets.end();
			if (thresh_passed) {
				it = this->feature_sets.begin() + lowest_index;
			}
			else {
				this->feature_sets.emplace_back(this->feature_length);
				it = this->feature_sets.end() - 1;
			}
			it->add(input);
			return std::distance(this->feature_sets.begin(), it);

		}

		std::vector<size_t> FeatureSpace::assign(std::vector<Feature>& input, double threshold,
			FeatureDistanceType dist_type, std::vector<int> mask) {
			std::vector<size_t> output;
			output.resize(input.size());
			std::set<size_t> selected_features;
			std::vector<std::tuple<double, size_t, size_t>> scores;
			if (mask.size() > 0 && mask.size() != this->feature_sets.size()) {
				std::stringstream ss;
				ss << "Mismatch in expected mask length (" << this->feature_sets.size() << ") and provided length (" << mask.size() << ").";
				throw std::runtime_error(ss.str());
			}

			size_t num_features = this->feature_sets.size();
			size_t mask_len = mask.size();
			int* mask_ptr = mask.data();
			std::vector<size_t> valid_indices;
			for (size_t i = 0; i < num_features; ++i) {
				if (mask_len == 0 || mask_ptr[i] == 0) {
					valid_indices.push_back(i);
				}
			}
			std::mutex lock;
			std::for_each(std::execution::par_unseq, valid_indices.begin(), valid_indices.end(), [&](size_t i) {
				const FeatureSet& feature_set = this->feature_sets[i];
				for (const auto& input_feature : input) {
					double dist = input_feature.dist(feature_set.mean(), dist_type);
					if (dist < threshold) {
						std::lock_guard<std::mutex> lock_guard(lock);
						scores.push_back(std::tuple<double, size_t, size_t>(dist, &input_feature - &input[0], i));
						selected_features.insert(&input_feature - &input[0]);
					}
				}
			});

			auto comp = [](const std::tuple<double, size_t, size_t>& first, 
				const std::tuple<double, size_t, size_t>& second) {
				return std::get<0>(first) < std::get<0>(second);
			};
			std::sort(scores.begin(), scores.end(), comp);

			std::set<size_t> spent_features;
			std::set<size_t> spent_feature_sets;
			std::vector<int> feature_mask(input.size(), 0);

			for (auto& score_tuple : scores) {
				if (spent_features.size() == selected_features.size() || 
					spent_feature_sets.size() == this->feature_sets.size())
					break;
				size_t feature_index = std::get<1>(score_tuple);
				size_t feature_set_index = std::get<2>(score_tuple);
				if (spent_features.find(feature_index) != spent_features.end() ||
					spent_feature_sets.find(feature_set_index) != spent_feature_sets.end())
					continue;
				this->feature_sets[feature_set_index].add(input[feature_index]);
				spent_features.insert(feature_index);
				spent_feature_sets.insert(feature_set_index);
				feature_mask[feature_index] = 1;
				output[feature_index] = feature_set_index;
			}

			for (size_t i = 0; i < input.size(); ++i) {
				if (feature_mask[i] >= 1)
					continue;
				this->feature_sets.emplace_back(this->feature_length);
				this->feature_sets.back().add(input[i]);
				output[i] = this->feature_sets.size() - 1;
			}

			return output;
		}

		std::vector<FeatureSet>::iterator FeatureSpace::begin() {
			return this->feature_sets.begin();
		}

		std::vector<FeatureSet>::iterator FeatureSpace::end() {
			return this->feature_sets.end();
		}

		size_t FeatureSpace::size() {
			return this->feature_sets.size();
		}

		size_t FeatureSpace::length() {
			return this->feature_length;
		}

		void FeatureSpace::clear() {
			this->feature_sets.clear();
		}

		FeatureSet& FeatureSpace::at(size_t index) {
			return this->feature_sets[index];
		}

		FeatureSet& FeatureSpace::operator[](size_t index) {
			return this->at(index);
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

							float tmp = src_images->at(n).ptr<uchar>(h)[w * 3ul + c];

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