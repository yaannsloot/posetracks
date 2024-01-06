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
#pragma once

#include <opencv2/opencv.hpp>

namespace me {
	
	namespace dnn {

		// GENERAL TODO: Make generic class definitions for model types <- DONE

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

		enum class FeatureDistanceType {
			EUCLIDEAN,
			NORM_EUCLIDEAN
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

		struct Feature {
			Feature() {}
			Feature(std::vector<double>& data) : data(data) {}
			Feature(std::initializer_list<double> data) : data(data) {}
			Feature(const Feature& other) : data(other.data) {}
			std::vector<double> data;
			double norm() const;
			Feature operator/(double val) const;
			Feature operator-(const Feature& other) const;
			Feature& operator=(const Feature& other);
			double dist(const Feature& other, FeatureDistanceType d_type = FeatureDistanceType::NORM_EUCLIDEAN) const;
			size_t size() const;
		};

		/// <summary>
		/// Specialized container for fast mean and median operations on a managed set of feature vectors.
		/// 
		/// Currently uses a list of sets for each element so that everything is pre-sorted for median calculations.
		/// This currently uses a linear search which could slow down operations when the feature length reaches a considerable size
		/// Consider using a custon implementation of a red-black tree that uses order statistics for faster search and insert
		/// </summary>
		class FeatureSet {
		public:
			FeatureSet(size_t feature_length) : feature_length(feature_length) {}
			void add(Feature& f);
			const Feature& at(size_t index) const;
			void remove(size_t index);
			void erase(std::vector<Feature>::iterator position);
			const Feature& mean() const;
			std::vector<Feature>::iterator begin();
			std::vector<Feature>::iterator end();
			size_t size() const;
			size_t length() const;
			const Feature& operator[](size_t index) const;
		private:
			std::vector<Feature> features;
			size_t feature_length;
			Feature mean_feature;
		};

		class FeatureSpace {
		public:
			FeatureSpace(size_t feature_length) : feature_length(feature_length) {}

			/// <summary>
			/// Performs a single feature assignment into this feature space. This is not a temporally stable operation. 
			/// If two features from the same time frame are added using this function and are similar enough, they will be assigned the same id.
			/// </summary>
			/// <param name="input">Feature to be added to this feature space</param>
			/// <param name="threshold">Maximum distance to potential feature sets within this space</param>
			/// <param name="identity_type">Which feature to use as the set identity during comparisons</param>
			/// <returns>An Index pointing to the feature set the input was assigned to</returns>
			size_t assign(Feature &input, double threshold = 0.4, FeatureDistanceType dist_type = FeatureDistanceType::NORM_EUCLIDEAN);

			/// <summary>
			/// Performs a one-to-one assignment of a vector of features to this feature space. 
			/// All provided features are scored relative to existing feature sets and assigned in ascending order based on distance,
			/// continuing until all selected features are exhausted or existing feature sets are exhausted, whichever comes first.
			/// If there are leftover input features, they will be assigned to new feature sets.
			/// </summary>
			/// <param name="input">Vector of features to be added to this feature space</param>
			/// <param name="threshold">Maximum distance to potential feature sets within this space</param>
			/// <param name="identity_type">Which feature to use as the set identity during comparisons</param>
			/// <param name="mask">Optional mask vector for excluding existing sets from assignment</param>
			/// <returns>A vector of indexes that reference the set each input feature was assigned to</returns>
			std::vector<size_t> assign(std::vector<Feature>& input, double threshold = 0.4, 
				FeatureDistanceType dist_type = FeatureDistanceType::NORM_EUCLIDEAN, std::vector<int> mask = std::vector<int>());
			std::vector<FeatureSet>::iterator begin();
			std::vector<FeatureSet>::iterator end();
			size_t size();
			size_t length();
			void clear();
			FeatureSet& at(size_t index);
			FeatureSet& operator[](size_t index);
		private:
			size_t feature_length;
			std::vector<FeatureSet> feature_sets;
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

		std::vector<float> LetterboxImage(const cv::Mat& src, cv::Mat& dst, const cv::Size& out_size, bool use_cuda = true);

		std::vector<float> FitImage(const cv::Mat& src, cv::Mat& dst, const cv::Size& out_size, bool use_cuda = true);

		void StretchImage(const cv::Mat& src, cv::Mat& dst, const cv::Size& out_size, bool use_cuda = true);

		bool checkForProvider(const std::string provider_str);

		cv::Mat getRoiWithPadding(const cv::Mat& image, cv::Rect roi);

	}

}