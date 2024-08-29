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
#include <opencv2/tracking.hpp>

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
	void scale_detection(double scale_factor);
};

enum class ScalingMode {
	/// <summary>
	/// Use scale detection to determine original coordinate scale.
	/// </summary>
	AUTO,
	/// <summary>
	/// Normalize input using source net size before scaling to frame dimensions.
	/// </summary>
	NORMALIZE_INPUT,
	/// <summary>
	/// Scale directly to frame dimensions. Input must be normalized before using this option.
	/// </summary>
	DIRECT
};

/// <summary>
/// 
/// </summary>
/// <param name="detections"></param>
/// <param name="src_net_size"></param>
/// <param name="target_frame_size"></param>
/// <param name="scaling_mode"></param>
void fixDetectionCoordinates(std::vector<Detection>& detections, cv::Size src_net_size, cv::Size target_frame_size, ScalingMode scaling_mode = ScalingMode::NORMALIZE_INPUT);

/// <summary>
/// 
/// </summary>
/// <param name="detections"></param>
/// <param name="src_net_size"></param>
/// <param name="target_frame_size"></param>
/// <param name="scaling_mode"></param>
void fixDetectionCoordinates(std::vector<std::vector<Detection>>& detections, cv::Size src_net_size, cv::Size target_frame_size, ScalingMode scaling_mode = ScalingMode::NORMALIZE_INPUT);

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

struct Tag {
	Tag() {}
	Tag(int id) : id(id) {}
	Tag(cv::Point2d& ca, cv::Point2d& cb, cv::Point2d& cc, cv::Point2d& cd) : corners{ca, cb, cc, cd} {}
	Tag(int id, cv::Point2d& ca, cv::Point2d& cb, cv::Point2d& cc, cv::Point2d& cd) : id(id), corners{ ca, cb, cc, cd } {}
	Tag(int id, double conf) : id(id), conf(conf) {}
	Tag(double conf, cv::Point2d& ca, cv::Point2d& cb, cv::Point2d& cc, cv::Point2d& cd) : conf(conf), corners{ ca, cb, cc, cd } {}
	Tag(int id, double conf, cv::Point2d& ca, cv::Point2d& cb, cv::Point2d& cc, cv::Point2d& cd) : id(id), conf(conf), corners{ca, cb, cc, cd} {}
	cv::Point2d corners[4];
	cv::Point2d& operator[](const size_t& index) { return corners[index]; }
	int id = 0;
	double conf = 1.0;
};

/// <summary>
/// Specialized container for fast mean operations on a managed set of feature vectors.
/// 
/// Tracks the mean by updating it on every insertion. Each inserted feature updates the mean through weighted addition.
/// If a feature is removed, the mean is recalculated using the internal list of features. 
/// This means it is much faster to insert a new feature than it is to remove one.
/// </summary>
class FeatureSet {
public:
	/// <summary>
	/// 
	/// </summary>
	/// <param name="feature_length"></param>
	FeatureSet(size_t feature_length) : feature_length(feature_length) {}

	/// <summary>
	/// 
	/// </summary>
	/// <param name="f"></param>
	void add(Feature& f);

	/// <summary>
	/// 
	/// </summary>
	/// <param name="index"></param>
	/// <returns></returns>
	const Feature& at(size_t index) const;

	/// <summary>
	/// 
	/// </summary>
	/// <param name="index"></param>
	void remove(size_t index);

	/// <summary>
	/// 
	/// </summary>
	/// <param name="position"></param>
	void erase(std::vector<Feature>::iterator position);

	/// <summary>
	/// 
	/// </summary>
	/// <returns></returns>
	const Feature& mean() const;

	/// <summary>
	/// 
	/// </summary>
	/// <returns></returns>
	std::vector<Feature>::iterator begin();

	/// <summary>
	/// 
	/// </summary>
	/// <returns></returns>
	std::vector<Feature>::iterator end();
	size_t size() const;

	/// <summary>
	/// 
	/// </summary>
	/// <returns></returns>
	size_t length() const;

	/// <summary>
	/// 
	/// </summary>
	/// <param name="index"></param>
	/// <returns></returns>
	const Feature& operator[](size_t index) const;
private:
	std::vector<Feature> features;
	size_t feature_length;
	Feature mean_feature;
};


class FeatureSpace {
public:
	/// <summary>
	/// Constructs a new feature space
	/// </summary>
	/// <param name="feature_length">Expected feature length of all features assigned to this space</param>
	FeatureSpace(size_t feature_length) : feature_length(feature_length) {}

	/// <summary>
	/// Performs a single feature assignment into this feature space. This is not a temporally stable operation. 
	/// If two features from the same time frame are added using this function and are similar enough, they will be assigned the same id.
	/// </summary>
	/// <param name="input">Feature to be added to this feature space</param>
	/// <param name="threshold">Maximum distance to potential feature sets within this space</param>
	/// <param name="dist_type">Distance operation to use</param>
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
	/// <param name="dist_type">Distance operation to use</param>
	/// <param name="mask">Optional mask vector for excluding existing sets from assignment</param>
	/// <returns>A vector of indexes that reference the set each input feature was assigned to</returns>
	std::vector<size_t> assign(std::vector<Feature>& input, double threshold = 0.4, 
		FeatureDistanceType dist_type = FeatureDistanceType::NORM_EUCLIDEAN, std::vector<int> mask = std::vector<int>());

	/// <summary>
	/// Returns an iterator pointing to the first element in the internal vector that stores all the tracked feature sets in this feature space
	/// </summary>
	std::vector<FeatureSet>::iterator begin();

	/// <summary>
	/// Returns an iterator pointing to the past-the-end element in the internal vector that stores all the tracked feature sets in this feature space
	/// </summary>
	std::vector<FeatureSet>::iterator end();

	/// <summary>
	/// Returns the number of tracked feature sets in this feature space
	/// </summary>
	size_t size();

	/// <summary>
	/// Returns the expected feature length of this feature space
	/// </summary>
	size_t length();

	/// <summary>
	/// Removes all feature sets from this feature space
	/// </summary>
	void clear();

	/// <summary>
	/// Returns a reference to the feature set at the specified position within this feature space
	/// </summary>
	/// <param name="index">Index of the desired feature set</param>
	FeatureSet& at(size_t index);

	/// <summary>
	/// Returns a reference to the feature set at the specified position within this feature space
	/// </summary>
	/// <param name="index">Index of the desired feature set</param>
	FeatureSet& operator[](size_t index);
private:
	size_t feature_length;
	std::vector<FeatureSet> feature_sets;
};

struct TrackerState {
	Feature f;
	Detection d;
	size_t t = 0;
};

class FeatureTracker {
public:
	FeatureTracker(size_t feature_length) : f_space(feature_length) {}
	std::vector<size_t> assign(std::vector<Detection>& input_boxes, std::vector<Feature>& input_features, double score_threshold = 0.7, double f_space_threshold = 0.4,
		FeatureDistanceType dist_type = FeatureDistanceType::NORM_EUCLIDEAN, std::vector<int> mask = std::vector<int>());
private:
	std::vector<std::vector<Detection>> detections;
	std::vector<TrackerState> predicted_states;
	std::vector<cv::KalmanFilter> filters;
	std::vector<int> init_list;
	size_t last_frame = 0;
	FeatureSpace f_space; // Internal feature space
	void grow_detections_vector();
	void grow_state_vector(); // Grows the state vector if the feature space size has increased
	void grow_filter_vector(); // Grows the filter vector if the feature space size has increased
	void update_predictions(std::vector<size_t> targets); // Update predicted states for specified feature space targets
};

inline double iou(cv::Rect2d a, cv::Rect2d b);

std::vector<size_t> nms(std::vector<Detection>& detections, float iou_thresh);

std::vector<float> LetterboxImage(const cv::Mat& src, cv::Mat& dst, const cv::Size& out_size);

std::vector<float> FitImage(const cv::Mat& src, cv::Mat& dst, const cv::Size& out_size);

void StretchImage(const cv::Mat& src, cv::Mat& dst, const cv::Size& out_size);

bool checkForProvider(const std::string provider_str);

cv::Mat getRoiWithPadding(const cv::Mat& image, cv::Rect roi);

cv::Mat getRoiNoPadding(const cv::Mat& image, cv::Rect roi);

bool isRoiOutsideImage(const cv::Size& imageSize, const cv::Rect& roi);

void drawTags(cv::Mat& out_image, std::vector<Tag>& tags);
