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

#define NOMINMAX

#include "dnn.hpp"

#ifdef ME_CUDA_IMGPROC_ENABLED
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudawarping.hpp>
#include <opencv2/cudaarithm.hpp>
#endif

#include <onnxruntime_cxx_api.h>
#include <execution>

namespace me::dnn {

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

	void Detection::scale_detection(double scale_factor)
	{
		auto size = this->bbox.size();
		auto center = (this->bbox.tl() + this->bbox.br()) / 2;
		size *= scale_factor;
		cv::Point2d new_tl(
			center.x - size.width / 2,
			center.y - size.height / 2
		);
		this->bbox = cv::Rect2d(new_tl, size);
	}

	void iou_ratio(std::vector<Detection>& detections, cv::Size& src_net_size, int& norm_count, int& net_size_count) {
		cv::Rect2d norm_rect(0, 0, 1, 1);
		cv::Rect2d net_rect(0, 0, src_net_size.width, src_net_size.height);
		norm_count = 0;
		net_size_count = 0;
		for (auto& det : detections) {
			double iou_a = iou(det.bbox, norm_rect);
			double iou_b = iou(det.bbox, net_rect);
			if (iou_a > iou_b)
				++norm_count;
			else
				++net_size_count;
		}
	}

	void fixDetectionCoordinates(std::vector<Detection>& detections, cv::Size src_net_size, cv::Size target_frame_size, ScalingMode scaling_mode)
	{
		if (scaling_mode == ScalingMode::AUTO) {
			int norm_count = 0;
			int net_size_count = 0;
			iou_ratio(detections, src_net_size, norm_count, net_size_count);
			if (norm_count > net_size_count)
				scaling_mode = ScalingMode::DIRECT;
			else
				scaling_mode = ScalingMode::NORMALIZE_INPUT;
		}
		for (auto& det : detections) {
			auto& src_bbox = det.bbox;
			if (scaling_mode == ScalingMode::NORMALIZE_INPUT) {
				src_bbox.x /= src_net_size.width;
				src_bbox.y /= src_net_size.height;
				src_bbox.width /= src_net_size.width;
				src_bbox.height /= src_net_size.height;
			}
			src_bbox.x *= target_frame_size.width;
			src_bbox.y *= target_frame_size.height;
			src_bbox.width *= target_frame_size.width;
			src_bbox.height *= target_frame_size.height;
		}
	}

	void fixDetectionCoordinates(std::vector<std::vector<Detection>>& detections, cv::Size src_net_size, cv::Size target_frame_size, ScalingMode scaling_mode)
	{
		if (scaling_mode == ScalingMode::AUTO) {
			int norm_count = 0;
			int net_size_count = 0;
			for (auto& dets : detections) {
				iou_ratio(dets, src_net_size, norm_count, net_size_count);
			}
			if (norm_count > net_size_count)
				scaling_mode = ScalingMode::DIRECT;
			else
				scaling_mode = ScalingMode::NORMALIZE_INPUT;
		}
		for (auto& dets : detections) {
			fixDetectionCoordinates(dets, src_net_size, target_frame_size, scaling_mode);
		}
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

	// NOTE TO SELF. RECALCULATION MAY NOT BE NECESSARY. JUST CALCULATE THE SUM THROUGH MULTIPLICATION AND SUBTRACT WHAT IS REMOVED
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

		// Check for malformed inputs
		if (mask.size() > 0 && mask.size() != this->feature_sets.size()) {
			std::stringstream ss;
			ss << "Mismatch in expected mask length (" << this->feature_sets.size() << ") and provided length (" << mask.size() << ").";
			throw std::runtime_error(ss.str());
		}

		// Gather info about input and obtain pointers to data
		std::vector<size_t> output;
		output.resize(input.size());
		std::set<size_t> selected_features;
		std::vector<std::tuple<double, size_t, size_t>> scores;
		size_t num_features = this->feature_sets.size();
		size_t mask_len = mask.size();
		int* mask_ptr = mask.data();

		// Obtain a list of indices to feature sets that weren't masked
		std::vector<size_t> valid_indices;
		for (size_t i = 0; i < num_features; ++i) {
			if (mask_len == 0 || mask_ptr[i] == 0) {
				valid_indices.push_back(i);
			}
		}

		// Score input features to the mean feature of preexisting feature sets within this feature space. 
		// All distances must pass a threshold before being added to the scores
		// This part is computationally expensive and uses parallel processing to improve performance 
		// A mutex lock will be used for write synchronization
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

		// Sort list of scores in ascending order (smaller score is better and will be used first)
		auto comp = [](const std::tuple<double, size_t, size_t>& first, 
			const std::tuple<double, size_t, size_t>& second) {
			return std::get<0>(first) < std::get<0>(second);
		};
		std::sort(scores.begin(), scores.end(), comp);

		// Perform one-to-one assignment of input to existing feature sets that passed the threshold
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

		// Create new feature sets for the remaining input
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

	std::vector<size_t> FeatureTracker::assign(std::vector<Detection>& input_boxes, std::vector<Feature>& input_features, double score_threshold, double f_space_threshold,
		FeatureDistanceType dist_type, std::vector<int> mask) {

		// Check for malformed inputs
		std::vector<size_t> output;
		if (input_boxes.size() != input_features.size()) {
			std::stringstream ss;
			ss << "Error in provided input: num boxes and num features do not match (" << input_boxes.size() << " != " << input_features.size() << ").";
			throw std::runtime_error(ss.str());
		}
		if (mask.size() > 0 && mask.size() != this->f_space.size()) {
			std::stringstream ss;
			ss << "Mismatch in expected mask length (" << this->f_space.size() << ") and provided length (" << mask.size() << ").";
			throw std::runtime_error(ss.str());
		}

		// Gather info about input and obtain pointers to data
		size_t input_size = input_boxes.size();
		output.resize(input_size);
		size_t num_states = this->predicted_states.size();
		auto* state_ptr = this->predicted_states.data();
		auto* boxes_ptr = input_boxes.data();
		auto* features_ptr = input_features.data();
		size_t mask_len = mask.size();
		int* mask_ptr = mask.data();
		std::set<size_t> selected_input;
		std::vector<std::tuple<double, size_t, size_t>> scores;
		std::vector<size_t> valid_states;

		// Obtain list of valid state indices that both were updated on last assignment and were not masked
		for (size_t i = 0; i < num_states; ++i) {
			if ((mask_len == 0 || mask_ptr[i] == 0) && (state_ptr[i].t == last_frame)) {
				valid_states.push_back(i);
			}
		}

		// Score predicted states relative to input
		// Input boxes must overlap predictions and their related features must be within the score threshold
		// This part is computationally expensive and uses parallel processing to improve performance 
		// A mutex lock will be used for write synchronization
		std::mutex lock;
		std::for_each(std::execution::par_unseq, valid_states.begin(), valid_states.end(), [&](size_t i) {
			for (size_t j = 0; j < input_size; ++j) {
				auto& t_state = state_ptr[i];
				auto& bbox = boxes_ptr[j];
				auto& feature = features_ptr[j];
				auto iou_score = iou(bbox.bbox, t_state.d.bbox);
				if (iou_score > 0) {
					double dist = t_state.f.dist(feature, dist_type);
					if (dist < score_threshold) {
						std::lock_guard<std::mutex> lock_guard(lock);
						scores.push_back(std::tuple<double, size_t, size_t>(dist, i, j));
						selected_input.insert(j);
					}
				}
			}
		});

		// Sort list of scores in ascending order (smaller score is better and will be used first)
		auto comp = [](const std::tuple<double, size_t, size_t>& first,
			const std::tuple<double, size_t, size_t>& second) {
				return std::get<0>(first) < std::get<0>(second);
		};
		std::sort(scores.begin(), scores.end(), comp);

		// Perform one-to-one assignment of input to predicted states that passed the threshold
		std::set<size_t> spent_states;
		std::set<size_t> spent_input;
		std::vector<int> masked_indices(this->f_space.size(), 0);
		std::vector<int> input_mask(input_size, 0);
		for (auto& score_tuple : scores) {
			if (spent_input.size() == selected_input.size() ||
				spent_states.size() == valid_states.size())
				break;
			size_t state_index = std::get<1>(score_tuple);
			size_t input_index = std::get<2>(score_tuple);
			if (spent_input.find(input_index) != spent_input.end() ||
				spent_states.find(state_index) != spent_states.end())
				continue;
			this->f_space[state_index].add(features_ptr[input_index]);
			spent_states.insert(state_index);
			spent_input.insert(input_index);
			masked_indices[state_index] = 1;
			input_mask[input_index] = 1;
			output[input_index] = state_index;
		}

		// Using the mask generated from the above assignments, re-ID any left over detections to the feature space
		std::vector<size_t> reid_indices; // Index map
		std::vector<Feature> reid_features; // Features to pass to next step
		for (size_t i = 0; i < input_size; ++i) {
			if (input_mask[i] == 0) {
				reid_indices.push_back(i);
				reid_features.push_back(features_ptr[i]);
			}
		}
		// One-to-one assignment function from FeatureSpace slass
		auto reid_assignments = this->f_space.assign(reid_features, f_space_threshold, dist_type, masked_indices);

		// Merge id vector from feature space assignment to the output vector
		size_t num_assignments = reid_assignments.size();
		auto* assn_ptr = reid_assignments.data();
		for (size_t i = 0; i < num_assignments; ++i) {
			output[reid_indices[i]] = assn_ptr[i];
		}

		// Finish up by adjusting the size of all internal state vectors used by the tracker
		grow_detections_vector();
		// The detections vector is used by the update_predictions function. We must add the new detections here
		for (size_t i = 0; i < input_size; ++i) {
			this->detections[output[i]].push_back(input_boxes[i]);
		}
		grow_filter_vector(); // Add filters for new detections that weren't mapped to preexisting tracks
		grow_state_vector(); // Adjust the size of the state vector to reflect the size of the feature space
		update_predictions(output); // Update predicted states using the ids from the output. Predicted states will have last_frame + 1
		last_frame++; // Roll the last frame counter forward to match predicted states
		return output;

	}

	void FeatureTracker::grow_detections_vector() {
		size_t current = detections.size();
		size_t space_size = f_space.size();
		if (current < space_size)
			detections.resize(space_size);
	}

	void FeatureTracker::grow_state_vector() {
		size_t current = predicted_states.size();
		size_t space_size = f_space.size();
		if (current < space_size)
			predicted_states.resize(space_size);
	}

	void FeatureTracker::grow_filter_vector() {
		size_t current = filters.size();
		size_t space_size = f_space.size();
		if (current < space_size) {
			size_t diff = space_size - current;
			for (size_t i = 0; i < diff; ++i) {
				cv::KalmanFilter new_filter(4, 2);
				new_filter.transitionMatrix = (cv::Mat_<float>(4, 4) <<
					1, 0, 1, 0,
					0, 1, 0, 1,
					0, 0, 1, 0,
					0, 0, 0, 1);
				new_filter.measurementMatrix = (cv::Mat_<float>(2, 4) <<
					1, 0, 0, 0,
					0, 1, 0, 0);
				new_filter.measurementNoiseCov = cv::Mat_<float>::eye(2, 2) * 0.001;
				new_filter.processNoiseCov = cv::Mat_<float>::eye(4, 4) * 0.03;
				new_filter.statePre.at<float>(0) = 0;
				new_filter.statePre.at<float>(1) = 0;
				new_filter.statePre.at<float>(2) = 0;
				new_filter.statePre.at<float>(3) = 0;
				new_filter.statePost.at<float>(0) = 0;
				new_filter.statePost.at<float>(1) = 0;
				new_filter.statePost.at<float>(2) = 0;
				new_filter.statePost.at<float>(3) = 0;
				filters.push_back(new_filter);
			}
			init_list.resize(space_size, 0);
		}
	}

	/// <summary>
	/// Internal function used to update predicted states within the tracker. 
	/// These states are of type TrackerState and contain variables f, d, and t, 
	/// where f is a feature vector of length n, d is a bounding box prediction, 
	/// and t is the instance in time, last_frame + 1, that that prediction is assumed to occur.
	/// This function targets specific track ids, or indexes, and is meant to be 
	/// used after the primary calculations within a tracking iteration. All tracks modified during an iteration 
	/// are to be specified with the targets vector parameter. Also do note that
	/// both the filter vector and state vector should be grown with 
	/// grow_filter_vector() and grow_state_vector() before calling this function.
	/// </summary>
	/// <param name="targets">Predicted state indices to update</param>
	void FeatureTracker::update_predictions(std::vector<size_t> targets) {
		for (size_t& target : targets) {
			Feature& last_feature = *(f_space[target].end() - 1);
			Detection& last_detection = detections[target].back();
			if (init_list[target] == 0) {
				predicted_states[target].t = last_frame + 1;
				predicted_states[target].d = last_detection;
				predicted_states[target].f = last_feature;
				auto tl = last_detection.bbox.tl();
				auto br = last_detection.bbox.br();
				float cx = (float)((tl.x + br.x) / 2);
				float cy = (float)((tl.y + br.y) / 2);
				cv::Mat measurement = (cv::Mat_<float>(2, 1) << cx, cy);
				filters[target].predict();
				filters[target].correct(measurement);
				init_list[target] = 1;
			}
			else {
				predicted_states[target].t = last_frame + 1;
				predicted_states[target].f = last_feature;
				auto tl = last_detection.bbox.tl();
				auto br = last_detection.bbox.br();
				float cx = (float)((tl.x + br.x) / 2);
				float cy = (float)((tl.y + br.y) / 2);
				cv::Mat measurement = (cv::Mat_<float>(2, 1) << cx, cy);
				filters[target].correct(measurement);
				auto prediction = filters[target].predict();
				double estimated_x = (double)(prediction.at<float>(0));
				double estimated_y = (double)(prediction.at<float>(1));
				double width = last_detection.bbox.width;
				double height = last_detection.bbox.height;
				cv::Point2d new_tl(estimated_x - width / 2, estimated_y - height / 2);
				cv::Point2d new_br(estimated_x + width / 2, estimated_y + height / 2);
				predicted_states[target].d = last_detection;
				predicted_states[target].d.bbox = cv::Rect2d(new_tl, new_br);
			}
		}
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

#ifdef ME_CUDA_IMGPROC_ENABLED
		if (cv::cuda::getCudaEnabledDeviceCount() > 0 && use_cuda) {
			cv::cuda::GpuMat gpuIn;
			cv::cuda::GpuMat gpuOut;

			gpuIn.upload(src);

			cv::cuda::resize(gpuIn, gpuOut, cv::Size(mid_w, mid_h));

			cv::cuda::copyMakeBorder(gpuOut, gpuOut, top, down, left, right, cv::BORDER_CONSTANT, cv::Scalar(114, 114, 114));

			gpuOut.download(dst);
#else
		if (use_cuda) {
			cv::resize(src, dst, cv::Size(mid_w, mid_h));
			cv::copyMakeBorder(dst, dst, top, down, left, right, cv::BORDER_CONSTANT, cv::Scalar(114, 114, 114));
#endif
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

#ifdef ME_CUDA_IMGPROC_ENABLED
		if (cv::cuda::getCudaEnabledDeviceCount() > 0 && use_cuda) {
			cv::cuda::GpuMat gpuIn;
			cv::cuda::GpuMat gpuOut;

			gpuIn.upload(src);

			cv::cuda::resize(gpuIn, gpuOut, cv::Size(mid_w, mid_h));

			// Crop the image to the output size
			cv::cuda::GpuMat gpuCrop(gpuOut, cv::Rect(left, top, out_w, out_h));
			gpuCrop.download(dst);
		}
#else
		if (use_cuda) {
			cv::resize(src, dst, cv::Size(mid_w, mid_h));
			// Crop the image to the output size
			dst = dst(cv::Rect(left, top, out_w, out_h));
		}
#endif
		else {
			cv::resize(src, dst, cv::Size(mid_w, mid_h));
			// Crop the image to the output size
			dst = dst(cv::Rect(left, top, out_w, out_h));
		}

		std::vector<float> pad_info{ static_cast<float>(left), static_cast<float>(top), scale };
		return pad_info;
	}

	void StretchImage(const cv::Mat& src, cv::Mat& dst, const cv::Size& out_size, bool use_cuda) {
#ifdef ME_CUDA_IMGPROC_ENABLED
		if (cv::cuda::getCudaEnabledDeviceCount() > 0 && use_cuda) {
			cv::cuda::GpuMat gpuIn;
			cv::cuda::GpuMat gpuOut;

			gpuIn.upload(src);

			cv::cuda::resize(gpuIn, gpuOut, out_size);

			gpuOut.download(dst);
		}
#else
		if (use_cuda) {
			cv::resize(src, dst, out_size);
		}
#endif
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

	cv::Mat getRoiNoPadding(const cv::Mat& image, cv::Rect roi) {
		// Create rects representing the image and the ROI
		auto image_rect = cv::Rect(0, 0, image.cols, image.rows);

		// Find intersection, i.e. valid crop region
		auto intersection = image_rect & roi;

		return image(intersection);
	}

	bool isRoiOutsideImage(const cv::Size& imageSize, const cv::Rect& roi) {
		// Check if the ROI is completely to the left, right, above, or below the image
		bool isLeft = roi.x + roi.width <= 0;
		bool isRight = roi.x >= imageSize.width;
		bool isAbove = roi.y + roi.height <= 0;
		bool isBelow = roi.y >= imageSize.height;

		// If any of these conditions is true, the ROI is completely outside the image
		return isLeft || isRight || isAbove || isBelow;
	}

	void drawTags(cv::Mat& out_image, std::vector<Tag>& tags)
	{
		for (auto& tag : tags) {
			for (size_t i = 0; i < 3; ++i) {
				auto& pt1 = tag[i];
				auto& pt2 = tag[i + 1];
				cv::line(out_image, pt1, pt2, cv::Scalar(0, 255, 0), 2);
			}
			cv::line(out_image, tag[0], tag[3], cv::Scalar(0, 255, 0), 2);
			cv::drawMarker(out_image, tag[0], cv::Scalar(255, 0, 255), cv::MarkerTypes::MARKER_SQUARE, 20, 2);
			auto center = (tag[0] + tag[1] + tag[2] + tag[3]) / 4;
			cv::putText(out_image, std::to_string(tag.id), center, cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 0), 2);
		}
	}

}