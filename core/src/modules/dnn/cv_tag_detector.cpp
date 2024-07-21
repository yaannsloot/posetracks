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

#include "cv_tag_detector.hpp"
#include <numeric>
#include <execution>

namespace me::dnn::models {

	CVTagDetectorImpl::CVTagDetectorImpl()
	{
		logid = "me_opencv_aruco_detector_driver";
		det_params = cv::aruco::DetectorParameters();
		det_dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_1000);
		detector = cv::aruco::ArucoDetector(det_dict, det_params);
		preprocess_size = cv::Size();
	}

	void CVTagDetectorImpl::load(const std::string& model_path, Executor target_executor) {}

	void CVTagDetectorImpl::unload() {}

	bool CVTagDetectorImpl::is_loaded()
	{
		return true;
	}

	cv::Size CVTagDetectorImpl::net_size()
	{
		return preprocess_size;
	}

	void CVTagDetectorImpl::infer(const cv::Mat& image, std::vector<Tag>& tags)
	{
		std::vector<cv::Mat> images{ image };
		std::vector<std::vector<Tag>> batch_tags;
		infer(images, batch_tags);
		tags = batch_tags[0];
	}

	void CVTagDetectorImpl::infer(const std::vector<cv::Mat>& images, std::vector<std::vector<Tag>>& tags)
	{
		tags.clear();
		tags.resize(images.size());
		std::vector<size_t> img_indices(images.size());
		std::iota(img_indices.begin(), img_indices.end(), 0);
		auto* img_ptr = images.data();
		auto* tags_ptr = tags.data();
		std::for_each(std::execution::par_unseq, img_indices.begin(), img_indices.end(), [&] (size_t i) {
			std::vector<int> ids;
			std::vector<std::vector<cv::Point2f> > corners, rejected;
			cv::Size normalize_dims = img_ptr[i].size();
			if (preprocess_size == cv::Size())
				detector.detectMarkers(img_ptr[i], corners, ids, rejected);
			else {
				normalize_dims = preprocess_size;
				cv::Mat resized_img;
				cv::resize(img_ptr[i], resized_img, preprocess_size, 0.0, 0.0);
				detector.detectMarkers(resized_img, corners, ids, rejected);
			}
			auto* corners_ptr = corners.data();
			auto* ids_ptr = ids.data();
			for (size_t j = 0; j < corners.size(); ++j) {
				auto& tag_corners = corners_ptr[j];
				cv::Point2d ca(tag_corners[0].x / normalize_dims.width, tag_corners[0].y / normalize_dims.height);
				cv::Point2d cb(tag_corners[1].x / normalize_dims.width, tag_corners[1].y / normalize_dims.height);
				cv::Point2d cc(tag_corners[2].x / normalize_dims.width, tag_corners[2].y / normalize_dims.height);
				cv::Point2d cd(tag_corners[3].x / normalize_dims.width, tag_corners[3].y / normalize_dims.height);
				tags_ptr[i].push_back(Tag(ids_ptr[j], ca, cb, cc, cd));
			}
		});
	}

	void CVTagDetectorImpl::set_params(cv::aruco::DetectorParameters new_params)
	{
		det_params = new_params;
		detector = cv::aruco::ArucoDetector(det_dict, det_params);
	}

	void CVTagDetectorImpl::set_dict_type(cv::aruco::PredefinedDictionaryType dict_type)
	{
		det_dict = cv::aruco::getPredefinedDictionary(dict_type);
		detector = cv::aruco::ArucoDetector(det_dict, det_params);
	}

	void CVTagDetectorImpl::set_preprocess_size(cv::Size new_size)
	{
		preprocess_size = new_size;
	}

	CVTagDetector::CVTagDetector()
	{
		model_ptr = std::make_shared<CVTagDetectorImpl>();
	}

	void CVTagDetector::set_params(cv::aruco::DetectorParameters new_params)
	{
		if (model_ptr != nullptr) {
			std::shared_ptr<CVTagDetectorImpl> cv_det_ptr = std::dynamic_pointer_cast<CVTagDetectorImpl>(model_ptr);
			cv_det_ptr->set_params(new_params);
		}
	}

	void CVTagDetector::set_dict_type(cv::aruco::PredefinedDictionaryType dict_type)
	{
		if (model_ptr != nullptr) {
			std::shared_ptr<CVTagDetectorImpl> cv_det_ptr = std::dynamic_pointer_cast<CVTagDetectorImpl>(model_ptr);
			cv_det_ptr->set_dict_type(dict_type);
		}
	}

	void CVTagDetector::set_preprocess_size(cv::Size new_size)
	{
		if (model_ptr != nullptr) {
			std::shared_ptr<CVTagDetectorImpl> cv_det_ptr = std::dynamic_pointer_cast<CVTagDetectorImpl>(model_ptr);
			cv_det_ptr->set_preprocess_size(new_size);
		}
	}

}