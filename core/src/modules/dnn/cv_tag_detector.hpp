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

#include "models.hpp"
#include <opencv2/aruco.hpp>

class CVTagDetectorImpl : public TagModelImpl {
public:
	CVTagDetectorImpl();
	virtual void load(const std::string& model_path, Executor target_executor) override;
	virtual void unload() override;
	virtual bool is_loaded() override;
	virtual cv::Size net_size() override;
	virtual void infer(const cv::Mat& image, std::vector<Tag>& tags) override;
	virtual void infer(const std::vector<cv::Mat>& images, std::vector<std::vector<Tag>>& tags) override;
	void set_params(cv::aruco::DetectorParameters new_params);
	void set_dict_type(cv::aruco::PredefinedDictionaryType dict_type);
	void set_preprocess_size(cv::Size new_size);
private:
	cv::aruco::DetectorParameters det_params;
	cv::aruco::Dictionary det_dict;
	cv::aruco::ArucoDetector detector;
	cv::Size preprocess_size;
};

/// <summary>
/// Dummy module that uses OpenCV's ArUco detector in place of an actual machine learning model
/// </summary>
class CVTagDetector : public TagModel {
public:
	CVTagDetector();
	void set_params(cv::aruco::DetectorParameters new_params);
	void set_dict_type(cv::aruco::PredefinedDictionaryType dict_type);
	void set_preprocess_size(cv::Size new_size);
};
