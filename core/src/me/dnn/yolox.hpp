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

#include "models.hpp"

namespace me::dnn::models {

	class YOLOXModelImpl : public DetectionModelImpl {
	public:
		YOLOXModelImpl();
		virtual cv::Size net_size() override;
		virtual void infer(const cv::Mat& image, std::vector<Detection>& detections, float conf_thresh, float iou_thresh) override;
		virtual void infer(const std::vector<cv::Mat>& images, std::vector<std::vector<Detection>>& detections, float conf_thresh, float iou_thresh) override;
	};

	class YOLOXModel : public DetectionModel {
	public:
		YOLOXModel();
	};

}