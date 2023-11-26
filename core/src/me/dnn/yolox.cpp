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

#include "yolox.hpp"

namespace me {

	namespace dnn {

		namespace models {



			YOLOXModelImpl::YOLOXModelImpl()
			{
			}

			cv::Size YOLOXModelImpl::net_size()
			{
				return cv::Size();
			}

			void YOLOXModelImpl::infer(const cv::Mat& image, std::vector<Detection>& detections, float conf_thresh, float iou_thresh)
			{
			}

			void YOLOXModelImpl::infer(const std::vector<cv::Mat>& images, std::vector<std::vector<Detection>>& detections, float conf_thresh, float iou_thresh)
			{
			}

			YOLOXModel::YOLOXModel()
			{
				model_ptr = std::make_shared<YOLOXModelImpl>();
			}

		}

	}

}