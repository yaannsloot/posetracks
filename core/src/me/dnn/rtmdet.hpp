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

namespace me {

	namespace dnn {

		namespace models {

			class RTMDetModel : public DetectionModel {
			public:
				RTMDetModel();

				/// <summary>
				/// The size of the currently loaded model's input in pixels
				/// </summary>
				/// <returns>A cv::Size object with width and height in pixels</returns>
				cv::Size net_size() override;

				/// <summary>
				/// Run an inference on the currently loaded model
				/// </summary>
				/// <param name="image">Input image to process</param>
				/// <param name="detections">Output vector for bounding box detections</param>
				/// <param name="conf_thresh">Confidence threshold</param>
				/// <param name="iou_thresh">IoU threshold</param>
				void infer(const cv::Mat& image, std::vector<Detection>& detections, float conf_thresh, float iou_thresh) override;

				/// <summary>
				/// Run a batch inference on the currently loaded model
				/// </summary>
				/// <param name="images">Input vector containing images to process</param>
				/// <param name="detections">Output vector of vectors containing bounding box detections for each image</param>
				/// <param name="conf_thresh">Confidence threshold</param>
				/// <param name="iou_thresh">IoU threshold</param>
				void infer(const std::vector<cv::Mat>& images, std::vector<std::vector<Detection>>& detections, float conf_thresh, float iou_thresh) override;

			};

		}

	}

}