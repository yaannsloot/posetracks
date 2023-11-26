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
			
			class RTMPoseModelImpl : public PoseModelImpl {
			public:
				RTMPoseModelImpl();
				cv::Size net_size() override;
				void infer(const cv::Mat& image, Pose& pose) override;
				void infer(const std::vector<cv::Mat>& images, std::vector<Pose>& poses) override;
			};

			class RTMPoseModel : public PoseModel {
			public:
				RTMPoseModel();
			};

		}

	}

}