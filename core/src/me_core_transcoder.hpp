/*
me_core_transcoder.hpp
Implements a class that serves as a wrapper for cv::VideoCapture

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

#ifndef ME_CORE_TRANSCODER_HPP
#define ME_CORE_TRANSCODER_HPP

#include <me_core.hpp>
#include <me_core_simplepool.hpp>
#include <atomic>

namespace me {

	namespace core {

		// Managed instance of cv::VideoCapture
		class Transcoder {
		public:
			Transcoder();
			~Transcoder();
			bool load(std::string path, bool use_hw_accel = false);
			bool next_frame(cv::Mat& frame, int retry_count = 100);
			bool grab_frame(cv::Mat& frame, int frame_id, int retry_count = 100);
			std::vector<std::shared_future<void>> next_frames(std::vector<cv::Mat>& frames, std::vector<bool>& success, int batch_size, int retry_count = 100);
			std::vector<std::shared_future<void>> grab_frames(std::vector<cv::Mat>& frames, std::vector<bool>& success, int start_frame, int batch_size, int retry_count = 100);
			bool set_frame(int frame_id);
			int current_frame();
			int frame_count();
			cv::Size frame_size();
			double fps();
			void close();
			bool is_open();
			std::string get_fourcc_str();
			int get_fourcc();
		private:
			cv::VideoCapture cap;
			std::string last_path;
		};

	}

}

#endif