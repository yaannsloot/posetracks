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

#include "frameprovider.hpp"
#include <me/threading/simplepool.hpp>

namespace me {

	namespace io {

		/// <summary>
		/// Frame provider implementation for image lists
		/// </summary>
		class ImageListImpl : public FrameProviderImpl {
		public:
			ImageListImpl();
			virtual bool load(std::string path, bool use_hw_accel) override;
			virtual bool next_frame(cv::Mat& frame, int retry_count) override;
			virtual bool grab_frame(cv::Mat& frame, int frame_id, int retry_count) override;
			virtual std::vector<std::shared_future<void>> next_frames(std::vector<cv::Mat>& frames, std::vector<bool>& success, int batch_size, int retry_count) override;
			virtual std::vector<std::shared_future<void>> grab_frames(std::vector<cv::Mat>& frames, std::vector<bool>& success, int start_frame, int batch_size, int retry_count) override;
			virtual bool set_frame(int frame_id) override;
			virtual int current_frame() override;
			virtual int frame_count() override;
			virtual cv::Size frame_size() override;
			virtual double fps() override;
			virtual void close() override;
			virtual bool is_open() override;
			virtual std::string get_fourcc_str() override;
			virtual int get_fourcc() override;
		private:
			int f_index = 0;
			cv::Size f_size;
			std::vector<std::string> paths;
			std::shared_ptr<threading::SimplePool> internal_pool;
			std::queue<std::pair<int, std::shared_future<cv::Mat>>> future_queue;
			int max_queue = std::thread::hardware_concurrency();
		};

		/// <summary>
		/// Frame provider instance that operates off of a list of images
		/// </summary>
		class ImageList : public FrameProvider {
		public:
			ImageList();
		};

	}

}