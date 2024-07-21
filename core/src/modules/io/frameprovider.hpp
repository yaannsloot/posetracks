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
#include <future>

namespace me::io {

	/// <summary>
	/// Base abstract class for a custom frame provider
	/// </summary>
	class FrameProviderImpl {
	public:
		virtual bool load(std::string path, bool use_hw_accel = false) = 0;
		virtual bool next_frame(cv::Mat& frame, int retry_count = 100) = 0;
		virtual bool grab_frame(cv::Mat& frame, int frame_id, int retry_count = 100) = 0;
		virtual std::vector<std::shared_future<void>> next_frames(std::vector<cv::Mat>& frames, std::vector<bool>& success, int batch_size, int retry_count = 100) = 0;
		virtual std::vector<std::shared_future<void>> grab_frames(std::vector<cv::Mat>& frames, std::vector<bool>& success, int start_frame, int batch_size, int retry_count = 100) = 0;
		virtual bool set_frame(int frame_id) = 0;
		virtual int current_frame() = 0;
		virtual int frame_count() = 0;
		virtual cv::Size frame_size() = 0;
		virtual double fps() = 0;
		virtual void close() = 0;
		virtual bool is_open() = 0;
		virtual std::string get_fourcc_str() = 0;
		virtual int get_fourcc() = 0;
	};


	/// <summary>
	/// Managed instance of a frame provider
	/// </summary>
	class FrameProvider {
	public:
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
	protected:
		std::shared_ptr<FrameProviderImpl> fp_instance;
	};

}