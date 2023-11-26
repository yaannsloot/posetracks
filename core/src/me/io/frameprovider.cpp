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

#include "frameprovider.hpp"

namespace me {

	namespace io {

		bool FrameProvider::load(std::string path, bool use_hw_accel)
		{
			if (fp_instance != nullptr)
				return fp_instance->load(path, use_hw_accel);
			return false;
		}

		bool FrameProvider::next_frame(cv::Mat& frame, int retry_count)
		{
			if (fp_instance != nullptr)
				return fp_instance->next_frame(frame, retry_count);
			return false;
		}

		bool FrameProvider::grab_frame(cv::Mat& frame, int frame_id, int retry_count)
		{
			if (fp_instance != nullptr)
				return fp_instance->grab_frame(frame, frame_id, retry_count);
			return false;
		}

		std::vector<std::shared_future<void>> FrameProvider::next_frames(std::vector<cv::Mat>& frames, std::vector<bool>& success, int batch_size, int retry_count)
		{
			if (fp_instance != nullptr)
				return fp_instance->next_frames(frames, success, batch_size, retry_count);
			return std::vector<std::shared_future<void>>();
		}

		std::vector<std::shared_future<void>> FrameProvider::grab_frames(std::vector<cv::Mat>& frames, std::vector<bool>& success, int start_frame, int batch_size, int retry_count)
		{
			if (fp_instance != nullptr)
				return fp_instance->grab_frames(frames, success, start_frame, batch_size, retry_count);
			return std::vector<std::shared_future<void>>();
		}

		bool FrameProvider::set_frame(int frame_id)
		{
			if (fp_instance != nullptr)
				return fp_instance->set_frame(frame_id);
			return false;
		}

		int FrameProvider::current_frame()
		{
			if (fp_instance != nullptr)
				return fp_instance->current_frame();
			return 0;
		}

		int FrameProvider::frame_count()
		{
			if (fp_instance != nullptr)
				return fp_instance->frame_count();
			return 0;
		}

		cv::Size FrameProvider::frame_size()
		{
			if (fp_instance != nullptr)
				return fp_instance->frame_size();
			return cv::Size();
		}

		double FrameProvider::fps()
		{
			if (fp_instance != nullptr)
				return fp_instance->fps();
			return 0.0;
		}

		void FrameProvider::close()
		{
			if (fp_instance != nullptr)
				fp_instance->close();
		}

		bool FrameProvider::is_open()
		{
			if (fp_instance != nullptr)
				return fp_instance->is_open();
			return false;
		}

		std::string FrameProvider::get_fourcc_str()
		{
			if (fp_instance != nullptr)
				return fp_instance->get_fourcc_str();
			return std::string();
		}

		int FrameProvider::get_fourcc()
		{
			if (fp_instance != nullptr)
				return fp_instance->get_fourcc();
			return 0;
		}

	}

}