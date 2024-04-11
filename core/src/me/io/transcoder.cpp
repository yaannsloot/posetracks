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

#include "transcoder.hpp"
#include "../threading/simplepool.hpp"

namespace me::io {

	TranscoderImpl::TranscoderImpl() {
		if (!me::threading::global_pool.Running()) {
			me::threading::global_pool.Start();
		}
	}

	TranscoderImpl::~TranscoderImpl() {
		last_path = "";
		close();
	}

	bool TranscoderImpl::load(std::string path, bool use_hw_accel) {
		if (cap.isOpened())
			cap.release();
		bool success = false;
		if (use_hw_accel)
			success = cap.open(path, cv::CAP_FFMPEG, { cv::CAP_PROP_HW_ACCELERATION, cv::VIDEO_ACCELERATION_ANY });
		else
			success = cap.open(path, cv::CAP_FFMPEG);
		if (success)
			last_path = path;
		return success;
	}

	bool TranscoderImpl::next_frame(cv::Mat& frame, int retry_count) {
		if (cap.isOpened()) {
			bool success = cap.read(frame);
			for (int i = 0; i < retry_count; i++) {
				if (!success)
					success = cap.read(frame);
				else
					break;
			}
			return success;
		}
		return false;
	}

	bool TranscoderImpl::grab_frame(cv::Mat& frame, int frame_id, int retry_count) {
		set_frame(frame_id);
		return this->next_frame(frame, retry_count);
	}

	std::vector<std::shared_future<void>> TranscoderImpl::next_frames(std::vector<cv::Mat>& frames, std::vector<bool>& success, int batch_size, int retry_count) {
		return grab_frames(frames, success, this->current_frame(), batch_size, retry_count);
	}

	std::vector<std::shared_future<void>> TranscoderImpl::grab_frames(std::vector<cv::Mat>& frames, std::vector<bool>& success, int start_frame, int batch_size, int retry_count) {

		std::vector<std::shared_future<void>> tasks;

		// CONSTANTS
		const int current_frame = start_frame;
		const int total_frames = this->frame_count();

		// GUARDS
		if (current_frame >= total_frames)
			return tasks;

		// PRE-PROCESS OPERATIONS
		const int diff = total_frames - current_frame;
		if (diff < batch_size)
			batch_size = diff;

		success.resize(batch_size);
		frames.resize(batch_size);

		auto splits = me::threading::calculateSegments((size_t)batch_size, (size_t)std::thread::hardware_concurrency());

		// MAIN TASK LOOP
		for (auto& split : splits) {
			auto task = me::threading::global_pool.QueueJob([](std::pair<size_t, size_t>* split,
				int offset,
				int retry_count,
				std::vector<cv::Mat>* frames,
				std::vector<bool>* success,
				std::string path) {

					int start_index = offset + split->first;

					cv::VideoCapture local_cap(path);

					if (!local_cap.isOpened())
						return;

					local_cap.set(cv::CAP_PROP_POS_FRAMES, start_index);

					for (size_t f = split->first; f < split->second; ++f) {
						bool s = local_cap.read(frames->at(f));
						for (int c = 0; c < retry_count; ++c) {
							if (!s)
								s = local_cap.read(frames->at(f));
							else
								break;
						}
						success->at(f) = s;
					}
					local_cap.release();

				}, &split, current_frame, retry_count, &frames, &success, last_path);

			tasks.push_back(task.share());

		}

		int destination_frame = current_frame + batch_size;
		cap.set(cv::CAP_PROP_POS_FRAMES, destination_frame);

		return tasks;

	}

	bool TranscoderImpl::set_frame(int frame_id) {
		return cap.set(cv::CAP_PROP_POS_FRAMES, (double)frame_id);
	}

	int TranscoderImpl::current_frame() {
		if (cap.isOpened())
			return (int)cap.get(cv::CAP_PROP_POS_FRAMES);
		return -1;
	}

	int TranscoderImpl::frame_count() {
		if (cap.isOpened())
			return (int)cap.get(cv::CAP_PROP_FRAME_COUNT);
		return -1;
	}

	cv::Size TranscoderImpl::frame_size() {
		if (cap.isOpened()) {
			int frame_width = (int)cap.get(cv::CAP_PROP_FRAME_WIDTH);
			int frame_height = (int)cap.get(cv::CAP_PROP_FRAME_HEIGHT);
			return cv::Size(frame_width, frame_height);
		}
		return cv::Size(0, 0);
	}

	double TranscoderImpl::fps() {
		if (cap.isOpened())
			return cap.get(cv::CAP_PROP_FPS);
		return -1;
	}

	void TranscoderImpl::close() {
		cap.release();
	}

	bool TranscoderImpl::is_open() {
		return cap.isOpened();
	}

	std::string TranscoderImpl::get_fourcc_str() {
		if (cap.isOpened()) {
			int fourcc = static_cast<int>(cap.get(cv::CAP_PROP_FOURCC));
			char fourcc_str[] = {
				static_cast<char>(fourcc & 0XFF),
				static_cast<char>((fourcc & 0XFF00) >> 8),
				static_cast<char>((fourcc & 0XFF0000) >> 16),
				static_cast<char>((fourcc & 0XFF000000) >> 24),
				0
			};
			return std::string(fourcc_str);
		}
		return "";
	}

	int TranscoderImpl::get_fourcc() {
		if (cap.isOpened())
			return (int)cap.get(cv::CAP_PROP_FOURCC);
		return -1;
	}

	Transcoder::Transcoder() {
		fp_instance = std::make_shared<TranscoderImpl>();
	}

}