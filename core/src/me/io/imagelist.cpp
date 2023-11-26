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

#include "imagelist.hpp"
#include <filesystem>

namespace me {

	namespace io {

		// TODO: add multithreading support

		std::vector<std::string> image_sequence_resolve_all(const std::string& filepath) {
			std::filesystem::path path(filepath);
			std::string filename_noext = path.stem().string();
			std::string ext = path.extension().string();
			std::string filename_nodigits = filename_noext;

			// Remove trailing digits from the filename
			for (auto it = filename_noext.rbegin(); it != filename_noext.rend(); ++it) {
				if (isdigit(*it)) {
					filename_nodigits.pop_back();
				}
				else {
					break;
				}
			}

			if (filename_nodigits.size() == filename_noext.size()) {
				// Input isn't from a sequence
				return {};
			}

			std::vector<std::string> files;
			for (const auto& entry : std::filesystem::directory_iterator(path.parent_path())) {
				if (entry.is_regular_file() &&
					entry.path().filename().string().find(filename_nodigits) == 0 &&
					entry.path().extension().string() == ext) {
					files.push_back(entry.path().string());
				}
			}

			return files;
		}

		ImageListImpl::ImageListImpl()
		{
			if (!me::threading::global_pool.Running()) {
				me::threading::global_pool.Start();
			}
		}

		bool ImageListImpl::load(std::string path, bool use_hw_accel)
		{
			if (this->is_open())
				this->close();
			if (std::filesystem::exists(path)) {
				this->paths = image_sequence_resolve_all(path);
				cv::Mat first = cv::imread(paths[0]);
				if (first.empty()) {
					this->close();
					return false;
				}
				this->f_size.width = first.cols;
				this->f_size.height = first.rows;
				if (this->internal_pool == nullptr)
					this->internal_pool = std::make_shared<threading::SimplePool>();
				if (!this->internal_pool->Running())
					this->internal_pool->Start();
				return true;
			}
			return false;
		}

		cv::Mat get_image(std::string path, int retry_count) {
			cv::Mat frame;
			bool success = false;
			for (int i = 0; i < retry_count; ++i) {
				frame = cv::imread(path);
				success = !frame.empty();
				if (success)
					break;
			}
			std::cout << "[MotionEngine](" << std::this_thread::get_id() << ") Read image " << std::filesystem::path(path).stem().string() << std::endl;
			return frame;
		}

		bool ImageListImpl::next_frame(cv::Mat& frame, int retry_count)
		{
			if (!this->is_open())
				return false;
			if (this->f_index >= this->paths.size())
				return false;
			if(!future_queue.empty()) {
				while (!future_queue.empty()) {
					auto pair = future_queue.front();
					if (pair.first == this->f_index + 1)
						break;
					else if (pair.first == this->f_index) {
						try {
							frame = pair.second.get();
						} catch(...){}
						future_queue.pop();
						break;
					}
					else
						future_queue.pop();
				}
			}
			if (frame.empty())
				frame = get_image(this->paths[f_index], retry_count);
			bool success = !frame.empty();
			if (success)
				++this->f_index;

			// Top off the queue
			while (future_queue.size() < this->max_queue) {
				if (future_queue.empty()) {
					if (this->f_index >= this->paths.size())
						break;
					auto future = this->internal_pool->QueueJob(get_image, this->paths[f_index], retry_count).share();
					future_queue.push(std::make_pair(f_index, future));
				}
				else {
					auto pair = future_queue.back();
					int next = pair.first + 1;
					if (next >= this->paths.size())
						break;
					auto future = this->internal_pool->QueueJob(get_image, this->paths[next], retry_count).share();
					future_queue.push(std::make_pair(next, future));
				}
			}

			return success;
		}

		bool ImageListImpl::grab_frame(cv::Mat& frame, int frame_id, int retry_count)
		{
			set_frame(frame_id);
			return this->next_frame(frame, retry_count);
		}

		std::vector<std::shared_future<void>> ImageListImpl::next_frames(std::vector<cv::Mat>& frames, std::vector<bool>& success, int batch_size, int retry_count)
		{
			return std::vector<std::shared_future<void>>();
		}

		std::vector<std::shared_future<void>> ImageListImpl::grab_frames(std::vector<cv::Mat>& frames, std::vector<bool>& success, int start_frame, int batch_size, int retry_count)
		{
			return std::vector<std::shared_future<void>>();
		}

		bool ImageListImpl::set_frame(int frame_id)
		{
			if (frame_id < this->paths.size()) {
				this->f_index = frame_id;
				return true;
			}
			return false;
		}

		int ImageListImpl::current_frame()
		{
			return this->f_index;
		}

		int ImageListImpl::frame_count()
		{
			return paths.size();
		}

		cv::Size ImageListImpl::frame_size()
		{
			return this->f_size;
		}

		double ImageListImpl::fps()
		{
			return 0.0;
		}

		void ImageListImpl::close()
		{
			this->f_index = 0;
			this->paths.clear();
			this->f_size.width = 0;
			this->f_size.height = 0;
			while (!this->future_queue.empty()) {
				this->future_queue.pop();
			}
			this->internal_pool.reset();
		}

		bool ImageListImpl::is_open()
		{
			return this->paths.size() > 0;
		}

		std::string ImageListImpl::get_fourcc_str()
		{
			return "LIST";
		}

		int ImageListImpl::get_fourcc()
		{
			const std::string str = this->get_fourcc_str();
			int fourcc = 0;
			for (int i = 0; i < 4; ++i) {
				fourcc |= (str[i] << (i * 8));
			}
			return fourcc;
		}

		ImageList::ImageList() {
			this->fp_instance = std::make_shared<ImageListImpl>();
		}

	}

}