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

#include "frameprovider.hpp"

class TranscoderImpl : public FrameProviderImpl {
public:
	TranscoderImpl();
	~TranscoderImpl();
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
	cv::VideoCapture cap;
	std::string last_path;
};

/// <summary>
/// Frame provider instance that operates off of a movie file
/// </summary>
class Transcoder : public FrameProvider {
public:
	Transcoder();
};
