/*
me_utils.cpp
Source code for header me_utils.hpp

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

#include <me_utils.hpp>
#include <iostream>
#include <map>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudawarping.hpp>
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudacodec.hpp>

namespace me {
	namespace utility {

		std::vector<cv::Mat> SplitBatchesYOLO(cv::Mat& input_blob) {
			std::vector<cv::Mat> output;
			const int input_dim = input_blob.dims;
			const int output_rows = input_blob.size[1];
			const int output_cols = input_blob.size[2];
			int batch_size = 1;
			if (input_dim > 2)
				batch_size = input_blob.size[0];

			if (batch_size == 1)
				output.push_back(input_blob);
			else {
				for (int b = 0; b < batch_size; b++) {
					output.push_back(cv::Mat(output_rows, output_cols, CV_32F, input_blob.ptr<float>(b)));
				}
			}
			return output;
		}

		cv::Mat PackDetections(std::vector<Detection>& detections) {
			const auto num_det = detections.size();
			cv::Mat output(num_det, 6, CV_32F);
			for (int r = 0; r < num_det; r++) {
				*output.ptr<float>(r, 0) = (float)detections[r].class_id;
				*output.ptr<float>(r, 1) = (float)detections[r].bbox.tl().y;
				*output.ptr<float>(r, 2) = (float)detections[r].bbox.tl().x;
				*output.ptr<float>(r, 3) = (float)detections[r].bbox.br().y;
				*output.ptr<float>(r, 4) = (float)detections[r].bbox.br().x;
				*output.ptr<float>(r, 5) = detections[r].score;
			}
			return output;
		}

		std::vector<Detection> UnpackDetections(cv::Mat& input) {
			const auto num_det = input.rows;
			std::vector<Detection> output;
			for (int r = 0; r < num_det; r++) {
				Detection det;
				det.class_id = (int)*input.ptr<float>(r, 0);
				det.bbox = cv::Rect2d(
					cv::Point(
						*input.ptr<float>(r, 2),
						*input.ptr<float>(r, 1)
					),
					cv::Point(
						*input.ptr<float>(r, 4),
						*input.ptr<float>(r, 3)
					)
				);
				det.score = *input.ptr<float>(r, 5);
				output.push_back(det);
			}
			return output;
		}

		std::vector<std::vector<Detection>> PostprocessYOLO(std::vector<cv::Mat>& detection_mats, float conf_thresh, cv::Size img_size) {
			std::vector<std::vector<Detection>> output;
			for (cv::Mat& det_mat : detection_mats) {
				std::vector<Detection> sub_outs;
				const int num_det = det_mat.rows;
				const int num_classes = det_mat.cols - 5;
				for (int d = 0; d < num_det; d++) {
					float objectivity = *det_mat.ptr<float>(d, 4);
					if (objectivity > conf_thresh) {
						Detection det;
						det.score = objectivity;
						int id = 0;
						float confidence = 0;
						for (int c = 0; c < num_classes; c++) {
							float conf = *det_mat.ptr<float>(d, 5 + c);
							if (conf >= confidence) {
								id = c;
								confidence = conf;
							}
						}
						det.class_id = id;
						float x = *det_mat.ptr<float>(d, 0) * img_size.width;
						float y = *det_mat.ptr<float>(d, 1) * img_size.height;
						float width = *det_mat.ptr<float>(d, 2) * img_size.width;
						float height = *det_mat.ptr<float>(d, 3) * img_size.height;
						cv::Rect rect(x - width / 2, y - height / 2, width, height);
						det.bbox = rect;
						sub_outs.push_back(det);
					}
				}
				output.push_back(sub_outs);
			}
			return output;
		}

		cv::Mat DoNMSForBox(cv::Mat& input, float conf_thresh, float nms_thresh) {
			auto detections = UnpackDetections(input);
			std::vector<Detection> filtered_detections;
			std::unordered_map<int, std::vector<Detection>> class_map;
			for (me::utility::Detection& det : detections) {
				if (class_map.count(det.class_id) == 0)
					class_map[det.class_id] = std::vector<Detection>();
				class_map[det.class_id].push_back(det);
			}
			for (auto& class_pair : class_map) {
				std::vector<int> indices;
				std::vector<cv::Rect2d> boxes;
				std::vector<float> scores;
				for (auto& box : class_pair.second) {
					boxes.push_back(box.bbox);
					scores.push_back(box.score);
				}
				cv::dnn::NMSBoxes(boxes, scores, conf_thresh, nms_thresh, indices);
				for (int i : indices) {
					filtered_detections.push_back(class_pair.second[i]);
				}
			}
			return PackDetections(filtered_detections);
		}

		MovieReader::MovieReader(const std::string movie_path) {
			clip = cv::VideoCapture(movie_path, cv::CAP_FFMPEG, {cv::CAP_PROP_HW_ACCELERATION, cv::VIDEO_ACCELERATION_D3D11});
		}

		bool MovieReader::set_frame(int frame) {
			return clip.set(cv::CAP_PROP_POS_FRAMES, (double)frame);
		}

		int MovieReader::frame_count() {
			int frame_count = 0;
			if (clip.isOpened())
				frame_count = (int)clip.get(cv::CAP_PROP_FRAME_COUNT);
			return frame_count;
		}

		int MovieReader::current_frame() {
			int frame_num = 0;
			if (clip.isOpened())
				frame_num = (int)clip.get(cv::CAP_PROP_POS_FRAMES);
			return frame_num;
		}

		cv::Size MovieReader::frame_size() {
			cv::Size size;
			if (clip.isOpened()) {
				int frame_width = (int)clip.get(cv::CAP_PROP_FRAME_WIDTH);
				int frame_height = (int)clip.get(cv::CAP_PROP_FRAME_HEIGHT);
				size = cv::Size(frame_width, frame_height);
			}
			return size;
		}

		double MovieReader::get_fps() {
			double fps = 0;
			if(clip.isOpened())
				fps = clip.get(cv::CAP_PROP_FPS);
			return fps;
		}

		bool MovieReader::frame(cv::Mat& output) {
			if (clip.isOpened())
				return clip.read(output);
			return false;
		}

		bool MovieReader::grab_frame(int frame, cv::Mat& output) {
			set_frame(frame);
			return this->frame(output);
		}

		void MovieReader::close() {
			clip.release();
		}

		bool MovieReader::is_open() {
			return clip.isOpened();
		}

		std::string MovieReader::get_fourcc() {
			std::string output;
			if (clip.isOpened()) {
				int fourcc = static_cast<int>(clip.get(cv::CAP_PROP_FOURCC));
				char fourcc_str[] = {
					static_cast<char>(fourcc & 0XFF),
					static_cast<char>((fourcc & 0XFF00) >> 8),
					static_cast<char>((fourcc & 0XFF0000) >> 16),
					static_cast<char>((fourcc & 0XFF000000) >> 24),
					0
				};
				output = std::string(fourcc_str);
			}
			return output;
		}

		MovieReader::~MovieReader() {
			clip.release();
		}

		std::vector<float> LetterboxImage(const cv::Mat& src, cv::Mat& dst, const cv::Size& out_size) {
			auto in_h = static_cast<float>(src.rows);
			auto in_w = static_cast<float>(src.cols);
			float out_h = out_size.height;
			float out_w = out_size.width;

			float scale = std::min(out_w / in_w, out_h / in_h);

			int mid_h = static_cast<int>(in_h * scale);
			int mid_w = static_cast<int>(in_w * scale);

			cv::cuda::GpuMat gpuIn;
			cv::cuda::GpuMat gpuOut;

			gpuIn.upload(src);

			cv::cuda::resize(gpuIn, gpuOut, cv::Size(mid_w, mid_h));

			int top = (static_cast<int>(out_h) - mid_h) / 2;
			int down = (static_cast<int>(out_h) - mid_h + 1) / 2;
			int left = (static_cast<int>(out_w) - mid_w) / 2;
			int right = (static_cast<int>(out_w) - mid_w + 1) / 2;

			cv::cuda::copyMakeBorder(gpuOut, gpuOut, top, down, left, right, cv::BORDER_CONSTANT, cv::Scalar(114, 114, 114));

			gpuOut.download(dst);

			std::vector<float> pad_info{ static_cast<float>(left), static_cast<float>(top), scale };
			return pad_info;
		}

		std::string currentDateTime() {
			time_t     now = time(0);
			struct tm  tstruct;
			char       buf[80];
			localtime_s(&tstruct, &now);
			// Visit http://en.cppreference.com/w/cpp/chrono/c/strftime
			// for more information about date/time format
			strftime(buf, sizeof(buf), "%Y-%m-%d.%X", &tstruct);

			return buf;
		}

		cv::Mat DoNMSForYOLO(cv::Mat& input, float conf_thresh, float iou_thresh, cv::Size image_dim) {
			const int num_classes = input.cols - 5;
			cv::Mat out_mat(0, 6, CV_32FC1);
			std::vector<int> *indices = new std::vector<int>[num_classes];
			std::vector<float> *scores = new std::vector<float>[num_classes];
			std::vector<cv::Rect2d> *boxes = new std::vector<cv::Rect2d>[num_classes];

			try {
				const auto num_boxes = input.rows;
				for (int i = 0; i < num_boxes; i++) {
					auto objectivity = input.at<float>(i, 4);
					int id = 0;
					float confidence = 0;
					for (int c = 0; c < num_classes; c++) {
						auto conf = *input.ptr<float>(i, 5 + c);
						if (conf >= confidence) {
							id = c;
							confidence = conf;
						}
					}
					if (confidence >= conf_thresh) {
						auto x = input.at<float>(i, 0) * image_dim.width;
						auto y = input.at<float>(i, 1) * image_dim.height;
						auto width = input.at<float>(i, 2) * image_dim.width;
						auto height = input.at<float>(i, 3) * image_dim.height;
						cv::Rect rect(x - width / 2, y - height / 2, width, height);
						boxes[id].push_back(rect);
						scores[id].push_back(objectivity);
					}
				}

				for (int c = 0; c < num_classes; c++) {
					cv::dnn::NMSBoxes(boxes[c], scores[c], 0.0, iou_thresh, indices[c]);
					for (auto i : indices[c]) {
						cv::Mat out_row(1, 6, CV_32FC1);
						*out_row.ptr<float>(0, 0) = (float)c;
						*out_row.ptr<float>(0, 1) = (float)boxes[c][i].tl().y;
						*out_row.ptr<float>(0, 2) = (float)boxes[c][i].tl().x;
						*out_row.ptr<float>(0, 3) = (float)boxes[c][i].br().y;
						*out_row.ptr<float>(0, 4) = (float)boxes[c][i].br().x;
						*out_row.ptr<float>(0, 5) = scores[c][i];
						out_mat.push_back(out_row);
					}
				}
			}
			catch (std::exception& ex) {
				delete[] indices, scores, boxes;
				throw ex;
			}
			catch (cv::Exception& ex) {
				delete[] indices, scores, boxes;
				throw ex;
			}
			delete[] indices, scores, boxes;
			return out_mat;
		}

		void log(std::string input, std::string level) {
#ifdef WITH_CONSOLE
			FILE* pConsole;
			AllocConsole();
			freopen_s(&pConsole, "CONOUT$", "wb", stdout);
#endif
			std::cout << currentDateTime() << " [MEComputeLibdll][" << level << "]: " << input << std::endl;
		}

		size_t factorial(size_t n) {
			size_t f = 1;
			for (size_t i = 1; i <= n; i++) {
				f *= i;
			}
			return f;
		}

		namespace curves {

			Curve::Curve(std::vector<cv::Point2d> points) {
				addPoints(points);
			}

			Curve::Curve() {}

			cv::Point2d Curve::getPoint(int index) {
				int i = 0;
				for (auto it = points.begin(); it != points.end(); it++) {
					if (index == i)
						return *it;
					i++;
				}
				return cv::Point2d(0, 0);
			}

			std::vector<cv::Point2d*> Curve::getPoints() {
				std::vector<cv::Point2d*> pts;
				for (auto it = points.begin(); it != points.end(); it++) {
					pts.push_back(&it._Ptr->_Myval);
				}
				return pts;
			}

			size_t Curve::size() {
				return points.size();
			}

			void Curve::addPoint(cv::Point2d point) {
				points.emplace(point);
			}

			void Curve::addPoint(double x, double y) {
				points.emplace(cv::Point2d(x, y));
			}

			void Curve::addPoints(std::vector<cv::Point2d> points) {
				for (auto it = points.begin(); it != points.end(); it++) {
					this->points.emplace(*it);
				}
			}

			void Curve::removePoint(int index) {
				int i = 0;
				for (auto it = points.begin(); it != points.end(); it++) {
					if (index == i) {
						points.erase(it);
						return;
					}
					i++;
				}
			}

			void Curve::clearPoints() {
				points.clear();
			}

			cv::Point2d Curve::solve(double x) {
				return cv::Point2d(x, 0);
			}

			cv::Point2d CardinalCubicHermiteSpline::solve(double x) {
				return cv::Point2d(x, y(x));
			}

			void CardinalCubicHermiteSpline::setTension(double c) {
				if (c > 1)
					c = 1;
				if (c < 0)
					c = 0;
				this->c = c;
			}

			cv::Point2d CardinalCubicHermiteSpline::Pk(double x, int offset) {
				if (points.empty())
					return cv::Point2d(0, 0);
				auto it = points.begin();
				if (points.size() == 1)
					return *it;
				for (; std::next(it) != points.end(); it++) {
					if (it->x <= x && x < std::next(it)->x)
						break;
				}
				if (offset > 0) {
					for (int i = 0; i < offset && std::next(it) != points.end(); i++) {
						it++;
					}
				}
				if (offset < 0) {
					for (int i = 0; i < abs(offset) && it != points.begin(); i++) {
						it--;
					}
				}
				return *it;
			}

			cv::Point2d BezierCurve::solve(double x) {
				if (x < 0)
					x = 0;
				if (x > 1)
					x = 1;
				return B(x);
			}

			cv::Point2d BezierCurve::B(double t) {
				cv::Point2d result(0, 0);
				if (points.size() > 0) {
					size_t n = points.size() - 1;
					std::vector<cv::Point2d*> pts = getPoints();
					for (size_t i = 0; i <= n; i++) {
						result.x += nCk(n, i) * pow(1 - t, n - i) * pow(t, i) * pts[i]->x;
						result.y += nCk(n, i) * pow(1 - t, n - i) * pow(t, i) * pts[i]->y;
					}
				}
				return result;
			}

			double estimateT(BezierCurve& c, double x, size_t iterations) {
				CardinalCubicHermiteSpline spline;
				double offset = 1 / ((double)iterations - 1);
				for (size_t i = 0; i < iterations; i++) {
					spline.addPoint(cv::Point2d(c.solve(i * offset).x, i * offset));
				}
				return spline.solve(x).y;
			}

		}

		Range::Range() {
			curve.addPoint(0, 0);
			curve.addPoint(1, 0);
		}

		Range::Range(double min, double max) {
			curve.addPoint(0, min);
			curve.addPoint(1, max);
		}

		void Range::setMin(double min) {
			*curve.getPoints()[0] = cv::Point2d(0, min);
		}

		void Range::setMax(double max) {
			*curve.getPoints()[1] = cv::Point2d(1, max);
		}

		bool Range::contains(double val) {
			return (solve(0) <= val && val <= solve(1)) || (solve(1) <= val && val <= solve(0));
		}

		bool Range::checkForOverlap(Range& other) {
			return (other.solve(0) <= solve(0) && solve(0) <= other.solve(1)) ||
				(other.solve(0) <= solve(1) && solve(1) <= other.solve(1)) ||
				(solve(0) <= other.solve(0) && other.solve(0) <= solve(1)) ||
				(solve(0) <= other.solve(1) && other.solve(1) <= solve(1));
		}

		Range Range::getOverlapRange(Range& other) {
			Range overlap;
			if (checkForOverlap(other)) {
				double min = 0;
				double max = 0;
				if (other.solve(0) <= solve(0) && solve(0) <= other.solve(1))
					min = solve(0);
				else
					min = other.solve(0);
				if (other.solve(0) <= solve(1) && solve(1) <= other.solve(1))
					max = solve(1);
				else
					max = other.solve(1);
				overlap.setMin(min);
				overlap.setMax(max);
			}
			return overlap;
		}

		namespace sets {

			void TransformInterpolationSetHermite::addPos(double t, double x, double y, double z) {
				posX.addPoint(t, x);
				posY.addPoint(t, y);
				posZ.addPoint(t, z);
			}

			void TransformInterpolationSetHermite::addRot(double t, double x, double y, double z) {
				rotX.addPoint(t, x);
				rotY.addPoint(t, y);
				rotZ.addPoint(t, z);
			}

			cv::Point3d TransformInterpolationSetHermite::solvePos(double t) {
				return cv::Point3d(posX.solve(t).y, posY.solve(t).y, posZ.solve(t).y);
			}

			cv::Point3d TransformInterpolationSetHermite::solveRot(double t) {
				return cv::Point3d(rotX.solve(t).y, rotY.solve(t).y, rotZ.solve(t).y);
			}

			void PointInterpolationSetHermite::addPos(double t, double x, double y) {
				posX.addPoint(t, x);
				posY.addPoint(t, y);
			}

			cv::Point2d PointInterpolationSetHermite::solve(double t) {
				return cv::Point2d(posX.solve(t).y, posY.solve(t).y);
			}

		}

		bool exportArucoPdf(std::string out_file, PDFParams params, int marker_count, std::vector<int>& marker_list) {
			return false;
		}

		bool exportArucoPdf(std::string out_file, PDFParams params, int marker_count) {
			std::vector<int> dummy;
			return exportArucoPdf(out_file, params, marker_count, dummy);
		}

		const int dictStringToEnum(std::string _in) {
			for (int i = 0; i < 21; i++) {
				if (_in == dict_names[i])
					return i;
			}
			return -1;
		}

		const std::string dictEnumToString(int _in) {
			if (!(_in >= 0 && _in <= 21))
				throw std::runtime_error("enum index out of range");
			return dict_names[_in];
		}

	}
}