/*
me_utils.hpp
Includes utility functions and classes used elsewhere in the project

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

#ifdef MEUTILS_EXPORTS

#define MEUTILS __declspec(dllexport)

#else

#define MEUTILS __declspec(dllimport)

#endif

#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <set>
#include <stack>

#ifndef ME_UTILS_HPP
#define ME_UTILS_HPP

namespace me {
	namespace utility {

		// OpenCV DNN helper functions

		/**
		* @brief Holds bounding box information for a detected object
		*/
		struct Detection {
			int class_id = 0;
			cv::Rect2d bbox;
			float score = 0;
		};

		/**
		* @brief Split a blob into a vector of cv::Mat objects.
		*
		* @remarks
		* This function splits a blob into a vector of cv::Mat objects. This is useful for splitting a batched blob into
		* individual images.
		*
		* @param cv::Mat input_blob: The blob to split.
		* @return std::vector<cv::Mat> output: The vector of cv::Mat objects.
		*/
		MEUTILS std::vector<cv::Mat> SplitBatchesYOLO(cv::Mat& input_blob);

		/**
		* @brief Pack detections into a single cv::Mat
		*
		* @remarks
		* Each row of the output matrix is a detection, with the following format:
		* [class_id, top_left_y, top_left_x, bottom_right_y, bottom_right_x, score]
		*
		* @param std::vector<Detection>& detections: detections to pack
		* @return cv::Mat: packed detections
		*/
		MEUTILS cv::Mat PackDetections(std::vector<Detection>& detections);

		/**
		* @brief Unpacks a cv::Mat of detections into a vector of Detection objects
		*
		* @remarks
		* The input cv::Mat must have 6 columns, where the first column is the class ID,
		* the second column is the top-left y coordinate, the third column is the top-left
		* x coordinate, the fourth column is the bottom-right y coordinate, the fifth column
		* is the bottom-right x coordinate, and the sixth column is the confidence score.
		*
		* @param cv::Mat& input: The input cv::Mat of detections
		* @return std::vector<Detection>: The vector of Detection objects
		*/
		MEUTILS std::vector<Detection> UnpackDetections(cv::Mat& input);

		/**
		* @brief Postprocess the output of YOLOv3, YOLOv3-tiny, YOLOv4, YOLOv4-tiny, etc.
		*
		* @remarks
		* The output of YOLOv3, YOLOv3-tiny, YOLOv4, YOLOv4-tiny, etc. is a 2D matrix with shape (N, 5 + C),
		* where N is the number of detections, C is the number of classes, and 5 is the number of columns used
		* to store the bounding box information (x, y, w, h, objectivity).
		*
		* @param std::vector<cv::Mat>& detection_mats: The output of YOLOv3, YOLOv3-tiny, YOLOv4, YOLOv4-tiny, etc.
		* @param float conf_thresh: The threshold of objectivity.
		* @param cv::Size img_size: The size of the input image.
		* @return std::vector<std::vector<Detection>>: The filtered detections.
		*/
		MEUTILS std::vector<std::vector<Detection>> PostprocessYOLO(std::vector<cv::Mat>& detection_mats, float conf_thresh, cv::Size img_size);

		/**
		* @brief Performs non-maximum suppression on a set of detections
		*
		* @remarks
		* This function uses the standard non-maximum suppression algorithm included in OpenCV.
		* The input is a matrix of detections, where each row is a detection and each column is
		* a value. The columns are as follows:
		* 0: class id
		* 1: top left y
		* 2: top left x
		* 3: bottom right y
		* 4: bottom right x
		* 5: confidence
		*
		* @param cv::Mat& input: The input matrix of detections
		* @param float conf_thresh: The confidence threshold to use
		* @param float nms_thresh: The non-maximum suppression threshold to use
		* @return cv::Mat: The matrix of detections after non-maximum suppression
		*/
		MEUTILS cv::Mat DoNMSForBox(cv::Mat& input, float conf_thresh, float nms_thresh);

		/**
		* @class MovieReader
		* @brief A class for reading movie clips using OpenCV's VideoCapture class
		* 
		* @remarks
		* This class is a wrapper for OpenCV's VideoCapture class. It provides a simple interface for reading movie clips.
		* It prioritizes hardware acceleration. If hardware acceleration is not available, it will fall back to software
		*/
		class MEUTILS MovieReader {

		public:
			/**
			* @brief Constructs a MovieReader object from a movie file.
			* 
			* @param std::string movie_path: The path to the movie file.
			*/
			MovieReader(const std::string movie_path);

			/**
			* @brief Sets the current frame of the movie clip.
			* 
			* @param int frame: The frame to jump to.
			*/
			bool set_frame(int frame);

			/**
			* @brief Gets the total number of frames in the movie clip.
			* 
			* @return int: The total number of frames in the movie clip.
			*/
			int frame_count();

			/**
			* @brief Gets the current frame of the movie clip.
			* 
			* @return int: The current frame of the movie clip.
			*/
			int current_frame();

			/**
			* @brief Gets the frame size of the movie clip.
			* 
			* @return cv::Size: The frame size of the movie clip.
			*/
			cv::Size frame_size();

			/**
			* @brief Gets the frame rate of the movie clip.
			* 
			* @return double: The frame rate of the movie clip.
			*/
			double get_fps();

			/**
			* @brief Gets the next frame of the movie clip.
			* 
			* @param cv::Mat& output: The output frame.
			* @return bool: True if the frame was successfully read, false otherwise.
			*/
			bool frame(cv::Mat& output);

			/**
			* @brief Gets the next frame of the movie clip.
			* 
			* @param int frame: The frame to grab.
			* @param cv::Mat& output: The output frame.
			* @return bool: True if the frame was successfully read, false otherwise.
			*/
			bool grab_frame(int frame, cv::Mat& output);

			/**
			* @brief Closes the movie clip.
			*/
			void close();

			/**
			* @brief Checks if the movie clip is open.
			* 
			* @return bool: True if the movie clip is open, false otherwise.
			*/
			bool is_open();

			/**
			* @brief Gets the fourcc code of the movie clip.
			*
			* @return std::string: The fourcc code of the movie clip.
			*/
			std::string get_fourcc();
			~MovieReader();

		private:
			cv::VideoCapture clip;

		};

		/**
		* @brief Letterbox an image to a given size.
		*
		* @remarks
		* This function is used to letterbox an image to a given size. Extra space is filled with black pixels.
		*
		* @param const cv::Mat& src: The source image.
		* @param cv::Mat& dst: The destination image.
		* @param const cv::Size& out_size: The size of the output image.
		* @returns std::vector<float>: Padding information.
		*/
		MEUTILS std::vector<float> LetterboxImage(const cv::Mat& src, cv::Mat& dst, const cv::Size& out_size);

		/**
		* @brief Get current date/time, format is YYYY-MM-DD.HH:mm:ss
		*
		* @return std::string: Formatted string containing current date/time
		*/
		MEUTILS std::string currentDateTime();

		/**
		* @brief Do non-maximum suppression algorithm for YOLO
		*
		* @remarks
		* Perform non-maximum suppression algorithm for YOLO. The input matrix should be a 2D matrix with 5 + num_classes columns.
		* The first 4 columns are the coordinates of the bounding box, the 5th column is the objectivity, and the rest are the
		* confidence scores for each class. The output matrix will be a 2D matrix with 6 columns. The first column is the class
		* id, the second to the fifth columns are the coordinates of the bounding box, and the last column is the confidence score.
		*
		* @param cv::Mat& input: The input matrix
		* @param float conf_thresh: The confidence threshold
		* @param float iou_thresh: The IoU threshold
		* @param cv::Size image_dim: The size of the image
		* @return cv::Mat: The output matrix
		*/
		MEUTILS cv::Mat DoNMSForYOLO(cv::Mat& input, float conf_thresh, float iou_thresh, cv::Size image_dim);

		/**
		* @brief Log a message to the console
		*
		* @remarks
		* Old code from the original MEComputeLibdll project. Would allocate a new console window and log to it
		* when running from unity.
		*
		* @param std::string input: The message to log
		* @param std::string level: The level of the message (e.g. INFO, WARNING, ERROR)
		*/
		MEUTILS void log(std::string input, std::string level = "INFO");

		/**
		* @brief Compute the factorial of a number
		*
		* @param size_t n: Number to compute the factorial
		*/
		MEUTILS size_t factorial(size_t n);

		namespace curves {

			/**
			* @class Curve
			* @brief Base class for curves
			* 
			* @remarks
			* This class is the base class for curves. It provides a common interface for all curves.
			* Functions like a polyline, where the x values are interpolated linearly between the points.
			*/
			class MEUTILS Curve {
			public:

				/**
				* @brief Constructor for the Curve class.
				* 
				* @param std::vector<cv::Point2d> points: The points of the curve.
				*/
				explicit Curve(std::vector<cv::Point2d> points);

				/**
				* @brief Constructor for the Curve class.
				*/
				explicit Curve();

				/**
				* @brief Get the point at the given index.
				* If the index is out of range, the function returns (0, 0)
				*
				* @param int index: The index of the point
				*/
				cv::Point2d getPoint(int index);


				std::vector<cv::Point2d*> getPoints();


				size_t size();


				void addPoint(cv::Point2d point);


				void addPoint(double x, double y);


				void addPoints(std::vector<cv::Point2d> points);


				void removePoint(int index);


				void clearPoints();


				virtual cv::Point2d solve(double x);

			protected:
				// Internal comparison function
				struct keycompare {
					bool operator() (const cv::Point2d& a, const cv::Point2d& b) const
					{
						return a.x < b.x;
					}
				};
				// Internal storage variables
				std::set<cv::Point2d, keycompare> points;
			};

			// BULLSHIT MATH FOR INTERPOLATION WHICH I NEED BC NONE OF THE FRAME TIMES ARE GOING TO LINE UP BUT WHATEVER
			// Sources: https://en.wikipedia.org/wiki/Cubic_Hermite_spline, https://www.desmos.com/calculator/th2kg7mb1b

			// Cardinal Cubic Hermite Spline implementation
			class MEUTILS CardinalCubicHermiteSpline : public Curve {
			public:
				using Curve::Curve;
				void setTension(double c);
				cv::Point2d solve(double x);
			private:
				// Hermite basis functions
				double h00(double t) { return 2 * pow(t, 3) - 3 * pow(t, 2) + 1; }
				double h10(double t) { return pow(t, 3) - 2 * pow(t, 2) + t; }
				double h01(double t) { return -2 * pow(t, 3) + 3 * pow(t, 2); }
				double h11(double t) { return pow(t, 3) - pow(t, 2); }

				// Additional spline functions
				double t0(double x) { return Pk(x, 1).x - Pk(x, 0).x; }
				double t(double x) { return (x - Pk(x, 0).x) / t0(x); }
				double mk(double x, double c, int offset) { return (1 - c) * ((Pk(x, 1 + offset).y - Pk(x, -1 + offset).y) / (Pk(x, 1 + offset).x - Pk(x, -1 + offset).x)); }
				double y(double x) { return h00(t(x)) * Pk(x, 0).y + h10(t(x)) * t0(x) * mk(x, c, 0) + h01(t(x)) * Pk(x, 1).y + h11(t(x)) * t0(x) * mk(x, c, 1); }
				cv::Point2d Pk(double x, int offset);

				double c = 0.5; // Cardinal spline tension parameter. Changes the tangents in the spline. 1 yields all zero tangents and 0.5 yields a Catmull-Rom spline
			};

			// Implements bezier curves
			// Sources: https://en.wikipedia.org/wiki/B%C3%A9zier_curve, https://www.desmos.com/calculator/ebdtbxgbq0

			class MEUTILS BezierCurve : public Curve {
			public:
				using Curve::Curve;
				// BEWARE: Beziers are driven by t and x means t in this context. t must be within the interval of [0, 1]
				cv::Point2d solve(double x);
			private:
				// Internal evaluation function
				cv::Point2d B(double t);

				// Helper functions
				size_t nCk(size_t n, size_t k) { return factorial(n) / (factorial(k) * factorial(n - k)); }
			};

			MEUTILS double estimateT(BezierCurve& c, double x, size_t iterations = 10);

		}

		template <typename T>
		class Edge {
		public:
			Edge(const T& a, const T& b) { this->a = a; this->b = b; };
			T getA() const { return a; }
			T getB() const { return b; }
		private:
			T a;
			T b;
		};

		template <typename T>
		bool operator==(const Edge<T>& a, const Edge<T>& b) {
			return (a.getA() == b.getA() && a.getB() == b.getB()) || (a.getA() == b.getB() && a.getB() == b.getA());
		}

		template <typename T>
		bool operator!=(const Edge<T>& a, const Edge<T>& b) {
			return !(a == b);
		}

		template <typename T>
		bool operator< (const Edge<T>& a, const Edge<T>& b) {
			return (a.getA() < b.getA() || a.getB() < b.getB() || a.getA() < b.getB() || a.getB() < b.getA()) && a != b;
		}

		template <typename T>
		class Graph {
		public:
			std::set<T> getVerticies();
			bool checkContinuity();
			std::stack<T> getVertexPathDFS(const T& a, const T& b);
			std::set<Edge<T>> getEdges();
			std::set<T> getAdjacentVerticies(const T& v);
			void addEdge(const T& a, const T& b);
			void addEdge(const Edge<T>& e);
			void removeEdge(const T& a, const T& b);
			void removeEdge(const Edge<T>& e);
			bool empty();
		private:
			std::set<Edge<T>> edges;
			void recursive_traverse(std::set<T>& visited, const T& current);
			void dfs(std::set<T>& visited, const T& current, const T& target, std::stack<T>& path, bool& found);
		};

		template <typename T>
		std::set<T> Graph<T>::getVerticies() {
			std::set<T> result;
			for (auto it = edges.begin(); it != edges.end(); it++) {
				result.emplace(it->getA());
				result.emplace(it->getB());
			}
			return result;
		}

		template <typename T>
		bool Graph<T>::checkContinuity() {
			if (!edges.empty()) {
				std::set<T> verts = getVerticies();
				std::set<T> reachable;
				T first = *verts.begin();
				recursive_traverse(reachable, first);
				if (reachable.size() == verts.size())
					return true;
			}
			return false;
		}

		template <typename T>
		std::stack<T> Graph<T>::getVertexPathDFS(const T& a, const T& b) {
			std::stack<T> result;
			if (!edges.empty()) {
				std::set<T> visited;
				bool found = false;
				dfs(visited, a, b, result, found);
			}
			return result;
		}

		template <typename T>
		std::set<Edge<T>> Graph<T>::getEdges() {
			return edges;
		}

		template <typename T>
		std::set<T> Graph<T>::getAdjacentVerticies(const T& v) {
			std::set<T> result;
			for (auto it = edges.begin(); it != edges.end(); it++) {
				if ((v == it->getA() || v == it->getB()) && it->getA() != it->getB()) {
					if (v == it->getA())
						result.emplace(it->getB());
					else
						result.emplace(it->getA());
				}
			}
			return result;
		}

		template <typename T>
		void Graph<T>::addEdge(const T& a, const T& b) {
			edges.emplace(Edge<T>(a, b));
		}

		template <typename T>
		void Graph<T>::addEdge(const Edge<T>& e) {
			edges.emplace(e);
		}

		template <typename T>
		void Graph<T>::removeEdge(const T& a, const T& b) {
			edges.erase(std::remove(edges.begin(), edges.end(), Edge<T>(a, b)));
		}

		template <typename T>
		void Graph<T>::removeEdge(const Edge<T>& e) {
			edges.erase(std::remove(edges.begin(), edges.end(), e));
		}

		template <typename T>
		bool Graph<T>::empty() {
			return edges.empty();
		}

		template <typename T>
		void Graph<T>::recursive_traverse(std::set<T>& visited, const T& current) {
			visited.emplace(current);
			std::set<T> adjacent = getAdjacentVerticies(current);
			for (auto it = adjacent.begin(); it != adjacent.end(); it++) {
				if (std::find(visited.begin(), visited.end(), *it) == visited.end())
					recursive_traverse(visited, *it);
			}
		}

		template <typename T>
		void Graph<T>::dfs(std::set<T>& visited, const T& current, const T& target, std::stack<T>& path, bool& found) {
			visited.emplace(current);
			if (current == target) {
				found = true;
			}
			if (!found) {
				std::set<T> adjacent = getAdjacentVerticies(current);
				for (auto it = adjacent.begin(); it != adjacent.end(); it++) {
					if (std::find(visited.begin(), visited.end(), *it) == visited.end())
						dfs(visited, *it, target, path, found);
					if (found)
						break;
				}
			}
			if (found)
				path.push(current);
		}

		class MEUTILS Range {
		public:
			Range();
			Range(double min, double max);
			double solve(double t) { return curve.solve(t).y; }
			void setMin(double min);
			void setMax(double max);
			bool contains(double val);
			bool checkForOverlap(Range& other);
			Range getOverlapRange(Range& other);
		private:
			curves::BezierCurve curve;
		};

		namespace sets {

			class MEUTILS TransformInterpolationSetHermite {
			public:
				void addPos(double t, double x, double y, double z);
				void addRot(double t, double x, double y, double z);
				cv::Point3d solvePos(double t);
				cv::Point3d solveRot(double t);
				curves::CardinalCubicHermiteSpline* curvePosX() { return &posX; };
				curves::CardinalCubicHermiteSpline* curvePosY() { return &posY; };
				curves::CardinalCubicHermiteSpline* curvePosZ() { return &posZ; };
				curves::CardinalCubicHermiteSpline* curveRotX() { return &rotX; };
				curves::CardinalCubicHermiteSpline* curveRotY() { return &rotY; };
				curves::CardinalCubicHermiteSpline* curveRotZ() { return &rotZ; };
			private:
				curves::CardinalCubicHermiteSpline posX;
				curves::CardinalCubicHermiteSpline posY;
				curves::CardinalCubicHermiteSpline posZ;
				curves::CardinalCubicHermiteSpline rotX;
				curves::CardinalCubicHermiteSpline rotY;
				curves::CardinalCubicHermiteSpline rotZ;
			};

			class MEUTILS PointInterpolationSetHermite {
			public:
				void addPos(double t, double x, double y);
				cv::Point2d solve(double t);
				curves::CardinalCubicHermiteSpline* curvePosX() { return &posX; };
				curves::CardinalCubicHermiteSpline* curvePosY() { return &posY; };
			private:
				curves::CardinalCubicHermiteSpline posX;
				curves::CardinalCubicHermiteSpline posY;
			};

		}

		// DICTIONARY STRING DEFINITIONS

		// ARUCO
		const std::string dict_names[21] = { "DICT_4X4_50", "DICT_4X4_100", "DICT_4X4_250", "DICT_4X4_1000",
					"DICT_5X5_50", "DICT_5X5_100", "DICT_5X5_250", "DICT_5X5_1000",
					"DICT_6X6_50", "DICT_6X6_100", "DICT_6X6_250", "DICT_6X6_1000",
					"DICT_7X7_50", "DICT_7X7_100", "DICT_7X7_250", "DICT_7X7_1000",
					"DICT_ARUCO_ORIGINAL", "DICT_APRILTAG_16h5", "DICT_APRILTAG_25h9",
					"DICT_APRILTAG_36h10", "DICT_APRILTAG_36h11" };


		struct PDFParams {
			int pdf_dpi = 72;
			float pdf_dim_x = 8.5f;
			float pdf_dim_y = 11.0f;
			float marker_dim = 2.0f;
			float corner_offset = 0.3f;
			float min_distance_x = 0.3f;
			float min_distance_y = 0.3f;
			float max_horiz_label_scale = 0.8f;
			float max_vert_label_scale = 0.8f;
			float label_height = 0.3f;
			float title_scale = 0.6f;
		};

		MEUTILS bool exportArucoPdf(std::string out_file, PDFParams params, int marker_count, std::vector<int>& marker_list);

		MEUTILS bool exportArucoPdf(std::string out_file, PDFParams params, int marker_count);

		/**
		Converts a string representation of an aruco dictionary enum value to its integer equivalent.
		To be used with OpenCV's aruco library
		*/
		MEUTILS const int dictStringToEnum(std::string _in);

		/**
		Converts a predefined aruco dictionary enum value to its string equivalent.
		To be used with OpenCV's aruco library
		*/
		MEUTILS const std::string dictEnumToString(int _in);

		class MEUTILS membuf : public std::basic_streambuf<char> {
		public:
			membuf(const uint8_t* p, size_t l) {
				setg((char*)p, (char*)p, (char*)p + l);
			}
		};

		class memstream : public std::istream {
		public:
			memstream(const uint8_t* p, size_t l) :
				std::istream(&_buffer),
				_buffer(p, l) {
				rdbuf(&_buffer);
			}

		private:
			membuf _buffer;
		};

	}
}

#endif
