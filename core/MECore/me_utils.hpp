/*
* 
*/

#ifdef MECORE_EXPORTS

#define MECORE __declspec(dllexport)

#else

#define MECORE __declspec(dllimport)

#endif

#include <string>
#include <vector>
#include <opencv2/aruco.hpp>
#include <set>
#include <stack>

#ifndef ME_UTILS_HPP
#define ME_UTILS_HPP

namespace me {
	namespace utility {

		MECORE std::string currentDateTime();

		MECORE void log(std::string input, std::string level = "INFO");

		MECORE size_t factorial(size_t n);

		namespace curves {

			// Base class
			// Solve has behavior similar to a polyline in which points are interpolated linearly
			class MECORE Curve {
			public:
				explicit Curve(std::vector<cv::Point2d> points) { addPoints(points); }
				explicit Curve() {}
				cv::Point2d getPoint(int index);
				std::vector<cv::Point2d*> getPoints();
				size_t size() { return points.size(); }
				void addPoint(cv::Point2d point);
				void addPoint(double x, double y);
				void addPoints(std::vector<cv::Point2d> points);
				void removePoint(int index);
				void clearPoints() { points.clear(); }
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
			class MECORE CardinalCubicHermiteSpline : public Curve {
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

			// Implements bezier curves, which are needed for 
			// Sources: https://en.wikipedia.org/wiki/B%C3%A9zier_curve, https://www.desmos.com/calculator/ebdtbxgbq0

			class MECORE BezierCurve : public Curve {
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

			MECORE double estimateT(BezierCurve& c, double x, size_t iterations = 10);

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

		class MECORE Range {
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

			class MECORE TransformInterpolationSetHermite {
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

			class MECORE PointInterpolationSetHermite {
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

		MECORE bool exportArucoPdf(std::string out_file, PDFParams params, int marker_count, std::vector<int>& marker_list);

		MECORE bool exportArucoPdf(std::string out_file, PDFParams params, int marker_count);

		/**
		Converts a string representation of an aruco dictionary enum value to its integer equivalent.
		To be used with OpenCV's aruco library
		*/
		MECORE const int dictStringToEnum(std::string _in);

		/**
		Converts a predefined aruco dictionary enum value to its string equivalent.
		To be used with OpenCV's aruco library
		*/
		MECORE const std::string dictEnumToString(int _in);

		class MECORE membuf : public std::basic_streambuf<char> {
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
