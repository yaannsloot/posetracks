#include <pch.h>
#include <me_utils.hpp>
#include <iostream>

namespace me {
	namespace utility {

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