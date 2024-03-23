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

#include "hermite.hpp"

namespace me {
	
	namespace data {
		
		cv::Point2d Pk(double x, int offset, std::vector<cv::Point2d>& points) {
			if (points.empty())
				return cv::Point2d(0, 0);
			auto it = points.begin();
			if (points.size() == 1)
				return *it;
			for (; std::next(it) != points.end(); ++it) {
				if ((*it).x <= x && x < (*std::next(it)).x)
					break;
			}
			if (offset > 0) {
				for (int i = 0; i < offset && std::next(it) != points.end(); ++i) {
					++it;
				}
			}
			if (offset < 0) {
				for (int i = 0; i < abs(offset) && it != points.begin(); ++i) {
					--it;
				}
			}
			return *it;
		}

		// Hermite basis functions
		double h00(double t) { return 2 * pow(t, 3) - 3 * pow(t, 2) + 1; }
		double h10(double t) { return pow(t, 3) - 2 * pow(t, 2) + t; }
		double h01(double t) { return -2 * pow(t, 3) + 3 * pow(t, 2); }
		double h11(double t) { return pow(t, 3) - pow(t, 2); }

		// Additional spline functions
		double t0(double x, std::vector<cv::Point2d>& points) { return Pk(x, 1, points).x - Pk(x, 0, points).x; }
		double t(double x, std::vector<cv::Point2d>& points) { return (x - Pk(x, 0, points).x) / t0(x, points); }
		double mk(double x, double c, int offset, std::vector<cv::Point2d>& points) { return (1 - c) * ((Pk(x, 1 + offset, points).y - Pk(x, -1 + offset, points).y) / (Pk(x, 1 + offset, points).x - Pk(x, -1 + offset, points).x)); }
		double y(double x, double c, std::vector<cv::Point2d>& points) { return h00(t(x, points)) * Pk(x, 0, points).y + h10(t(x, points)) * t0(x, points) * mk(x, c, 0, points) + h01(t(x, points)) * Pk(x, 1, points).y + h11(t(x, points)) * t0(x, points) * mk(x, c, 1, points); }

		cv::Point2d CardinalCubicHermiteSpline::solve(double x) {
			return cv::Point2d(x, y(x, this->c, this->getPoints()));
		}

		void CardinalCubicHermiteSpline::setTension(double c) {
			if (c > 1)
				c = 1;
			if (c < 0)
				c = 0;
			this->c = c;
		}
		
	}
	
}