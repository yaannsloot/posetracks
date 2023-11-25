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

#include "bezier.hpp"
#include "hermite.hpp"

namespace me {
	
	namespace data {
		
		size_t nCk(size_t n, size_t k) { return factorial(n) / (factorial(k) * factorial(n - k)); }

		cv::Point2d B(double t, std::vector<cv::Point2d*>& pts) {
			cv::Point2d result(0, 0);
			if (pts.size() > 0) {
				size_t n = pts.size() - 1;
				for (size_t i = 0; i <= n; i++) {
					result.x += nCk(n, i) * pow(1 - t, n - i) * pow(t, i) * pts[i]->x;
					result.y += nCk(n, i) * pow(1 - t, n - i) * pow(t, i) * pts[i]->y;
				}
			}
			return result;
		}

		cv::Point2d BezierCurve::solve(double x) {
			if (x < 0)
				x = 0;
			if (x > 1)
				x = 1;
			return B(x, this->getPoints());
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
	
}