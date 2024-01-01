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

#include "curve.hpp"

namespace me {
	
	namespace data {
		
		size_t factorial(size_t n) {
			size_t f = 1;
			for (size_t i = 1; i <= n; i++) {
				f *= i;
			}
			return f;
		}
		
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

	}
	
}