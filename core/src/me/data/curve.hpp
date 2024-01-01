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

#include <opencv2/opencv.hpp>
#include <vector>

namespace me {
	
	namespace data {
		
		size_t factorial(size_t n);
		
		/**
		* @class Curve
		* @brief Base class for curves
		*
		* @remarks
		* This class is the base class for curves. It provides a common interface for all curves.
		* Functions like a polyline, where the x values are interpolated linearly between the points.
		*/
		class Curve {
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
		
	}
	
}