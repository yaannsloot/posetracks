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

#include "range.hpp"

namespace me::data {
		
	Range::Range() {
		curve.addPoint(0, 0);
		curve.addPoint(1, 0);
	}

	Range::Range(double min, double max) {
		curve.addPoint(0, min);
		curve.addPoint(1, max);
	}

	void Range::setMin(double min) {
		auto points = curve.getPoints();
		points[0] = cv::Point2d(0, min);
		curve.clearPoints();
		curve.addPoints(points);
	}

	void Range::setMax(double max) {
		auto points = curve.getPoints();
		points[1] = cv::Point2d(1, max);
		curve.clearPoints();
		curve.addPoints(points);
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
	
}