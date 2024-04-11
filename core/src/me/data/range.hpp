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

#include "bezier.hpp"

namespace me::data {
		
	class Range {
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
		BezierCurve curve;
	};
	
}