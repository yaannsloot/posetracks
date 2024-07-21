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

#include "curve.hpp"

namespace me::data {
		
	// Implements bezier curves
	// Sources: https://en.wikipedia.org/wiki/B%C3%A9zier_curve, https://www.desmos.com/calculator/ebdtbxgbq0

	class BezierCurve : public Curve {
	public:
		using Curve::Curve;
		// BEWARE: Beziers are driven by t and x means t in this context. t must be within the interval of [0, 1]
		cv::Point2d solve(double x);
	};

	double estimateT(BezierCurve& c, double x, size_t iterations = 10);
	
}