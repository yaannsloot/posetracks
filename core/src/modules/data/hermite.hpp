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

// Sources: https://en.wikipedia.org/wiki/Cubic_Hermite_spline, https://www.desmos.com/calculator/th2kg7mb1b

// Cardinal Cubic Hermite Spline implementation
class CardinalCubicHermiteSpline : public Curve {
public:
	using Curve::Curve;
	void setTension(double new_c);
	cv::Point2d solve(double x);
private:
	double c = 0.5; // Cardinal spline tension parameter. Changes the tangents in the spline. 1 yields all zero tangents and 0.5 yields a Catmull-Rom spline
};
	