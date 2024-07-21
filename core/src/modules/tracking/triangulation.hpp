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

#include "data.hpp"

namespace me::tracking {

	/// <summary>
	/// Triangulate points for a set of cameras. 
	/// The transformations of these cameras must be calculated ahead of time for the solver to work properly.
	/// </summary>
	/// <param name="t_data">Vector of tracking data for each camera</param>
	/// <param name="cam_Kk">Vector of camera intrinsic information. Must be the same size as t_data</param>
	/// <param name="cam_Rt">Vector of camera transformations. Must be the same size</param>
	/// <returns></returns>
	TrackingData3D triangulateStatic(const std::vector<TrackingData>& t_data, const std::vector<Kk>& cam_Kk, const std::vector<Rt>& cam_Rt);

}