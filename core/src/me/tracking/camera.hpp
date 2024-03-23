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

namespace me {

	namespace tracking {

		/// <summary>
		/// Solve the position of a camera relative to another using tracked point correspondences
		/// </summary>
		/// <param name="t_data_a">Tracking data from the origin camera</param>
		/// <param name="t_data_b">Tracking data from the target camera</param>
		/// <param name="cam_mtx_dist_a">Camera matrix and distortion data from the origin camera</param>
		/// <param name="cam_mtx_dist_b">Camera matrix and distortion data from the target camera</param>
		/// <returns></returns>
		Rt solveStaticPair(const TrackingData& t_data_a, const TrackingData& t_data_b, const Kk& cam_Kk_a, const Kk& cam_Kk_b);

		/// <summary>
		/// Solve the position of a camera relative to another using tracked point correspondences
		/// </summary>
		/// <param name="t_points_a">Tracked points from the origin camera</param>
		/// <param name="t_points_b">Tracked points from the target camera</param>
		/// <param name="cam_mtx_dist_a">Camera matrix and distortion data from the origin camera</param>
		/// <param name="cam_mtx_dist_b">Camera matrix and distortion data from the target camera</param>
		/// <returns></returns>
		Rt solveStaticPair(const TrackedPoints& t_points_a, const TrackedPoints& t_points_b, const Kk& cam_Kk_a, const Kk& cam_Kk_b);

		std::vector<Rt> solveStaticSet(const std::vector<TrackingData>& t_data, const std::vector<Kk>& cam_Kk);



	}



}