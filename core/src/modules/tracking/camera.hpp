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
	/// Solve the position of a camera relative to another using tracked point correspondences
	/// </summary>
	/// <param name="t_data_a">Tracking data from the origin camera</param>
	/// <param name="t_data_b">Tracking data from the target camera</param>
	/// <param name="cam_mtx_dist_a">Camera matrix and distortion data from the origin camera</param>
	/// <param name="cam_mtx_dist_b">Camera matrix and distortion data from the target camera</param>
	/// <returns>Transformation of the target camera relative to the origin camera</returns>
	Rt solveStaticPair(const TrackingData& t_data_a, const TrackingData& t_data_b, const Kk& cam_Kk_a, const Kk& cam_Kk_b);

	/// <summary>
	/// Solve the position of a camera relative to another using tracked point correspondences
	/// </summary>
	/// <param name="t_points_a">Tracked points from the origin camera</param>
	/// <param name="t_points_b">Tracked points from the target camera</param>
	/// <param name="cam_mtx_dist_a">Camera matrix and distortion data from the origin camera</param>
	/// <param name="cam_mtx_dist_b">Camera matrix and distortion data from the target camera</param>
	/// <returns>Transformation of the target camera relative to the origin camera</returns>
	Rt solveStaticPair(const TrackedPoints& t_points_a, const TrackedPoints& t_points_b, const Kk& cam_Kk_a, const Kk& cam_Kk_b);

	/// <summary>
	/// Solve the positions for a set of cameras relative to an anchor view using tracked point correspondences
	/// </summary>
	/// <param name="t_data">Vector of tracking data. The first element in the vector is treated as tracking data from the anchor</param>
	/// <param name="cam_Kk">Vector of camera intrinsic information. Must be the same size as t_data</param>
	/// <returns>A vector of transformations for each camera besides the first. 
	/// These transformations represent the transformation of each camera relative to the first camera</returns>
	std::vector<Rt> solveStaticSet(const std::vector<TrackingData>& t_data, const std::vector<Kk>& cam_Kk);

	/// <summary>
	/// Solve a camera's position using an observed tag as the origin
	/// </summary>
	/// <param name="observed_tag">The observed reference tag</param>
	/// <param name="cam_Kk">Camera matrix and distortion data from the target camera</param>
	/// <param name="square_length">Side length of the observed tag</param>
	/// <returns>The estimated 3D position of the camera relative to the observed tag</returns>
	Rt solveCameraWithTag(const me::dnn::Tag& observed_tag, const Kk& cam_Kk, const double square_length = 1.0);

	std::vector<Rt> fixOriginWithTag(const std::vector<Rt>& orig_cam_Rt, const std::vector<TrackingData>& t_data, const int origin_id, const int origin_frame, const double tag_scale = 1.0);

	Mat4x4 calculateProcrustesTransform(const std::vector<Rt>& orig_cam_Rt, const std::vector<dnn::Tag>& observed_tags);

}