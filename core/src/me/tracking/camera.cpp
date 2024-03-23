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

#include "camera.hpp"
#include <algorithm>
#include <numeric>
#include <execution>

namespace me {

	namespace tracking {

		Rt solveStaticPair(const TrackingData& t_data_a, const TrackingData& t_data_b, const Kk& cam_Kk_a, const Kk& cam_Kk_b)
		{
			auto t_common = find_common_data(t_data_a, t_data_b);
			return solveStaticPair(t_common.first.to_points(), t_common.second.to_points(), cam_Kk_a, cam_Kk_b);
		}

		Rt solveStaticPair(const TrackedPoints& t_points_a, const TrackedPoints& t_points_b, const Kk& cam_Kk_a, const Kk& cam_Kk_b)
		{
			if (t_points_a.size() != t_points_b.size())
				throw std::invalid_argument("t_points_a.size() != t_points_b.size()");

			if (t_points_a.size() < 5)
				return Rt();

			TrackedPoints undis_points_a;
			TrackedPoints undis_points_b;

			cv::undistortPoints(t_points_a, undis_points_a, cam_Kk_a.K, cam_Kk_a.k);
			cv::undistortPoints(t_points_b, undis_points_b, cam_Kk_b.K, cam_Kk_b.k);

			cv::Mat mask;

			cv::Mat E = cv::findEssentialMat(undis_points_a, undis_points_b, cv::Mat::eye(3, 3, CV_64FC1), cv::RANSAC, 0.999, 0.0001, mask);

			cv::Mat R_relative;
			cv::Mat t_relative;

			cv::recoverPose(E, undis_points_a, undis_points_b, cv::Mat::eye(3, 3, CV_64FC1), R_relative, t_relative, mask);

			Rt final_T;

			final_T.R = R_relative;
			final_T.t = t_relative;

			return final_T;

		}

		std::vector<Rt> solveStaticSet(const std::vector<TrackingData>& t_data, const std::vector<Kk>& cam_Kk)
		{
			if(t_data.size() != cam_Kk.size())
				throw std::invalid_argument("t_data.size() != cam_Kk.size()");

			const size_t num_views = t_data.size();
			const size_t num_solutions = num_views - 1;

			std::vector<Rt> result;

			if (num_views > 1) {
				result.resize(num_solutions);
				if (num_views == 2) {
					result[0] = solveStaticPair(t_data[0], t_data[1], cam_Kk[0], cam_Kk[1]);
				}
				else {

					// FIRST PASS
					
					// Solve all cameras that have solutions with anchor
					std::vector<int> solved(num_solutions, 0);
					std::vector<size_t> targets(num_solutions);
					std::iota(targets.begin(), targets.end(), 0);
					std::for_each(std::execution::par_unseq, targets.begin(), targets.end(), [&](size_t i) {
						const size_t data_idx = i + 1;
						auto common_data = find_common_data(t_data[0], t_data[data_idx]);
						const auto anchor_points = common_data.first.to_points();
						const auto target_points = common_data.second.to_points();
						if (anchor_points.size() >= 5) {
							result[i] = solveStaticPair(anchor_points, target_points, cam_Kk[0], cam_Kk[data_idx]);
							solved[i] = 1;
						}
					});

					// Solve remaining using previous solutions
					int num_solved = std::accumulate(solved.begin(), solved.end(), 0);
					if (num_solved > 0 && num_solved < num_solutions) {

						// Prepare an evaluation table. This can be used to avoid repeat merge operations
						std::vector<std::vector<size_t>> eval_table;
						for (size_t i = 0; i < num_solutions; ++i) {
							std::vector<size_t> row(num_solutions, 0);
							row[i] = 1;
							eval_table[i] = row;
						}

						// Loop through the current solutions until either all solutions are found or no additional solutions can be obtained
						while (num_solved < num_solutions) {

							// Copy list of solved solutions to avoid parallel collisions
							auto last_solved = solved;

							// Loop through solution indices
							std::for_each(std::execution::par_unseq, targets.begin(), targets.end(), [&](size_t i) {
								if (last_solved[i] == 0) {
									
									// Original index for current view
									const size_t data_idx = i + 1; 

									// Vector for new merged data that hasn't already been evaluated
									std::vector<std::pair<TrackedPoints, TrackedPoints>> merged_data(num_solutions); 

									// Loop through targets to find new candidates
									std::for_each(std::execution::par_unseq, targets.begin(), targets.end(), [&](size_t j) {

										// Original index for anchor candidate
										const size_t other_idx = j + 1;

										// Ensure that candidate hasn't already been evaluated and has a valid transformation
										if (eval_table[i][j] == 0 && last_solved[j] == 1) {
											auto combined_data = find_common_data(t_data[other_idx], t_data[data_idx]);
											merged_data[j] = std::pair<TrackedPoints, TrackedPoints>(
												combined_data.first.to_points(), 
												combined_data.second.to_points());
											size_t num_common = merged_data[j].first.size();
											eval_table[i][j] = num_common;
										}
									});

									// Get target candidate information
									auto eval_choice = std::max_element(eval_table[i].begin(), eval_table[i].end());
									const size_t choice_idx = std::distance(eval_table[i].begin(), eval_choice);
									const size_t choice_data_idx = choice_idx + 1;
									const size_t eval_magnitude = *eval_choice;

									// If candidate meets requirements for solution, solve current view
									if (eval_magnitude >= 5) {

										// Get transforms of current view in chosen sub view space
										Rt sub_T = solveStaticPair(merged_data[choice_idx].first,
											merged_data[choice_idx].second,
											cam_Kk[choice_data_idx],
											cam_Kk[data_idx]);

										// Convert to anchor space
										result[i].from4x4(result[choice_idx].to4x4() * sub_T.to4x4());

										// Mark as solved
										solved[i] = 1;
									}
								}
							});
							
							// Early break condition
							int new_solve_count = std::accumulate(solved.begin(), solved.end(), 0);
							if (new_solve_count == num_solved)
								break;

							num_solved = new_solve_count;
						}
					}

				}
			}

			return result;
		}



	}

}