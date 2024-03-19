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

#include "../dnn/dnn.hpp"

namespace me {

	namespace tracking {

		/// <summary>
		/// Map alias for any potential named poses on a single given frame
		/// </summary>
		using PoseIDMap = std::unordered_map<std::string, dnn::Pose>;

		/// <summary>
		/// Map alias for all frames that contain any number of named poses
		/// </summary>
		using PoseFrameMap = std::unordered_map<int, PoseIDMap>;

		/// <summary>
		/// Map alias for any potential named bounding box detections on a single given frame
		/// </summary>
		using DetectionIDMap = std::unordered_map<std::string, dnn::Detection>;

		/// <summary>
		/// Map alias for all frames that contain any number of named bounding box detections
		/// </summary>
		using DetectionFrameMap = std::unordered_map<int, DetectionIDMap>;

        /// <summary>
        /// Map alias for any potential tags on a single given frame
        /// </summary>
        using TagIDMap = std::unordered_map<int, dnn::Tag>;

        /// <summary>
        /// Map alias for all frames that contain any number of tags
        /// </summary>
        using TagFrameMap = std::unordered_map<int, TagIDMap>;

        using TrackedPoints = std::vector<cv::Point2f>;

		struct TrackingData {
			PoseFrameMap poses;
			DetectionFrameMap detections;
            TagFrameMap tags;
            TrackedPoints to_points(bool reduce_boxes = true, bool reduce_tags = false);
		};

        /// <summary>
        /// Alias for a pair of related tracking data. Usually returned as a result of intersection operations.
        /// </summary>
        using TDataPair = std::pair<TrackingData, TrackingData>;

        /// <summary>
        /// Function used to extract keys from a map
        /// </summary>
        /// <param name="inputMap">The map to extract keys from</param>
        /// <returns>A vector of keys</returns>
        template<typename Map>
        auto extractKeys(const Map& inputMap) -> std::vector<typename Map::key_type> {
            std::vector<typename Map::key_type> keys;
            keys.reserve(inputMap.size());

            for (const auto& element : inputMap) {
                keys.push_back(element.first);
            }

            return keys;
        }

        /// <summary>
        /// Function used to find common keys from two maps. Returned list will be sorted in ascending order.
        /// </summary>
        /// <param name="map1">First map</param>
        /// <param name="map2">Second map</param>
        /// <returns>A vector of common keys</returns>
        template<typename Map>
        auto findCommonKeys(const Map& map1, const Map& map2) -> std::vector<typename Map::key_type> {
            auto keys1 = extractKeys(map1);
            auto keys2 = extractKeys(map2);

            std::vector<typename Map::key_type> commonKeys;

            // Ensure the key vectors are sorted before finding their intersection
            std::sort(keys1.begin(), keys1.end());
            std::sort(keys2.begin(), keys2.end());

            // Find the intersection
            auto it = std::set_intersection(keys1.begin(), keys1.end(), keys2.begin(), keys2.end(), std::back_inserter(commonKeys));

            return commonKeys;
        }

        /// <summary>
        /// Function used to construct new TrackingData structures that share common data names. Useful for preprocessing before stereoscopic calculations.
        /// Resulting data will be sorted.
        /// </summary>
        /// <param name="data_a">First source data tree</param>
        /// <param name="data_b">Second source data tree</param>
        /// <returns>A pair of data trees that share common names</returns>
        TDataPair find_common_data(TrackingData& data_a, TrackingData& data_b);

	}

}