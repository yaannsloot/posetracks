/*
Copyright (C) 2023 Ian Sloat

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

#include "stats.hpp"
#include <numeric>
#include <algorithm>
#include <cmath>

namespace me {
	
	namespace data {
		
		double calculateMean(const std::vector<double>& data) {
			return std::accumulate(data.begin(), data.end(), 0.0) / data.size();
		}

		double calculateMedian(const std::vector<double>& data) {
			if (data.empty()) return 0;

			std::vector<double> sortedData = data;
			std::sort(sortedData.begin(), sortedData.end());

			size_t n = sortedData.size();
			if (n % 2 == 0) { // even number of elements
				return (sortedData[n / 2 - 1] + sortedData[n / 2]) / 2;
			}
			else { // odd number of elements
				return sortedData[n / 2];
			}
		}

		double calculateStdDev(const std::vector<double>& data, double mean) {
			double sq_sum = std::inner_product(data.begin(), data.end(), data.begin(), 0.0);
			return std::sqrt(sq_sum / data.size() - mean * mean);
		}

		std::vector<size_t> findOutliers(const std::vector<double>& data, double threshold) {
			double mean = calculateMean(data);
			double std_dev = calculateStdDev(data, mean);

			std::vector<size_t> outlier_indices;
			for (size_t i = 0; i < data.size(); ++i) {
				double z_score = (data[i] - mean) / std_dev;
				if (std::abs(z_score) > threshold) {
					outlier_indices.push_back(i);
				}
			}

			return outlier_indices;
		}

		// Function to calculate Q1 and Q3
		void calculateQuartiles(const std::vector<double>& data, double& Q1, double& Q3) {
			std::vector<double> copy(data);
			std::sort(copy.begin(), copy.end());

			size_t n = copy.size();
			Q1 = copy[n / 4];
			Q3 = copy[3 * n / 4];
		}

		// Function to find outliers
		std::vector<size_t> findOutliersIQR(const std::vector<double>& data) {
			double Q1, Q3;
			calculateQuartiles(data, Q1, Q3);

			double IQR = Q3 - Q1;
			double lower_bound = Q1 - 1.5 * IQR;
			double upper_bound = Q3 + 1.5 * IQR;

			std::vector<size_t> outlier_indices;
			for (size_t i = 0; i < data.size(); ++i) {
				if (data[i] < lower_bound || data[i] > upper_bound) {
					outlier_indices.push_back(i);
				}
			}

			return outlier_indices;
		}
		
	}
	
}