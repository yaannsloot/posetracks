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

#include "filters.hpp"
#include <cmath>
#include <stdexcept>
#include <string>
#include <algorithm>
#include <execution>

namespace me::tracking {

	double gaussian(const double& x, const double& mu, const double& sigma)
	{
		double a = (x - mu) / sigma;
		return exp(-0.5 * a * a);
	}

	std::vector<double> g_kernel_1d(const int width)
	{
		if (width < 1)
			throw std::invalid_argument("width must be >= 1");
		std::vector<double> kernel(width, 1);
		if (width == 1)
			return kernel;
		double sum = 0;
		double radius = static_cast<double>(width - 1) / 2;
		double sigma = radius / 2;
		for (int i = 0; i < width; ++i) {
			double a = gaussian(i, radius, sigma);
			sum += a;
			kernel[i] = a;
		}
		std::for_each(kernel.begin(), kernel.end(), [&](double& n) { n /= sum; });
		return kernel;
	}

	size_t mirror_idx(const int& index, const size_t& orig_length)
	{
		if (orig_length == 0)
			throw std::invalid_argument("original length must be > 0");
		if (orig_length == 1)
			return 0;
		const size_t last_index = orig_length - 1;
		const size_t abs_index = static_cast<size_t>(abs(index));
		if ((abs_index / last_index) % 2 != 0)
			return last_index - (abs_index % last_index);
		else
			return abs_index % last_index;
	}

	std::vector<double> g_conv_1d(const std::vector<double>& input, const int kernel_radius)
	{
		if (kernel_radius < 1)
			throw std::invalid_argument("kernel radius must be > 0");
		if (input.empty())
			return std::vector<double>();
		const int kernel_width = 1 + kernel_radius * 2;
		const auto kernel = g_kernel_1d(kernel_width);
		const int input_size = static_cast<int>(input.size());
		std::vector<double> output(input_size);
		std::vector<int> ivec(input_size);
		std::iota(ivec.begin(), ivec.end(), -kernel_radius);
		std::for_each(std::execution::par_unseq, ivec.begin(), ivec.end(), [&](const int& i) {
			double result = 0;
			for (int j = 0; j < kernel_width; ++j) {
				const int k = i + j;
				const double& val = input[mirror_idx(k, input_size)];
				if (k < 0) {
					result += (-1 * val + input.front() * 2) * kernel[j];
				}
				else if (k >= input_size) {
					result += (-1 * val + input.back() * 2) * kernel[j];
				}
				else {
					result += val * kernel[j];
				}
			}
			output[i + kernel_radius] = result;
		});
		return output;
	}



}