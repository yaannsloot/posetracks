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

#include <vector>

namespace me::tracking {

	/// <summary>
	/// Create a gaussian kernel
	/// </summary>
	/// <param name="width">Width of the kernel</param>
	/// <returns>A vector containing the values of the kernel</returns>
	std::vector<double> g_kernel_1d(const int width = 3);

	/// <summary>
	/// Get the mirror index of an element based on the provided index.
	/// If the provided index points to an out-of-bounds location, a mirror index will be returned.
	/// </summary>
	/// <param name="index">Target index. Can be positive or negative.</param>
	/// <param name="orig_length">Original length of the array</param>
	size_t mirror_idx(const int& index, const size_t& orig_length);

	/// <summary>
	/// Perform a 1D convolution on a provided input vector using a gaussian kernel.
	/// Input will be padded with values such that the result retains the starting and ending values 
	/// of the original vector.
	/// </summary>
	/// <param name="input">Input vector</param>
	/// <param name="kernel_radius">Radius of the kernel. A value of 1 would yield a 3 value kernel, 2 would yield 5, and so on.</param>
	/// <returns>The result of convolution on the input vector</returns>
	std::vector<double> g_conv_1d(const std::vector<double>& input, const int kernel_radius = 1);



}