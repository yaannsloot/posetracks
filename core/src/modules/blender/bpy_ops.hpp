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

#include "bpy_data.hpp"

void OP_FilterTrackGaussian(int kernel_width);

void OP_FilterFCurvesGaussian(int kernel_width);

void OP_TriangulatePoints(PyBOperator calling_op, const std::string& anchor);

void OP_SolveCameras_Invoke(const std::string& anchor);

void OP_SolveCameras_Execute(PyBOperator calling_op, const std::string& anchor, float solution_scale);