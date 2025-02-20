/* Copyright (C) 2025 Ian Sloat
 * Licensed under the GNU GPLv3 or later. See <https://www.gnu.org/licenses/>. */

#pragma once

#include "bpy_data.hpp"

void OP_FilterTrackGaussian(int kernel_width);

void OP_FilterFCurvesGaussian(int kernel_width, bool selected_only);

void OP_FilterTrackKalman(float measurementNoiseCov = 0.001, float processNoiseCov = 0.03);

void OP_FilterFCurvesKalman(float measurementNoiseCov = 0.001, float processNoiseCov = 0.03);

void OP_FilterLocationKalman(float measurementNoiseCov = 0.001, float processNoiseCov = 0.03);

void OP_TriangulatePoints(PyBOperator calling_op, const std::string &anchor);

void OP_SolveCameras_Invoke(const std::string &anchor);

void OP_SolveCameras_Execute(PyBOperator calling_op, const std::string &anchor, float solution_scale);
