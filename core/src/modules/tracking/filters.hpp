/* Copyright (C) 2025 Ian Sloat
 * Licensed under the GNU GPLv3 or later. See <https://www.gnu.org/licenses/>. */

#pragma once

#include <vector>
#include <array>
#include <opencv2/opencv.hpp>

constexpr float epsilon = 1e-5f;

template <int N>
class ConstantVelocityKF
{
public:
	ConstantVelocityKF(float noise_scale = 0.9) : noise_scale(noise_scale)
	{
		recalc_q();
		for (int i = 0; i < N; ++i)
		{
			kf[i] = cv::KalmanFilter(2, 1);
			kf[i].transitionMatrix = F;
			kf[i].processNoiseCov = Q;
			kf[i].measurementNoiseCov = R;
			kf[i].measurementMatrix = H;
		}
	}

	void set_delta(float dt)
	{
		if (dt == this->dt)
			return;
		F.at<float>(0, 1) = dt;
		recalc_q();
		this->dt = dt;
	}

	void set_noise_scale(float n_scale)
	{
		if (n_scale == noise_scale)
			return;
		noise_scale = n_scale;
		R.at<float>(0) = safe_noise_scale();
		recalc_q();
	}

	void update(std::array<float, N> pos)
	{
		if (u)
		{
			for (int i = 0; i < N; ++i)
			{
				kf[i].correct((cv::Mat_<float>(1, 1) << pos[i]));
			}
		}
		else
		{
			for (int i = 0; i < N; ++i)
			{
				kf[i].statePre = (cv::Mat_<float>(2, 1) << pos[i], 0);
				kf[i].statePost = (cv::Mat_<float>(2, 1) << pos[i], 0);
			}
			u = true;
		}
	}

	std::array<float, N> predict()
	{
		std::array<float, N> output;
		if (p)
		{
			for (int i = 0; i < N; ++i)
			{
				cv::Mat p = kf[i].predict();
				output[i] = p.at<float>(0);
			}
		}
		else
		{
			for (int i = 0; i < N; ++i)
			{
				cv::Mat sp = kf[i].statePre;
				output[i] = sp.at<float>(0);
			}
			p = true;
		}
		return output;
	}

	std::array<float, N> filter(std::array<float, N> pos)
	{
		update(pos);
		return predict();
	}

private:
	std::array<cv::KalmanFilter, N> kf;
	float noise_scale;
	float dt = 1;
	cv::Mat F = (cv::Mat_<float>(2, 2) << 1, 1, 0, 1);
	cv::Mat Q = cv::Mat::zeros(2, 2, CV_32F);
	cv::Mat R = (cv::Mat_<float>(1, 1) << safe_noise_scale());
	cv::Mat H = (cv::Mat_<float>(1, 2) << 1, 0);
	bool u = false;
	bool p = false;

	float safe_noise_scale()
	{
		return epsilon + (1 - 2 * epsilon) * std::min(std::max(noise_scale, 0.0f), 1.0f);
	}

	void recalc_q()
	{
		float dt4_4 = powf(dt, 4) / 4;
		float dt3_2 = powf(dt, 3) / 2;
		float dt2 = powf(dt, 2);
		Q.at<float>(0, 0) = dt4_4;
		Q.at<float>(0, 1) = dt3_2;
		Q.at<float>(1, 0) = dt3_2;
		Q.at<float>(1, 1) = dt2;
		Q *= 1 - safe_noise_scale();
	}
};

template <int N>
class ConstantAccelerationKF
{
public:
	ConstantAccelerationKF(float noise_scale = 0.9) : noise_scale(noise_scale)
	{
		recalc_q();
		for (int i = 0; i < N; ++i)
		{
			kf[i] = cv::KalmanFilter(3, 1);
			kf[i].transitionMatrix = F;
			kf[i].processNoiseCov = Q;
			kf[i].measurementNoiseCov = R;
			kf[i].measurementMatrix = H;
		}
	}

	void set_delta(float dt)
	{
		if (dt == this->dt)
			return;
		F.at<float>(0, 1) = dt;
		F.at<float>(1, 2) = dt;
		F.at<float>(0, 2) = powf(dt, 2) / 2;
		recalc_q();
		this->dt = dt;
	}

	void set_noise_scale(float n_scale)
	{
		if (n_scale == noise_scale)
			return;
		noise_scale = n_scale;
		R.at<float>(0) = safe_noise_scale();
		recalc_q();
	}

	void update(std::array<float, N> pos)
	{
		if (u)
		{
			for (int i = 0; i < N; ++i)
			{
				kf[i].correct((cv::Mat_<float>(1, 1) << pos[i]));
			}
		}
		else
		{
			for (int i = 0; i < N; ++i)
			{
				kf[i].statePre = (cv::Mat_<float>(3, 1) << pos[i], 0, 0);
				kf[i].statePost = (cv::Mat_<float>(3, 1) << pos[i], 0, 0);
			}
			u = true;
		}
	}

	std::array<float, N> predict()
	{
		std::array<float, N> output;
		if (p)
		{
			for (int i = 0; i < N; ++i)
			{
				output[i] = kf[i].predict().at(0);
			}
		}
		else
		{
			for (int i = 0; i < N; ++i)
			{
				output[i] = kf[i].statePre.at(0);
			}
			p = true;
		}
		return output;
	}

	std::array<float, N> filter(std::array<float, N> pos)
	{
		update(pos);
		return predict();
	}

private:
	std::array<cv::KalmanFilter, N> kf;
	float noise_scale;
	float dt = 1;
	cv::Mat F = (cv::Mat_<float>(3, 3) << 1, 1, 0.5, 
		                                  0, 1, 1, 
		                                  0, 0, 1);
	cv::Mat Q = cv::Mat::zeros(3, 3, CV_32F);
	cv::Mat R = (cv::Mat_<float>(1, 1) << safe_noise_scale());
	cv::Mat H = (cv::Mat_<float>(1, 3) << 1, 0, 0);
	bool u = false;
	bool p = false;

	float safe_noise_scale()
	{
		return epsilon + (1 - 2 * epsilon) * std::min(std::max(noise_scale, 0.0f), 1.0f);
	}

	void recalc_q()
	{
		float dt6_36 = powf(dt, 6) / 36;
		float dt5_12 = powf(dt, 5) / 12;
		float dt4_6 = powf(dt, 4) / 6;
		float dt4_4 = powf(dt, 4) / 4;
		float dt3_2 = powf(dt, 3) / 2;
		float dt2 = powf(dt, 2);
		Q.at<float>(0, 0) = dt6_36;
		Q.at<float>(0, 1) = dt5_12;
		Q.at<float>(1, 0) = dt5_12;
		Q.at<float>(1, 1) = dt4_4;
		Q.at<float>(0, 2) = dt4_6;
		Q.at<float>(2, 0) = dt4_6;
		Q.at<float>(1, 2) = dt3_2;
		Q.at<float>(2, 1) = dt3_2;
		Q.at<float>(2, 2) = dt2;
		Q *= 1 - safe_noise_scale();
	}
};

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
size_t mirror_idx(const int &index, const size_t &orig_length);

/// <summary>
/// Perform a 1D convolution on a provided input vector using a gaussian kernel.
/// Input will be padded with values such that the result retains the starting and ending values
/// of the original vector.
/// </summary>
/// <param name="input">Input vector</param>
/// <param name="kernel_radius">Radius of the kernel. A value of 1 would yield a 3 value kernel, 2 would yield 5, and so on.</param>
/// <returns>The result of convolution on the input vector</returns>
std::vector<double> g_conv_1d(const std::vector<double> &input, const int kernel_radius = 1);
