#include <pch.h>
#include <chessboard_compute.hpp>
#include <MEComputeLib.h>
#include <motion_engine.hpp>
#include <nlohmann/json.hpp>
#include <nlohmann/json-schema.hpp>
#include <fstream>
#include <iostream>
#include <thread>
#include <stdexcept>
#include <assert.h>
#include <cstdio>
#include <algorithm>
#include <set>
#include <schema_asset.h>
#include <buildkey_asset.h>
#include <cctag/CCTag.hpp>
#include <cctag/ICCTag.hpp>
#include <cctag/Detection.hpp>
#include <boost/smart_ptr.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/types_c.h>

#define EPSILON 0.001

using json = nlohmann::json;
using nlohmann::json_schema::json_validator;

// Internal state variables
static std::string schemas = "./";
static char delim = ';';

// Private function definitions
void getQuaternion(cv::Mat R, double Q[]);

void computeDownsample(cv::Mat& source, cv::Mat& destination, bool resize, float downsample_scale, int downsample_width, int downsample_height, bool scale, bool width, bool height, bool &complete, json &cache_data);

// From sfm/src/projection.cpp & triangulation.cpp
namespace cv {
	namespace sfm {

		/**
		From "Triangulation", Hartley, R.I. and Sturm, P., Computer vision and image understanding, 1997
		*/
		Mat_<double> LinearLSTriangulation(Point3d u,       //homogenous image point (u,v,1)
			Matx34d P,       //camera 1 matrix
			Point3d u1,      //homogenous image point in 2nd camera
			Matx34d P1       //camera 2 matrix
		)
		{
			//build matrix A for homogenous equation system Ax = 0
			//assume X = (x,y,z,1), for Linear-LS method
			//which turns it into a AX = B system, where A is 4x3, X is 3x1 and B is 4x1
			Matx43d A(u.x * P(2, 0) - P(0, 0), u.x * P(2, 1) - P(0, 1), u.x * P(2, 2) - P(0, 2),
				u.y * P(2, 0) - P(1, 0), u.y * P(2, 1) - P(1, 1), u.y * P(2, 2) - P(1, 2),
				u1.x * P1(2, 0) - P1(0, 0), u1.x * P1(2, 1) - P1(0, 1), u1.x * P1(2, 2) - P1(0, 2),
				u1.y * P1(2, 0) - P1(1, 0), u1.y * P1(2, 1) - P1(1, 1), u1.y * P1(2, 2) - P1(1, 2)
			);
			Mat_<double> B = (Mat_<double>(4, 1) << -(u.x * P(2, 3) - P(0, 3)),
				-(u.y * P(2, 3) - P(1, 3)),
				-(u1.x * P1(2, 3) - P1(0, 3)),
				-(u1.y * P1(2, 3) - P1(1, 3)));

			Mat_<double> X;
			solve(A, B, X, DECOMP_SVD);

			return X;
		}

		/**
		From "Triangulation", Hartley, R.I. and Sturm, P., Computer vision and image understanding, 1997
		*/
		Mat_<double> IterativeLinearLSTriangulation(Point3d u,    //homogenous image point (u,v,1)
			Matx34d P,          //camera 1 matrix
			Point3d u1,         //homogenous image point in 2nd camera
			Matx34d P1          //camera 2 matrix
		) {

			double wi = 1, wi1 = 1;
			Mat_<double> X(4, 1);



			for (int i = 0; i < 10; i++) { //Hartley suggests 10 iterations at most
				Mat_<double> X_ = LinearLSTriangulation(u, P, u1, P1);
				X(0) = X_(0); X(1) = X_(1); X(2) = X_(2); X(3) = 1.0;
				//recalculate weights
				double p2x = Mat_<double>(Mat_<double>(P).row(2) * X)(0);
				double p2x1 = Mat_<double>(Mat_<double>(P1).row(2) * X)(0);

				//breaking point
				if (fabsf(wi - p2x) <= EPSILON && fabsf(wi1 - p2x1) <= EPSILON) break;

				wi = p2x;
				wi1 = p2x1;

				//reweight equations and solve
				Matx43d A((u.x * P(2, 0) - P(0, 0)) / wi, (u.x * P(2, 1) - P(0, 1)) / wi, (u.x * P(2, 2) - P(0, 2)) / wi,
					(u.y * P(2, 0) - P(1, 0)) / wi, (u.y * P(2, 1) - P(1, 1)) / wi, (u.y * P(2, 2) - P(1, 2)) / wi,
					(u1.x * P1(2, 0) - P1(0, 0)) / wi1, (u1.x * P1(2, 1) - P1(0, 1)) / wi1, (u1.x * P1(2, 2) - P1(0, 2)) / wi1,
					(u1.y * P1(2, 0) - P1(1, 0)) / wi1, (u1.y * P1(2, 1) - P1(1, 1)) / wi1, (u1.y * P1(2, 2) - P1(1, 2)) / wi1
				);
				Mat_<double> B = (Mat_<double>(4, 1) << -(u.x * P(2, 3) - P(0, 3)) / wi,
					-(u.y * P(2, 3) - P(1, 3)) / wi,
					-(u1.x * P1(2, 3) - P1(0, 3)) / wi1,
					-(u1.y * P1(2, 3) - P1(1, 3)) / wi1
					);

				solve(A, B, X_, DECOMP_SVD);
				X(0) = X_(0); X(1) = X_(1); X(2) = X_(2); X(3) = 1.0;
			}

			return X;
		}

		template<typename T>
		void
			homogeneousToEuclidean(const Mat& X_, Mat& x_)
		{
			int d = X_.rows - 1;

			const Mat_<T>& X_rows = X_.rowRange(0, d);
			const Mat_<T> h = X_.row(d);

			const T* h_ptr = h[0], * h_ptr_end = h_ptr + h.cols;
			const T* X_ptr = X_rows[0];
			T* x_ptr = x_.ptr<T>(0);
			for (; h_ptr != h_ptr_end; ++h_ptr, ++X_ptr, ++x_ptr)
			{
				const T* X_col_ptr = X_ptr;
				T* x_col_ptr = x_ptr, * x_col_ptr_end = x_col_ptr + d * x_.step1();
				for (; x_col_ptr != x_col_ptr_end; X_col_ptr += X_rows.step1(), x_col_ptr += x_.step1())
					*x_col_ptr = (*X_col_ptr) / (*h_ptr);
			}
		}

		void
			homogeneousToEuclidean(InputArray X_, OutputArray x_)
		{
			// src
			const Mat X = X_.getMat();

			// dst
			x_.create(X.rows - 1, X.cols, X.type());
			Mat x = x_.getMat();

			// type
			if (X.depth() == CV_32F)
			{
				homogeneousToEuclidean<float>(X, x);
			}
			else
			{
				homogeneousToEuclidean<double>(X, x);
			}
		}

		/** @brief Triangulates the a 3d position between two 2d correspondences, using the DLT.
		  @param xl Input vector with first 2d point.
		  @param xr Input vector with second 2d point.
		  @param Pl Input 3x4 first projection matrix.
		  @param Pr Input 3x4 second projection matrix.
		  @param objectPoint Output vector with computed 3d point.

		  Reference: @cite HartleyZ00 12.2 pag.312
		 */
		static void
			triangulateDLT(const Vec2d& xl, const Vec2d& xr,
				const Matx34d& Pl, const Matx34d& Pr,
				Vec3d& point3d)
		{
			Matx44d design;
			for (int i = 0; i < 4; ++i)
			{
				design(0, i) = xl(0) * Pl(2, i) - Pl(0, i);
				design(1, i) = xl(1) * Pl(2, i) - Pl(1, i);
				design(2, i) = xr(0) * Pr(2, i) - Pr(0, i);
				design(3, i) = xr(1) * Pr(2, i) - Pr(1, i);
			}

			Vec4d XHomogeneous;
			cv::SVD::solveZ(design, XHomogeneous);

			homogeneousToEuclidean(XHomogeneous, point3d);
		}

		/** @brief Triangulates the a 3d position between two 2d correspondences, using the HZ method.
		  @param xl Input vector with first 2d point.
		  @param xr Input vector with second 2d point.
		  @param Pl Input 3x4 first projection matrix.
		  @param Pr Input 3x4 second projection matrix.
		  @param objectPoint Output vector with computed 3d point.

		  From "Triangulation", Hartley, R.I. and Sturm, P., Computer vision and image understanding, 1997
		 */
		static void
			triangulateHZ(const Vec2d& xl, const Vec2d& xr,
				const Matx34d& Pl, const Matx34d& Pr,
				Vec3d& point3d)
		{
			cv::Point3d u(xl(0), xl(1), 1);
			cv::Point3d u1(xr(0), xr(1), 1);
			Vec4d XHomogeneous = IterativeLinearLSTriangulation(u, Pl, u1, Pr);

			//homogeneousToEuclidean(XHomogeneous, point3d);
			point3d(0) = XHomogeneous(0) / XHomogeneous(3);
			point3d(1) = XHomogeneous(1) / XHomogeneous(3);
			point3d(2) = XHomogeneous(2) / XHomogeneous(3);
		}

		/** @brief Triangulates the 3d position of 2d correspondences between n images, using the DLT
		 * @param x Input vectors of 2d points (the inner vector is per image). Has to be 2xN
		 * @param Ps Input vector with 3x4 projections matrices of each image.
		 * @param X Output vector with computed 3d point.

		 * Reference: it is the standard DLT; for derivation see appendix of Keir's thesis
		 */
		static void
			triangulateNViews(const Mat_<double>& x, const std::vector<Matx34d>& Ps, Vec3d& X)
		{
			CV_Assert(x.rows == 2);
			unsigned nviews = x.cols;
			CV_Assert(nviews == Ps.size());

			cv::Mat_<double> design = cv::Mat_<double>::zeros(3 * nviews, 4 + nviews);
			for (unsigned i = 0; i < nviews; ++i) {
				for (char jj = 0; jj < 3; ++jj)
					for (char ii = 0; ii < 4; ++ii)
						design(3 * i + jj, ii) = -Ps[i](jj, ii);
				design(3 * i + 0, 4 + i) = x(0, i);
				design(3 * i + 1, 4 + i) = x(1, i);
				design(3 * i + 2, 4 + i) = 1.0;
			}

			Mat X_and_alphas;
			cv::SVD::solveZ(design, X_and_alphas);
			homogeneousToEuclidean(X_and_alphas.rowRange(0, 4), X);
		}

		/** @brief Triangulates the 3d position of 2d correspondences between n images, using the HZ method
		 * @param x Input vectors of 2d points (the inner vector is per image). Has to be 2xN
		 * @param Ps Input vector with 3x4 projections matrices of each image.
		 * @param X Output vector with computed 3d point.

		 * From "Triangulation", Hartley, R.I. and Sturm, P., Computer vision and image understanding, 1997
		 */
		static void
			triangulateNViewsHZ(const Mat_<double>& x, const std::vector<Matx34d>& Ps, Vec3d& X)
		{
			CV_Assert(x.rows == 2);
			unsigned nviews = x.cols;
			CV_Assert(nviews == Ps.size());

			std::set<me::utility::Edge<int>> combinations;

			for (int i = 0; i < nviews; i++) {
				for (int j = 0; j < nviews; j++) {
					if (i != j)
						combinations.emplace(i, j);
				}
			}

			X(0) = 0;
			X(1) = 0;
			X(2) = 0;

			for (auto views = combinations.begin(); views != combinations.end(); views++) {
				Vec2d xl, xr;
				xl(0) = x(0, views->getA());
				xl(1) = x(1, views->getA());
				xr(0) = x(0, views->getB());
				xr(1) = x(1, views->getB());
				Vec3d X_sub(0, 0, 0);
				triangulateHZ(xl, xr, Ps[views->getA()], Ps[views->getB()], X_sub);
				X(0) += X_sub(0) / combinations.size();
				X(1) += X_sub(1) / combinations.size();
				X(2) += X_sub(2) / combinations.size();
			}
		}

		void
			triangulatePoints(InputArrayOfArrays _points2d, InputArrayOfArrays _projection_matrices,
				OutputArray _points3d)
		{
			// check
			size_t nviews = (unsigned)_points2d.total();
			CV_Assert(nviews >= 2 && nviews == _projection_matrices.total());

			// inputs
			size_t n_points;
			std::vector<Mat_<double> > points2d(nviews);
			std::vector<Matx34d> projection_matrices(nviews);
			{
				std::vector<Mat> points2d_tmp;
				_points2d.getMatVector(points2d_tmp);
				n_points = points2d_tmp[0].cols;

				std::vector<Mat> projection_matrices_tmp;
				_projection_matrices.getMatVector(projection_matrices_tmp);

				// Make sure the dimensions are right
				for (size_t i = 0; i < nviews; ++i) {
					CV_Assert(points2d_tmp[i].rows == 2 && points2d_tmp[i].cols == n_points);
					if (points2d_tmp[i].type() == CV_64F)
						points2d[i] = points2d_tmp[i];
					else
						points2d_tmp[i].convertTo(points2d[i], CV_64F);

					CV_Assert(projection_matrices_tmp[i].rows == 3 && projection_matrices_tmp[i].cols == 4);
					if (projection_matrices_tmp[i].type() == CV_64F)
						projection_matrices[i] = projection_matrices_tmp[i];
					else
						projection_matrices_tmp[i].convertTo(projection_matrices[i], CV_64F);
				}
			}

			// output
			_points3d.create(3, n_points, CV_64F);
			cv::Mat points3d = _points3d.getMat();

			// Two view
			if (nviews == 2)
			{
				const Mat_<double>& xl = points2d[0], & xr = points2d[1];

				const Matx34d& Pl = projection_matrices[0];    // left matrix projection
				const Matx34d& Pr = projection_matrices[1];    // right matrix projection

				// triangulate
				for (unsigned i = 0; i < n_points; ++i)
				{
					Vec3d point3d;
					triangulateDLT(Vec2d(xl(0, i), xl(1, i)), Vec2d(xr(0, i), xr(1, i)), Pl, Pr, point3d);
					for (char j = 0; j < 3; ++j)
						points3d.at<double>(j, i) = point3d[j];
				}
			}
			else if (nviews > 2)
			{
				// triangulate
				for (unsigned i = 0; i < n_points; ++i)
				{
					// build x matrix (one point per view)
					Mat_<double> x(2, nviews);
					for (unsigned k = 0; k < nviews; ++k)
					{
						points2d.at(k).col(i).copyTo(x.col(k));
					}

					Vec3d point3d;
					triangulateNViews(x, projection_matrices, point3d);
					for (char j = 0; j < 3; ++j)
						points3d.at<double>(j, i) = point3d[j];
				}
			}
		}

		void
			triangulatePointsHZ(InputArrayOfArrays _points2d, InputArrayOfArrays _projection_matrices,
				OutputArray _points3d)
		{
			// check
			size_t nviews = (unsigned)_points2d.total();
			CV_Assert(nviews >= 2 && nviews == _projection_matrices.total());

			// inputs
			size_t n_points;
			std::vector<Mat_<double> > points2d(nviews);
			std::vector<Matx34d> projection_matrices(nviews);
			std::vector<Mat> camera_matrices(nviews);
			{
				std::vector<Mat> points2d_tmp;
				_points2d.getMatVector(points2d_tmp);
				n_points = points2d_tmp[0].cols;

				std::vector<Mat> projection_matrices_tmp;
				_projection_matrices.getMatVector(projection_matrices_tmp);

				// Make sure the dimensions are right
				for (size_t i = 0; i < nviews; ++i) {
					CV_Assert(points2d_tmp[i].rows == 2 && points2d_tmp[i].cols == n_points);
					if (points2d_tmp[i].type() == CV_64F)
						points2d[i] = points2d_tmp[i];
					else
						points2d_tmp[i].convertTo(points2d[i], CV_64F);

					CV_Assert(projection_matrices_tmp[i].rows == 3 && projection_matrices_tmp[i].cols == 4);
					if (projection_matrices_tmp[i].type() == CV_64F)
						projection_matrices[i] = projection_matrices_tmp[i];
					else
						projection_matrices_tmp[i].convertTo(projection_matrices[i], CV_64F);
				}
			}

			// output
			_points3d.create(3, n_points, CV_64F);
			cv::Mat points3d = _points3d.getMat();

			// Two view
			if (nviews == 2)
			{
				const Mat_<double>& xl = points2d[0], & xr = points2d[1];

				const Matx34d& Pl = projection_matrices[0];    // left matrix projection
				const Matx34d& Pr = projection_matrices[1];    // right matrix projection

				const Mat Kl = camera_matrices[0];
				const Mat Kr = camera_matrices[1];

				// triangulate
				for (unsigned i = 0; i < n_points; ++i)
				{
					Vec3d point3d;
					triangulateHZ(Vec2d(xl(0, i), xl(1, i)), Vec2d(xr(0, i), xr(1, i)), Pl, Pr, point3d);
					for (char j = 0; j < 3; ++j)
						points3d.at<double>(j, i) = point3d[j];
				}
			}
			else if (nviews > 2)
			{
				// triangulate
				for (unsigned i = 0; i < n_points; ++i)
				{
					// build x matrix (one point per view)
					Mat_<double> x(2, nviews);
					for (unsigned k = 0; k < nviews; ++k)
					{
						points2d.at(k).col(i).copyTo(x.col(k));
					}

					Vec3d point3d;
					triangulateNViewsHZ(x, projection_matrices, point3d);
					for (char j = 0; j < 3; ++j)
						points3d.at<double>(j, i) = point3d[j];
				}
			}
		}

		template<typename T>
		void
			projectionFromKRt(const Mat_<T>& K, const Mat_<T>& R, const Mat_<T>& t, Mat_<T> P)
		{
			hconcat(K * R, K * t, P);
		}

		void
			projectionFromKRt(InputArray K_, InputArray R_, InputArray t_, OutputArray P_)
		{
			const Mat K = K_.getMat(), R = R_.getMat(), t = t_.getMat();
			const int depth = K.depth();
			CV_Assert((K.cols == 3 && K.rows == 3) && (t.cols == 1 && t.rows == 3) && (K.size() == R.size()));
			CV_Assert((depth == CV_32F || depth == CV_64F) && depth == R.depth() && depth == t.depth());

			P_.create(3, 4, depth);

			Mat P = P_.getMat();

			// type
			if (depth == CV_32F)
			{
				projectionFromKRt<float>(K, R, t, P);
			}
			else
			{
				projectionFromKRt<double>(K, R, t, P);
			}

		}

	}
}

bool hash_file(char* input_file, char* output_hash) {
	me::utility::memstream key(keyfile_bin, keyfile_bin_len);
	std::ifstream file(input_file, std::ifstream::in, std::ifstream::binary);
	if (file.is_open()) {
		std::string hash = me::crypto::StreamToHMAC_SHA1(file, key).to_string();
		file.close();
		me::utility::log("Hashed file \"" + std::string(input_file) + "\" (" + hash + ")");
		for (int i = 0; i < 40; i++) {
			output_hash[i] = hash[i];
		}
		return true;
	}
	return false;
}

bool analyze_frames(char* job_config_file, char* data_cache_file, bool use_bson_format) {

	// Cache prep
	json cache_data;
	std::ifstream cache_prev(data_cache_file, std::ifstream::in);
	if (cache_prev.is_open()) { // Check if exists
		me::utility::log("Loading cache file \"" + std::string(data_cache_file) + '"');
		cache_data = json::parse(cache_prev, nullptr, false, true); // Parse as json
		if (cache_data.is_discarded()) { // Secondary check as bson if parse failed
			cache_data = json();
			cache_prev.close();
			cache_prev.open(data_cache_file, std::ifstream::in | std::ifstream::binary); // Load as binary file
			if (cache_prev.is_open()) { // Check AGAIN just in case it really doesn't exist
				cache_data = json::from_bson(cache_prev, true, false); // Parse as bson
				if (cache_data.is_discarded()) {// Shoot it can't be read. Just make a new file I guess
					cache_data = json();
					me::utility::log("Existing cache file could not be read. A new file will be used instead.");
				}
			}
		}
	}
	else {
		me::utility::log("Created cache file \"" + std::string(data_cache_file) + '"');
	}
	if (cache_prev.is_open()) // Ensure the cache file is closed since we won't need it anymore
		cache_prev.close();
	cache_data["errors"] = json(); // Clear the error list for current analysis. Should be null on a successful run

	// PUT A CHECK HERE TO MAKE SURE THE CACHE DIRECTORIES ARE REACHABLE YOU ASSHOLE :)

	try {
		std::ifstream config(job_config_file, std::ifstream::in);
		// Config file check
		if (config.is_open()) { // Checks if config exists
			me::utility::log("Loading job config \"" + std::string(job_config_file) + '"');
			json schema_data = json::from_bson(calibration_config_bson, calibration_config_bson_len); // Load embedded schema definition from executable. Comes from schema_asset.h
			json config_data = json::parse(config, nullptr, false, true); // Load the config WITHOUT exceptions
			if (!config_data.is_discarded() && !schema_data.is_discarded()) { // Checks if the json parsing was successful

				bool valid = true;

				json_validator validator;
				validator.set_root_schema(schema_data);
				// Schema check for config. Filter out those bad values and catch misconfiguration
				try {
					validator.validate(config_data);
				}
				catch (const std::exception& e) {
					std::string what(e.what());
					me::utility::log(what, "ERROR");
					cache_data["errors"]["validation"].push_back(what);
					valid = false;
				}

				me::CalibrationConfig calibration_config;
				config_data.get_to(calibration_config);

				me::utility::log("Job config loaded.");

				// Input file check
				for (int i = 0; i < calibration_config.get_total_sequence_files(); i++) {
					std::ifstream sf(calibration_config.get_sequence_file_at(i), std::ifstream::in | std::ifstream::binary);
					if (!sf.is_open()) {
						valid = false;
						cache_data["errors"]["bad_path"].push_back(calibration_config.get_sequence_file_at(i));
						me::utility::log("Sequence file \"" + calibration_config.get_sequence_file_at(i) + "\" does not exist or could not be read.", "ERROR");
					}
				}

				// Pre-analysis config value sanity check (mostly checking ranges)
				if (calibration_config.do_calibrate_camera() && calibration_config.get_calibration_range().is_defined() && calibration_config.get_analysis_range().is_defined()
					&& calibration_config.get_calibration_range().get_begin() >= 0 && calibration_config.get_analysis_range().get_begin() >= 0
					&& calibration_config.get_calibration_range().get_end() >= 0 && calibration_config.get_analysis_range().get_end() >= 0) {
					if (calibration_config.get_analysis_range().get_begin() > calibration_config.get_analysis_range().get_end()
						|| calibration_config.get_calibration_range().get_begin() > calibration_config.get_calibration_range().get_end()) {
						valid = false;
						if (calibration_config.get_analysis_range().get_begin() > calibration_config.get_analysis_range().get_end()) {
							cache_data["errors"]["conflicting_values"].push_back("analysis_range is invalid (end > begin)");
							me::utility::log("analysis_range is invalid (end > begin)", "ERROR");
						}
						if (calibration_config.get_calibration_range().get_begin() > calibration_config.get_calibration_range().get_end()) {
							cache_data["errors"]["conflicting_values"].push_back("calibration_range is invalid (end > begin)");
							me::utility::log("calibration_range is invalid (end > begin)", "ERROR");
						}
					}
					else if (calibration_config.get_analysis_range().get_begin() > calibration_config.get_calibration_range().get_begin()
						|| calibration_config.get_analysis_range().get_end() < calibration_config.get_calibration_range().get_end()) {
						valid = false;
						cache_data["errors"]["conflicting_values"].push_back("calibration_range falls outside of analysis_range");
						me::utility::log("calibration_range falls outside of analysis_range", "ERROR");
					}
				}

				// Continues with analysis if verification was successful
				if (valid) {
					// Resources to use during analysis
					cv::VideoWriter master_video_writer;
					cv::VideoCapture master_video_reader;

					// PUT DOWNSAMPLE GARBAGE HERE LATER

					// Here we set analysis targets. If the sequence was defined as a list of images we will encode them into a video
					me::utility::log("Hashing sequence files...");

					me::utility::memstream key(keyfile_bin, keyfile_bin_len);
					std::map<std::string, std::string> analysis_targets;
					if(calibration_config.get_sequence_type() == me::SequenceType::SEQUENCE_VIDEO) {
						for(int i = 0; i < calibration_config.get_total_sequence_files(); i++) {
							std::ifstream video_file(calibration_config.get_sequence_file_at(i), std::ifstream::in, std::ifstream::binary);
							std::string video_hash = me::crypto::StreamToHMAC_SHA1(video_file, key).to_string();
							video_file.close();
							analysis_targets.emplace(video_hash, calibration_config.get_sequence_file_at(i));
							std::string file_name = calibration_config.get_sequence_file_at(i);
							if (file_name.find_last_of("/\\") != std::string::npos)
								file_name = file_name.substr(file_name.find_last_of("/\\") + 1);
							me::utility::log("Hashed video \"" + file_name + "\" (" + video_hash + ")");
						}
					}
					else {
						me::utility::log("Sequence type is set to \"images\". Encoding images into temorary video file...");
						std::vector<std::string> images;
						std::vector<std::string> file_names;
						for(int i = 0; i < calibration_config.get_total_sequence_files(); i++) {
							images.push_back(calibration_config.get_sequence_file_at(i));
							std::string name = calibration_config.get_sequence_file_at(i);
							if (name.find_last_of("/\\") != std::string::npos)
								name = name.substr(name.find_last_of("/\\") + 1);
							file_names.push_back(name);
						}
						std::sort(file_names.begin(), file_names.end(), [](const std::string& a, const std::string& b) {return a < b; });
						std::string keyhash = me::crypto::hashStreamSHA1(key).to_string();
						me::utility::log("Generating parity hash...");
						me::crypto::createParityFile(images, keyhash);
						std::ifstream parity_file(keyhash, std::ifstream::in | std::ifstream::binary);
						std::string parity_hash = me::crypto::StreamToHMAC_SHA1(parity_file, key).to_string();
						parity_file.close();
						std::remove(keyhash.c_str());
						std::stringstream name_concat;
						name_concat << file_names[0] << "..." << file_names[file_names.size() - 1];
						cv::Mat img = cv::imread(images[0]);
						me::utility::log("Writing frames...");
						master_video_writer.open(name_concat.str(), cv::CAP_FFMPEG,
							cv::VideoWriter::fourcc('m', 'p', '4', 'v'), 24, cv::Size(img.cols, img.rows), { cv::VIDEOWRITER_PROP_HW_ACCELERATION, cv::VIDEO_ACCELERATION_ANY });
						for (std::string f : images) {
							img = cv::imread(f);
							master_video_writer.write(img);
						}
						master_video_writer.release();
						analysis_targets.emplace(parity_hash, name_concat.str());
						me::utility::log("Hashed sequence images \"" + name_concat.str() + "\" (" + parity_hash + ")");
					}

					unsigned long current_target = 1;
					// MAIN ANALYSIS LOOP: Loop through the targets
					for (auto it = analysis_targets.begin(); it != analysis_targets.end(); it++) {
						// PART 1: MARKER DETECTION

						// Set group targets
						std::string group_id = it->first;
						std::string target_path = it->second;

						// Write file name to cache
						std::string file_name = target_path;
						if (file_name.find_last_of("/\\") != std::string::npos)
							file_name = file_name.substr(file_name.find_last_of("/\\") + 1);
						cache_data[group_id]["file_name"] = file_name;

						// Write hashed key to cache for version identification
						cache_data[group_id]["runtime_ver"] = me::crypto::StreamToHMAC_SHA1(key, key).to_string();

						// Write fps to cache for proper time scaling in further calculations
						master_video_reader.open(target_path, cv::CAP_FFMPEG, { cv::CAP_PROP_HW_ACCELERATION, cv::VIDEO_ACCELERATION_ANY });
						cache_data[group_id]["frames_per_second"] = master_video_reader.get(cv::CAP_PROP_FPS);
						master_video_reader.release();

						// Clear previous data for group if it exists and the "clear_point_cache" flag is set
						if (calibration_config.do_clear_cache()) {
							for (auto it = cache_data[group_id]["frames"].begin(); it != cache_data[group_id]["frames"].end(); it++) {
								if (it->contains("analysis_data"))
									it->erase("analysis_data");
							}
						}

						// Initialize memory structures and job variables
						std::set<int> dict_nums;
						if (calibration_config.get_charuco_properties().is_defined())
							dict_nums.insert(calibration_config.get_charuco_properties().get_dictionary());
						if (calibration_config.get_marker_properties().is_defined())
							dict_nums.insert(calibration_config.get_marker_properties().get_dictionary());
						if (calibration_config.get_sync_properties().is_defined())
							dict_nums.insert(calibration_config.get_sync_properties().get_dictionary());
						std::map<int, std::vector<std::vector<std::vector<cv::Point2f>>>> marker_corners;
						std::map<int, std::vector<std::vector<int>>> marker_ids;
						std::map<int, std::vector<bool>> marker_success;
						for (auto it = dict_nums.begin(); it != dict_nums.end(); it++) {
							marker_corners.emplace(*it, std::vector<std::vector<std::vector<cv::Point2f>>>());
							marker_ids.emplace(*it, std::vector<std::vector<int>>());
							marker_success.emplace(*it, std::vector<bool>());
						}
						std::vector<std::vector<cv::Point2f>> board_corners;
						std::vector<std::vector<int>> board_ids;
						std::vector<bool> board_success;

						// SUBLOOP: Full scan for each dictionary. This will happen at most 3 times per analysis target
						for (auto itt = dict_nums.begin(); itt != dict_nums.end(); itt++) {
							// JOB CONFIG
							me::ComputeTaskParams compute_params;
							compute_params.filepath = target_path;
							if (calibration_config.get_charuco_properties().is_defined() && calibration_config.get_charuco_properties().get_dictionary() == *itt) {
								me::calibration::CharucoProperties c_props = calibration_config.get_charuco_properties();
								cv::Ptr<cv::aruco::CharucoBoard> board = cv::aruco::CharucoBoard::create(c_props.get_size().width, c_props.get_size().height,
									c_props.get_square_length(), c_props.get_marker_length(), cv::aruco::getPredefinedDictionary(c_props.get_dictionary()));
								compute_params.board = board;
								compute_params.board_corners = &board_corners;
								compute_params.board_ids = &board_ids;
								compute_params.board_success = &board_success;
							}
							compute_params.dictionary = cv::aruco::getPredefinedDictionary(*itt);
							compute_params.marker_corners = &marker_corners.at(*itt);
							compute_params.marker_ids = &marker_ids.at(*itt);
							compute_params.marker_success = &marker_success.at(*itt);

							
							// Marker detection fine tuning
							int size_min = 10;
							int size_max = 80;
							int step = 10;
							cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
							parameters->adaptiveThreshWinSizeMin = size_min;
							parameters->adaptiveThreshWinSizeMax = size_max;
							parameters->adaptiveThreshWinSizeStep = step;
							parameters->errorCorrectionRate = 0.5;
							parameters->cornerRefinementMethod = cv::aruco::CORNER_REFINE_APRILTAG;
							compute_params.marker_parameters = parameters;

							// JOB CREATION AND EXECUTION
							me::ARComputeTask compute_task(compute_params);
							compute_task.start();
							while (compute_task.busy()) {
								me::utility::log("Detecting markers for \"" + me::utility::dictEnumToString(*itt) + "\"... (" + std::to_string(compute_task.get_complete()) + '/' + std::to_string(compute_task.get_total()) + ')',
									"JOB " + std::to_string(current_target) + "/" + std::to_string(analysis_targets.size()) + " (" + it->first.substr(0, 8) + ") | INFO");
								Sleep(1000);
							}
							me::utility::log("Detecting markers for \"" + me::utility::dictEnumToString(*itt) + "\"... (" + std::to_string(compute_task.get_total()) + '/' + std::to_string(compute_task.get_total()) + ')',
								"JOB " + std::to_string(current_target) + "/" + std::to_string(analysis_targets.size()) + " (" + it->first.substr(0, 8) + ") | INFO");
							compute_task.wait();

							// Write results to cache
							me::utility::log("Writing results to cache...",
								"JOB " + std::to_string(current_target) + "/" + std::to_string(analysis_targets.size()) + " (" + it->first.substr(0, 8) + ") | INFO");
							for (int i = 0; i < marker_success.at(*itt).size(); i++) {

								if (marker_success.at(*itt).at(i) == true) {
									for (int j = 0; j < marker_ids.at(*itt).at(i).size(); j++) {
										for (int k = 0; k < marker_corners.at(*itt).at(i).at(j).size(); k++) {
											cache_data[group_id]["frames"][std::to_string(i)]["analysis_data"]["marker_corners"][me::utility::dictEnumToString(*itt)]
												[std::to_string(marker_ids.at(*itt).at(i).at(j))][k][0] = marker_corners.at(*itt).at(i).at(j).at(k).x;
											cache_data[group_id]["frames"][std::to_string(i)]["analysis_data"]["marker_corners"][me::utility::dictEnumToString(*itt)]
												[std::to_string(marker_ids.at(*itt).at(i).at(j))][k][1] = marker_corners.at(*itt).at(i).at(j).at(k).y;
										}
									}
								}
								if (calibration_config.get_charuco_properties().is_defined() && calibration_config.get_charuco_properties().get_dictionary() == *itt && board_success.at(i)) {
									for (int j = 0; j < board_ids.at(i).size(); j++) {
										cache_data[group_id]["frames"][std::to_string(i)]["analysis_data"]["charuco_corners"][std::to_string(board_ids.at(i).at(j))][0] = board_corners.at(i).at(j).x;
										cache_data[group_id]["frames"][std::to_string(i)]["analysis_data"]["charuco_corners"][std::to_string(board_ids.at(i).at(j))][1] = board_corners.at(i).at(j).y;
									}
								}
							}

							// Save the current config for reference
							cache_data[group_id]["previous_config"] = calibration_config;

						}

						// PART 2: POST-ANALYSIS DATA PROCESSING
						
						// CAMERA CALIBRATION
						cv::Mat cam_mtx;
						cv::Mat cam_dist;
						me::calibration::CharucoProperties c_props;
						if (calibration_config.get_charuco_properties().is_defined() && calibration_config.do_calibrate_camera()) {
							c_props = calibration_config.get_charuco_properties();
							me::utility::log("Calibrating camera...",
								"JOB " + std::to_string(current_target) + "/" + std::to_string(analysis_targets.size()) + " (" + it->first.substr(0, 8) + ") | INFO");
							std::vector<std::vector<cv::Point2f>> filtered_board_corners;
							std::vector<std::vector<int>> filtered_board_ids;
							for (int i = 0; i < board_success.size(); i++) {
								if (board_success.at(i)) {
									filtered_board_corners.push_back(board_corners.at(i));
									filtered_board_ids.push_back(board_ids.at(i));
								}
							}
							std::vector<std::vector<cv::Point2f>> target_board_corners;
							std::vector<std::vector<int>> target_board_ids;
							size_t offset = filtered_board_corners.size() / ((size_t)calibration_config.get_calibration_frame_count() - 1);
							if (offset == 0)
								offset = 1;
							for (size_t i = 0; i < filtered_board_ids.size(); i += offset) {
								target_board_corners.push_back(filtered_board_corners.at(i));
								target_board_ids.push_back(filtered_board_ids.at(i));
							}
							if (target_board_ids.size() == 0)
								throw std::runtime_error("no points were discovered in the specified calibration range");
							me::calibration::CharucoProperties c_props = calibration_config.get_charuco_properties();
							cv::Ptr<cv::aruco::CharucoBoard> board = cv::aruco::CharucoBoard::create(c_props.get_size().width, c_props.get_size().height,
								c_props.get_square_length(), c_props.get_marker_length(), cv::aruco::getPredefinedDictionary(c_props.get_dictionary()));
							master_video_reader.open(target_path, cv::CAP_FFMPEG, { cv::CAP_PROP_HW_ACCELERATION, cv::VIDEO_ACCELERATION_ANY });
							cv::Size frame_size((int)master_video_reader.get(cv::CAP_PROP_FRAME_WIDTH), (int)master_video_reader.get(cv::CAP_PROP_FRAME_HEIGHT));
							master_video_reader.release();
							cv::aruco::calibrateCameraCharuco(target_board_corners, target_board_ids, board, frame_size, cam_mtx, cam_dist);
							me::utility::log("Calibration complete.",
								"JOB " + std::to_string(current_target) + "/" + std::to_string(analysis_targets.size()) + " (" + it->first.substr(0, 8) + ") | INFO");
						}
						else {
							// PUT CRAP HERE ONCE A JSON COMPATIBLE CACHE OBJECT IS IMPLEMENTED
						}

						// Write camera matrix to cache if it exists
						if (!cam_mtx.empty() && !cam_dist.empty()) {
							for (int i = 0; i < cam_mtx.rows; i++) {
								for (int j = 0; j < cam_mtx.cols; j++) {
									cache_data[group_id]["calibration_data"]["camera_matrix"][i][j] = cam_mtx.ptr<double>(i)[j];
								}
							}
							for (int i = 0; i < cam_dist.cols; i++) {
								cache_data[group_id]["calibration_data"]["distortion_coefficients"][i] = cam_dist.ptr<double>(0)[i];
							}
							cache_data[group_id]["calibration_data"]["charuco_properties"] = c_props;
						}
						 
						// LOAD A SEPARATE cam_mtx AND cam_dist HERE WHEN YOU ADD THE ABILITY TO USE CALIBRATION DATA FROM A DIFFRENT GROUP OR PUT IT IN A DIFFERENT FUNCTION

						// CAMERA POSE ESTIMATION
						if (!cam_mtx.empty() && !cam_dist.empty()) {
							me::utility::log("Estimating ChArUco pose information...",
								"JOB " + std::to_string(current_target) + "/" + std::to_string(analysis_targets.size()) + " (" + it->first.substr(0, 8) + ") | INFO");
							for (auto it = cache_data[group_id]["frames"].begin(); it != cache_data[group_id]["frames"].end(); it++) {
								if (it->contains("pose_data"))
									it->erase("pose_data");
							}
							me::calibration::CharucoProperties c_props = calibration_config.get_charuco_properties();
							cv::Ptr<cv::aruco::CharucoBoard> board = cv::aruco::CharucoBoard::create(c_props.get_size().width, c_props.get_size().height,
								c_props.get_square_length(), c_props.get_marker_length(), cv::aruco::getPredefinedDictionary(c_props.get_dictionary()));
							for (int i = 0; i < board_success.size(); i++) {
								cv::Mat rvec;
								cv::Mat tvec;
								if (board_success.at(i)) {
									bool pose_found = cv::aruco::estimatePoseCharucoBoard(board_corners.at(i), board_ids.at(i), board, cam_mtx, cam_dist, rvec, tvec);
									if (pose_found) {
										for (int i = 0; i < 10; i++) {
											cv::aruco::estimatePoseCharucoBoard(board_corners.at(i), board_ids.at(i), board, cam_mtx, cam_dist, rvec, tvec, true);
										}
										for (int j = 0; j < tvec.rows; j++) {
											cache_data[group_id]["frames"][std::to_string(i)]["pose_data"]["charuco_board"]["tvec"][j] = tvec.ptr<double>(j)[0];
										}
										for (int j = 0; j < rvec.rows; j++) {
											cache_data[group_id]["frames"][std::to_string(i)]["pose_data"]["charuco_board"]["rvec"][j] = rvec.ptr<double>(j)[0];
										}
									}
								}
							}
							me::utility::log("Pose estimation complete.",
								"JOB " + std::to_string(current_target) + "/" + std::to_string(analysis_targets.size()) + " (" + it->first.substr(0, 8) + ") | INFO");
						}
						
						// SYNC FRAME DETECTION
						if (calibration_config.get_sync_properties().is_defined()) {
							me::utility::log("Detecting sync frame...",
								"JOB " + std::to_string(current_target) + "/" + std::to_string(analysis_targets.size()) + " (" + it->first.substr(0, 8) + ") | INFO");
							auto* m_success = &marker_success.at(calibration_config.get_sync_properties().get_dictionary());
							auto* m_ids = &marker_ids.at(calibration_config.get_sync_properties().get_dictionary());
							bool found = false;
							for (int i = 0; i < m_success->size(); i++) {
								if (m_success->at(i) && std::find(m_ids->at(i).begin(), m_ids->at(i).end(), calibration_config.get_sync_properties().get_id()) != m_ids->at(i).end()) {
									cache_data[group_id]["calibration_data"]["sync_data"]["frame_number"] = i;
									cache_data[group_id]["calibration_data"]["sync_data"]["marker_dictionary"] = me::utility::dictEnumToString(calibration_config.get_sync_properties().get_dictionary());
									cache_data[group_id]["calibration_data"]["sync_data"]["marker_id"] = calibration_config.get_sync_properties().get_id();
									found = true;
									me::utility::log("Sync frame detected at frame " + std::to_string(i) + '.',
										"JOB " + std::to_string(current_target) + "/" + std::to_string(analysis_targets.size()) + " (" + it->first.substr(0, 8) + ") | INFO");
									break;
								}
							}
							if(!found)
								me::utility::log("Sync frame could not be found. Manual syncing may be necessary.",
									"JOB " + std::to_string(current_target) + "/" + std::to_string(analysis_targets.size()) + " (" + it->first.substr(0, 8) + ") | INFO");
						}

						current_target++;
						// END OF MAIN LOOP
					}

					// CLEANUP: If we managed to arrive here, we assume all went smoothly and the success flag can be set to true

					// Close master writer if it is still open
					if (master_video_writer.isOpened())
						master_video_writer.release();

					// Close master reader if it is still open
					if (master_video_reader.isOpened())
						master_video_reader.release();

					// Set the flag. Congrats, you made it all the way to the end!
					cache_data["success"] = true;
					
				}
				else {
					cache_data["success"] = false;
				}
			}
		}
		else
		me::utility::log("Something went wrong when trying to access job files", "ERROR");
	}
	catch (cv::Exception& e) {
		std::string what(e.what());
		me::utility::log(what, "ERROR");
		cache_data["errors"]["runtime"].push_back(what);
		cache_data["success"] = false;
	}
	catch (const std::exception& ex) {
		std::string what(ex.what());
		me::utility::log(what, "ERROR");
		cache_data["errors"]["runtime"].push_back(what);
		cache_data["success"] = false;
	}

	std::ofstream cache; // Open a new output stream to the cache
	if (use_bson_format)
		cache.open(data_cache_file, std::ofstream::out | std::ofstream::binary | std::ofstream::trunc); // If in bson format, open in binary mode
	else
		cache.open(data_cache_file, std::ofstream::out | std::ofstream::trunc); // Otherwise use plaintext

	if (!cache.fail()) {
		me::utility::log("Writing cache to disk...");
		// Write contents of cache_data to cache file, including errors
		if (use_bson_format) {
			std::vector<uchar> bson_data = json::to_bson(cache_data);
			cache.write((char*)bson_data.data(), bson_data.size());
		}
		else
			cache << cache_data.dump(4);
		me::utility::log("Analysis complete.");
		return cache_data["success"];
	}

	return false; // Default exit behavior
}

bool triangulate_points(char* data_cache_paths, char* target_data_groups, char* ground_plane_target, float marker_length, char* output_path, bool use_bson_format, double board_threshold, double marker_threshold, double scene_fps) {
	
	try {

		std::string arg1(data_cache_paths);
		std::string arg2(target_data_groups);
		std::string out_path(output_path);
		std::vector<std::string> paths;
		std::vector<std::string> targets;

		std::ofstream scene; // Open a new output stream to the cache
		if (use_bson_format)
			scene.open(out_path, std::ofstream::out | std::ofstream::binary | std::ofstream::trunc); // If in bson format, open in binary mode
		else
			scene.open(out_path, std::ofstream::out | std::ofstream::trunc); // Otherwise use plaintext

		if (scene.fail())
			throw std::runtime_error("Could not access output path \"" + out_path + "\"");

		// Splitting paths arg
		std::stringstream ss;
		for (int i = 0; i < arg1.size(); i++) {
			if (ss.str().size() > 0 && arg1[i] == delim) {
				paths.push_back(ss.str());
				ss.str(std::string());
			}
			else
				ss << arg1[i];
		}
		if (ss.str().size() > 0)
			paths.push_back(ss.str());

		// Splitting targets arg
		ss.str(std::string());
		for (int i = 0; i < arg2.size(); i++) {
			if (ss.str().size() > 0 && arg2[i] == delim) {
				targets.push_back(ss.str());
				ss.str(std::string());
			}
			else
				ss << arg2[i];
		}
		if (ss.str().size() > 0)
			targets.push_back(ss.str());

		std::map<std::string, me::tracking::TrackingData> datasets;
		for (auto it = paths.begin(); it != paths.end(); it++) {
			me::utility::log("Loading cache \"" + *it + "\"...");
			json cache_data;
			std::ifstream cache_json(*it, std::ifstream::in);
			if (cache_json.is_open()) { // Check if exists
				cache_data = json::parse(cache_json, nullptr, false, true); // Parse as json
				if (cache_data.is_discarded()) { // Secondary check as bson if parse failed
					cache_data = json();
					cache_json.close();
					cache_json.open(*it, std::ifstream::in | std::ifstream::binary); // Load as binary file
					if (cache_json.is_open()) { // Check AGAIN just in case it really doesn't exist
						cache_data = json::from_bson(cache_json, true, false); // Parse as bson
						if (cache_data.is_discarded()) {// Shoot it can't be read. Just make a new file I guess
							std::string msg = "The file \"";
							msg += *it;
							msg += "\" could not be read";
							throw std::runtime_error(msg);
						}
					}
				}
			}
			me::utility::log("Cache file \"" + *it + "\" loaded.");
			if (cache_json.is_open()) // Ensure the cache file is closed since we won't need it anymore
				cache_json.close();
			me::utility::log("Parsing cache data...");
			me::CacheData data;
			data.from_json(cache_data);
			if (!data.is_defined()) {
				std::string msg = "The file \"";
				msg += *it;
				msg += "\" was not recognized as a valid analysis file";
				throw std::runtime_error(msg);
			} 
			me::utility::log("Cache data parsed.");
			me::utility::log("Adding tracking groups to target groups...");
			// THIS NEEDS TO CHECK FOR PROPER VERSIONING AND ALSO FIX from_json IN motion_engine.cpp FOR TrackingData
			// IT DOESN'T DO THAT RN SO THATS WHY IM MENTIONING IT HERE RETARD
			// WE DID SPEND A CONSIDERABLE CHUNK OF TIME MAKING SURE WE HAD A SYSTEM FOR THAT IN THE CACHE FILE STRUCTURE
			// DON'T KNOW WHY WE WOULD JUST, YA KNOW, NOT ACTUALLY USE IT
			std::vector<std::string> purge_list;
			for (auto itt = targets.begin(); itt != targets.end(); itt++) {
				if (data.has_tracking_group_name(*itt)) {
					me::tracking::TrackingData tracking_data = data.get_tracking_data(*itt);
					if (!tracking_data.is_defined()) {
						std::string msg = "Target dataset \"";
						msg += *itt;
						msg += "\" was found in file \"";
						msg += *it;
						msg += "\" but failed an integrity check";
						throw std::runtime_error(msg);
					}
					purge_list.push_back(*itt);
					datasets.emplace(*itt, tracking_data);
				}
			}
			for (auto itt = purge_list.begin(); itt != purge_list.end(); itt++) {
				targets.erase(std::remove(targets.begin(), targets.end(), *itt));
			}
			me::utility::log("Tracking groups added.");
		}

		me::utility::log("Generating pose interpolation curves...");
		// Prepare interpolation curves for charuco pose data
		// Do a pass after you're done to make sure there are no errors and possible exceptions get thrown when they are supposed to
		std::map<std::string, me::utility::sets::TransformInterpolationSetHermite> pose_curves;
		for (auto it = datasets.begin(); it != datasets.end(); it++) {
			if (it->second.get_calibration_data().is_defined() && it->second.get_calibration_data().get_sync_data().is_defined()) {
				std::vector<int> frame_ids = it->second.get_frame_ids();
				if (frame_ids.size() > 0) {
					me::utility::sets::TransformInterpolationSetHermite curve;
					for (auto itt = frame_ids.begin(); itt != frame_ids.end(); itt++) {
						if (*itt >= it->second.get_calibration_data().get_sync_data().get_frame_number() &&
							it->second.get_frame_data(*itt).get_pose_data().is_defined()) {
							cv::Mat tvec = it->second.get_frame_data(*itt).get_pose_data().get_charuco_tvec();
							cv::Mat rvec = it->second.get_frame_data(*itt).get_pose_data().get_charuco_rvec();
							/*
							cv::Mat R;
							cv::Rodrigues(rvec, R);
							R = R.t();
							tvec = -R * tvec;
							cv::Rodrigues(R, rvec);
							*/
							curve.addPos((1 / (double)it->second.get_frames_per_second()) * ((double)*itt - it->second.get_calibration_data().get_sync_data().get_frame_number()),
								tvec.ptr<double>(0)[0], tvec.ptr<double>(1)[0], tvec.ptr<double>(2)[0]);
							curve.addRot((1 / (double)it->second.get_frames_per_second()) * ((double)*itt - it->second.get_calibration_data().get_sync_data().get_frame_number()),
								rvec.ptr<double>(0)[0], rvec.ptr<double>(1)[0], rvec.ptr<double>(2)[0]);
						}
					}
#ifdef VERBOSE_DEBUG
					std::cout << curve.curvePosX()->getPoints().size() << " points loaded from group " << it->first << std::endl;
					std::vector<cv::Point2d *> points = curve.curvePosX()->getPoints();
					std::cout << '[' << it->first << "] t=" << "first" << " pos(" << curve.curvePosX()->getPoints()[0]->y << ',' << 
						curve.curvePosY()->getPoints()[0]->y << ',' << curve.curvePosZ()->getPoints()[0]->y << ") rot(" << 
						curve.curveRotX()->getPoints()[0]->y << ',' << curve.curveRotY()->getPoints()[0]->y << ',' << curve.curveRotZ()->getPoints()[0]->y << ')' << std::endl;
					for (int i = 0; i < points.size(); i++) {
						cv::Point3d pos = curve.solvePos(points[i]->x);
						cv::Point3d rot = curve.solveRot(points[i]->x);
						std::cout << '[' << it->first << "] t=" << points[i]->x << " pos(" << pos.x << ',' << pos.y << ',' << pos.z << ") rot(" << rot.x << ',' << rot.y << ',' << rot.z << ')' << std::endl;
					}
#endif
					pose_curves.emplace(it->first, curve);
				}
				else {
					me::utility::log("No pose data found in group " + it->first + ". Group will be ignored.", "WARN");
				}
			}
			else {
				me::utility::log("No calibration data found in group " + it->first + ". Group will be ignored.", "WARN");
			}
		}

		if (pose_curves.empty()) {
			throw std::runtime_error("No pose data present in provided datasets. Triangulation cannot continue.");
		}

		me::utility::log("Finding pose time brackets...");
		// Bracket pose times into ranges based on threshold
		std::map<std::string, std::vector<me::utility::Range>> pose_time_brackets;
		for (auto it = pose_curves.begin(); it != pose_curves.end(); it++) {
			std::vector<me::utility::Range> brackets;
			std::vector<cv::Point2d*> points = it->second.curvePosX()->getPoints();
			bool qualified = false;
			double min = 0;
			double max = 0;
			for (size_t i = 0; i < points.size() - 1; i++) {
				double time = points[i]->x;
				if (!qualified && points[i + 1]->x - points[i]->x <= board_threshold) {
					min = points[i]->x;
					max = points[i + 1]->x;
					qualified = true;
				}
				else {
					if (qualified) {
						if (points[i + 1]->x - points[i]->x <= board_threshold) {
							max = points[i + 1]->x;
						}
						else {
							brackets.push_back(me::utility::Range(min, max));
							qualified = false;
						}
					}
				}
			}
			if (qualified) {
				brackets.push_back(me::utility::Range(min, max));
			}
			if (brackets.empty()) {
				throw std::runtime_error("Pose data in group " + it->first + " is too noisy for the given threshold value. Consider using a larger threshold value.");
			}
			pose_time_brackets.emplace(it->first, brackets);
		}

		me::utility::log("Detecting time bracket overlap...");
		// Align bracket overlap
		std::map<std::string, std::map<std::string, me::utility::Range>> overlap_brackets;
		for (auto it = pose_time_brackets.begin(); it != pose_time_brackets.end(); it++) {
			std::map<std::string, me::utility::Range> overlap_sub_map;
			for (auto itt = pose_time_brackets.begin(); itt != pose_time_brackets.end(); itt++) {
				if (itt->first != it->first) {
					bool search = true;
					for (auto a = it->second.begin(); a != it->second.end() && search; a++) {
						for (auto b = itt->second.begin(); b != itt->second.end() && search; b++) {
							if (a->checkForOverlap(*b)) {
								overlap_sub_map.emplace(itt->first, a->getOverlapRange(*b));
								search = false;
							}
						}
					}
				}
			}
			if (overlap_sub_map.empty())
				me::utility::log("Pose data in group " + it->first + " does not overlap with any other groups. Group will be ignored.", "WARN");
			else
				overlap_brackets.emplace(it->first, overlap_sub_map);
		}

		// Pick base origin and determine shortest paths to base
		if (overlap_brackets.empty())
			throw std::runtime_error("No overlap points detected. Triangulation cannot continue.");

		std::string base;
		if (overlap_brackets.count(ground_plane_target) > 0)
			base = ground_plane_target;
		else
			base = overlap_brackets.begin()->first;

		me::utility::log("Constructing scene graph...");
		// Organizing overlap points as a graph
		me::utility::Graph<std::string> scene_graph;
		for (auto it = overlap_brackets.begin(); it != overlap_brackets.end(); it++) {
			for (auto itt = it->second.begin(); itt != it->second.end(); itt++) {
				scene_graph.addEdge(it->first, itt->first);
			}
		}
		if (!scene_graph.checkContinuity()) {
			throw std::runtime_error("Found discontinuity between overlap points. Triangulation cannot continue.");
		}

		cv::Point3d world_pos = pose_curves.at(base).solvePos(overlap_brackets.at(base).begin()->second.solve(0.5));
		cv::Point3d world_rot = pose_curves.at(base).solveRot(overlap_brackets.at(base).begin()->second.solve(0.5));

		std::set<std::string> scene_views = scene_graph.getVerticies();

		if(scene_views.size() < 2)
			throw std::runtime_error("Valid camera views is < 2. At least 2 valid camera views are required for triangulation");

		// Get world_pos and world_rot relative to camera views using space transformations
		me::utility::log("Transforming space and time...");

		std::map<std::string, cv::Point3d> view_coords;
		std::map<std::string, cv::Point3d> view_rotations;

		for (auto it = scene_views.begin(); it != scene_views.end(); it++) {
			if (*it != base) {
				cv::Point3d cam_pos = world_pos;
				cv::Point3d cam_rot = world_rot;
				std::stack<std::string> path = scene_graph.getVertexPathDFS(base, *it);
				std::string current = path.top();
				path.pop();
				while (!path.empty()) {
					std::string next = path.top();
					path.pop();
					cv::Point3d pos = pose_curves.at(current).solvePos(overlap_brackets.at(next).at(current).solve(0.5));
					cv::Point3d rot = pose_curves.at(current).solveRot(overlap_brackets.at(next).at(current).solve(0.5));
					cv::Mat R;
					cv::Rodrigues((cv::Mat)rot, R);
					R = R.t();
					cv::Mat tvec_pt = -R * (cv::Mat)pos;
					cv::Mat tvec_cam = R * (cv::Mat)cam_pos + tvec_pt;
					cv::Mat R_cam;
					cv::Rodrigues((cv::Vec<double, 3>)cam_rot, R_cam);
					R = R * R_cam;
					cv::Point3d pos_n = pose_curves.at(next).solvePos(overlap_brackets.at(next).at(current).solve(0.5));
					cv::Point3d rot_n = pose_curves.at(next).solveRot(overlap_brackets.at(next).at(current).solve(0.5));
					cv::Mat R_n;
					cv::Rodrigues((cv::Mat)rot_n, R_n);
					tvec_cam = R_n * tvec_cam + (cv::Mat)pos_n;
					R = R_n * R;
					cv::Mat rvec_cam;
					cv::Rodrigues(R, rvec_cam);
					cam_pos = (cv::Point3d)tvec_cam;
					cam_rot = (cv::Point3d)rvec_cam;
				}
				view_coords.emplace(*it, cam_pos);
				view_rotations.emplace(*it, cam_rot);
			}
		}

#ifdef VERBOSE_DEBUG
		std::cout << "Connections:" << std::endl;
		std::set<me::utility::Edge<std::string>> edges = scene_graph.getEdges();
		for (auto it = edges.begin(); it != edges.end(); it++) {
			cv::Point3d pos_a = pose_curves.at(it->getA()).solvePos(overlap_brackets.at(it->getA()).at(it->getB()).solve(0.5));
			cv::Point3d rot_a = pose_curves.at(it->getA()).solveRot(overlap_brackets.at(it->getA()).at(it->getB()).solve(0.5));
			cv::Point3d pos_b = pose_curves.at(it->getB()).solvePos(overlap_brackets.at(it->getA()).at(it->getB()).solve(0.5));
			cv::Point3d rot_b = pose_curves.at(it->getB()).solveRot(overlap_brackets.at(it->getA()).at(it->getB()).solve(0.5));
			std::cout << '(' << it->getA() << ", " << it->getB() << "): posA(" << pos_a.x << ',' << pos_a.y << ',' << pos_a.z << ") rotA(" << rot_a.x << ',' << rot_a.y << ',' << rot_a.z << ") posB("
				<< pos_b.x << ',' << pos_b.y << ',' << pos_b.z << ") rotB(" << rot_b.x << ',' << rot_b.y << ',' << rot_b.z << ')' << std::endl;
		}
		std::cout << "World origin:" << std::endl;
		std::cout << base << ": pos(" << world_pos.x << ',' << world_pos.y << ',' << world_pos.z << ") rot(" << world_rot.x << ',' << world_rot.y << ',' << world_rot.z << ')' << std::endl;

		std::cout << "Relative positions:" << std::endl;
		for (auto it = view_coords.begin(); it != view_coords.end(); it++) {
			cv::Point3d cam_pos = it->second;
			cv::Point3d cam_rot = view_rotations.at(it->first);
			std::cout << it->first << ": pos(" << cam_pos.x << ',' << cam_pos.y << ',' << cam_pos.z << ") rot(" << cam_rot.x << ',' << cam_rot.y << ',' << cam_rot.z << ')' << std::endl;
		}
#endif
		
		me::utility::log("Generating marker interpolation curves...");
		// Construct interpolation curves for points in 2D camera space
		//         group_id          dict_id      marker_id      point_no             interpolation_curve
		std::map<std::string, std::map<int, std::map<int, std::map<int, me::utility::sets::PointInterpolationSetHermite>>>> marker_point_curves;
		std::set<std::string> group_ids_final;
		std::set<int> dict_ids_final;
		std::set<int> marker_ids_final;

		for (auto gid = scene_views.begin(); gid != scene_views.end(); gid++) {
			std::map<int, std::map<int, std::map<int, me::utility::sets::PointInterpolationSetHermite>>> dict_map;
			me::tracking::TrackingData data = datasets.at(*gid);
			std::vector<int> frames = data.get_frame_ids();
			for (auto fid = frames.begin(); fid != frames.end(); fid++) {
				if (*fid >= data.get_calibration_data().get_sync_data().get_frame_number()
					&& data.get_frame_data(*fid).get_analysis_data().is_defined()
					&& data.get_frame_data(*fid).get_analysis_data().has_markers()) {
					std::vector<int> dict_ids = data.get_frame_data(*fid).get_analysis_data().get_dictionary_ids();
					for (auto did = dict_ids.begin(); did != dict_ids.end(); did++) {
						std::vector<int> marker_ids = data.get_frame_data(*fid).get_analysis_data().get_marker_ids(*did);
						for (auto mid = marker_ids.begin(); mid != marker_ids.end(); mid++) {

							// This could cause issues if the charuco markers aren't getting filtered out. If it does, add whatever necessary so that those markers are ignored
							std::vector<cv::Point2f> points = data.get_frame_data(*fid).get_analysis_data().get_marker_corners(*did, *mid);
							for (int i = 0; i < points.size(); i++) {
								if (dict_map.count(*did) == 0) {
									std::map<int, std::map<int, me::utility::sets::PointInterpolationSetHermite>> new_map;
									dict_map.emplace(*did, new_map);
								}
								if (dict_map.at(*did).count(*mid) == 0) {
									std::map<int, me::utility::sets::PointInterpolationSetHermite> new_map;
									dict_map.at(*did).emplace(*mid, new_map);
								}
								if (dict_map.at(*did).at(*mid).count(i) == 0) {
									me::utility::sets::PointInterpolationSetHermite new_curve;
									dict_map.at(*did).at(*mid).emplace(i, new_curve);
								}
								dict_map.at(*did).at(*mid).at(i).addPos((1 / (double)data.get_frames_per_second()) * ((double)*fid - data.get_calibration_data().get_sync_data().get_frame_number()), 
									points[i].x, points[i].y);
							}

							group_ids_final.emplace(*gid);
							dict_ids_final.emplace(*did);
							marker_ids_final.emplace(*mid);

						}
					}
				}
			}
			if (!dict_map.empty())
				marker_point_curves.emplace(*gid, dict_map);
			else
				me::utility::log("No markers found in group \"" + *gid + "\". Group will be ignored.");
		}

		if (marker_point_curves.empty()) {
			throw std::runtime_error("No markers from views in scene graph were detected. Triangulation cannot continue.");
		}

		me::utility::log("Calculating marker interpolation time brackets...");
		// Construct time brackets for point curves
		//         group_id          dict_id      marker_id      point_no         range_vector
		std::map<std::string, std::map<int, std::map<int, std::map<int, std::vector<me::utility::Range>>>>> marker_point_brackets;

		// Also determine start and end times for scene
		double t_min = 0;
		double t_max = 0;
		bool t_range_init = true;

		for (auto gmap = marker_point_curves.begin(); gmap != marker_point_curves.end(); gmap++) {
			for (auto dmap = gmap->second.begin(); dmap != gmap->second.end(); dmap++) {
				for (auto mmap = dmap->second.begin(); mmap != dmap->second.end(); mmap++) {
					for (auto pmap = mmap->second.begin(); pmap != mmap->second.end(); pmap++) {
						std::vector<me::utility::Range> brackets;
						std::vector<cv::Point2d*> points = pmap->second.curvePosX()->getPoints();
						bool qualified = false;
						double min = 0;
						double max = 0;
						for (size_t i = 0; i < points.size() - 1; i++) {
							double time = points[i]->x;
							if (!qualified && points[i + 1]->x - points[i]->x <= marker_threshold) {
								min = points[i]->x;
								max = points[i + 1]->x;
								qualified = true;
								if (t_range_init) {
									t_min = min;
									t_max = max;
									t_range_init = false;
								}
								else {
									if (min < t_min)
										t_min = min;
									if (max > t_max)
										t_max = max;
								}
							}
							else {
								if (qualified) {
									if (points[i + 1]->x - points[i]->x <= marker_threshold) {
										max = points[i + 1]->x;
										if (max > t_max)
											t_max = max;
									}
									else {
										brackets.push_back(me::utility::Range(min, max));
										qualified = false;
									}
								}
							}
						}
						if (qualified) {
							brackets.push_back(me::utility::Range(min, max));
						}
						if (!brackets.empty()) {
							if (marker_point_brackets.count(gmap->first) == 0) {
								std::map<int, std::map<int, std::map<int, std::vector<me::utility::Range>>>> dict_map;
								marker_point_brackets.emplace(gmap->first, dict_map);
							}
							if (marker_point_brackets.at(gmap->first).count(dmap->first) == 0) {
								std::map<int, std::map<int, std::vector<me::utility::Range>>> new_map;
								marker_point_brackets.at(gmap->first).emplace(dmap->first, new_map);
							}
							if (marker_point_brackets.at(gmap->first).at(dmap->first).count(mmap->first) == 0) {
								std::map<int, std::vector<me::utility::Range>> new_map;
								marker_point_brackets.at(gmap->first).at(dmap->first).emplace(mmap->first, new_map);
							}
							marker_point_brackets.at(gmap->first).at(dmap->first).at(mmap->first).emplace(pmap->first, brackets);
						}
					}
				}
			}
		}

		if(marker_point_brackets.empty())
			throw std::runtime_error("Marker data is too noisy for the given threshold value. Consider using a larger threshold value.");

		me::utility::log("Triangulating marker coordinates...");
		// FRAME TRIANGULATION FINALLY

		struct marker_pose {
			std::vector<cv::Point3d> corner_points;
			cv::Point3d center_point;
			cv::Point3d center_rot;
		};

		//     t_val         dict_id      marker_id    marker_pose
		std::map<double, std::map<int, std::map<int, marker_pose>>> marker_coordinates;

		double frame_time = 1 / scene_fps;

		for (double t = t_min; t <= t_max; t += frame_time) {
			for (auto did = dict_ids_final.begin(); did != dict_ids_final.end(); did++) {
				for (auto mid = marker_ids_final.begin(); mid != marker_ids_final.end(); mid++) {
					std::map<std::string, std::vector<cv::Point2d>> points;
					for (auto gid = group_ids_final.begin(); gid != group_ids_final.end(); gid++) {
						if (marker_point_brackets.count(*gid) > 0 && 
							marker_point_brackets.at(*gid).count(*did) > 0 && 
							marker_point_brackets.at(*gid).at(*did).count(*mid) > 0) {
							std::vector<me::utility::Range> ranges = marker_point_brackets.at(*gid).at(*did).at(*mid).at(0);
							bool found = false;
							for (auto it = ranges.begin(); it != ranges.end(); it++) {
								if (it->contains(t)) {
									found = true;
									break;
								}
							}
							if (found) {
								std::vector<cv::Point2d> corners;
								for (int p = 0; p < 4; p++) {
									cv::Point2d point = marker_point_curves.at(*gid).at(*did).at(*mid).at(p).solve(t);
									
									corners.push_back(point);
								}
								points.emplace(*gid, corners);
							}
						}
					}
					if (points.size() > 1) {
						std::vector<cv::Mat> proj_mat;
						std::vector<cv::Mat> points_tri;
						std::vector<cv::Point3d> marker_rot;
						for (auto pt = points.begin(); pt != points.end(); pt++) {
							cv::Point3d pos_cam;
							cv::Point3d rot_cam;
							if (pt->first == base) {
								pos_cam = world_pos;
								rot_cam = world_rot;
							}
							else {
								pos_cam = view_coords.at(pt->first);
								rot_cam = view_rotations.at(pt->first);
							}
							std::vector<std::vector<cv::Point2f>> corners;
							std::vector<cv::Point2f> c_points;
							for (int p = 0; p < 4; p++) {
								c_points.push_back(cv::Point2f((float)pt->second[p].x, (float)pt->second[p].y));
							}
							corners.push_back(c_points);
							std::vector<cv::Vec3d> rvecs_m, tvecs_m;
							cv::aruco::estimatePoseSingleMarkers(corners, marker_length, datasets.at(pt->first).get_calibration_data().get_camera_matrix(),
								datasets.at(pt->first).get_calibration_data().get_distortion_coefficients(), rvecs_m, tvecs_m);
							cv::Mat R_cam;
							cv::Rodrigues((cv::Mat)rot_cam, R_cam);
							cv::Mat t_cam = (cv::Mat)pos_cam;
							cv::Mat P_cam;
							cv::sfm::projectionFromKRt(datasets.at(pt->first).get_calibration_data().get_camera_matrix(), R_cam, t_cam, P_cam);
							proj_mat.push_back(P_cam);
							cv::Mat points_cam(2, 4, CV_64FC1);
							for (int p = 0; p < 4; p++) {
								points_cam.ptr<double>(0, p)[0] = pt->second[p].x;
								points_cam.ptr<double>(1, p)[0] = pt->second[p].y;
							}
							
							cv::Mat undistort;
							cv::undistortImagePoints(points_cam, undistort, datasets.at(pt->first).get_calibration_data().get_camera_matrix(), datasets.at(pt->first).get_calibration_data().get_distortion_coefficients());
							for (int p = 0; p < 4; p++) {
								points_cam.ptr<double>(0, p)[0] = undistort.ptr<double>(0, p)[0];
								points_cam.ptr<double>(1, p)[0] = undistort.ptr<double>(0, p)[1];
							}
							
							points_tri.push_back(points_cam);
							cv::Mat R_m;
							cv::Rodrigues(rvecs_m[0], R_m);
							R_m = R_m * R_cam.t();
							cv::Vec3d m_rot_w;
							cv::Rodrigues(R_m, m_rot_w);
							marker_rot.push_back(cv::Point3d(m_rot_w(0), m_rot_w(1), m_rot_w(2)));
						}
						cv::Mat points_3d;
						cv::sfm::triangulatePointsHZ(points_tri, proj_mat, points_3d);
						marker_pose pose_m;
						cv::Point3d center(0, 0, 0);
						cv::Point3d rot(0, 0, 0);
						cv::Mat R_base;
						cv::Rodrigues((cv::Mat)world_rot, R_base);
						R_base = R_base.t();
						cv::Mat tvec_base = -R_base * (cv::Mat)world_pos;
						cv::Point3d t_base = (cv::Point3d)tvec_base;
						R_base = R_base.t();
						if (marker_coordinates.count(t) == 0) {
							std::map<int, std::map<int, marker_pose>> new_map;
							marker_coordinates.emplace(t, new_map);
						}
						if (marker_coordinates.at(t).count(*did) == 0) {
							std::map<int, marker_pose> new_map;
							marker_coordinates.at(t).emplace(*did, new_map);
						}
						for (int p = 0; p < 4; p++) {
							cv::Point3d point_3d;
							point_3d.x = points_3d.ptr<double>(0, p)[0];
							point_3d.y = points_3d.ptr<double>(1, p)[0];
							point_3d.z = points_3d.ptr<double>(2, p)[0];
							center.x += point_3d.x / 4;
							center.y += point_3d.y / 4;
							center.z += point_3d.z / 4;
							point_3d.x -= t_base.x;
							point_3d.y -= t_base.y;
							point_3d.z -= t_base.z;
							point_3d = (cv::Point3d)((cv::Mat)(R_base * (cv::Mat)point_3d));
							pose_m.corner_points.push_back(point_3d);
						}
						for (auto m_rot = marker_rot.begin(); m_rot != marker_rot.end(); m_rot++) {
							rot.x += m_rot->x / marker_rot.size();
							rot.y += m_rot->y / marker_rot.size();
							rot.z += m_rot->z / marker_rot.size();
						}
						cv::Mat R_m;
						cv::Rodrigues((cv::Mat)rot, R_m);
						R_m = R_m * R_base;
						cv::Vec3d m_rot;
						cv::Rodrigues(R_m, m_rot);
						rot.x = m_rot(0);
						rot.y = m_rot(1);
						rot.z = m_rot(2);
						center.x -= t_base.x;
						center.y -= t_base.y;
						center.z -= t_base.z;
						center = (cv::Point3d)((cv::Mat)(R_base * (cv::Mat)center));
						pose_m.center_point = center;
						pose_m.center_rot = rot;
						marker_coordinates.at(t).at(*did).emplace(*mid, pose_m);
					}
				}
			}
		}

		if (marker_coordinates.empty())
			throw std::runtime_error("No points could be calculated at specified frame rate.");

		me::utility::log("Writing scene to \"" + out_path + "\"...");

		json scene_data;

		scene_data["target_paths"] = arg1;
		scene_data["target_groups"] = arg2;
		scene_data["last_run"] = me::utility::currentDateTime();

		for (auto tmap = marker_coordinates.begin(); tmap != marker_coordinates.end(); tmap++) {
			for (auto dmap = tmap->second.begin(); dmap != tmap->second.end(); dmap++) {
				for (auto mmap = dmap->second.begin(); mmap != dmap->second.end(); mmap++) {
					for (int c = 0; c < 4; c++) {
						scene_data["marker_coordinates"][std::to_string(tmap->first)][me::utility::dictEnumToString(dmap->first)][std::to_string(mmap->first)]["corners"][c][0] = mmap->second.corner_points[c].x;
						scene_data["marker_coordinates"][std::to_string(tmap->first)][me::utility::dictEnumToString(dmap->first)][std::to_string(mmap->first)]["corners"][c][1] = mmap->second.corner_points[c].y;
						scene_data["marker_coordinates"][std::to_string(tmap->first)][me::utility::dictEnumToString(dmap->first)][std::to_string(mmap->first)]["corners"][c][2] = mmap->second.corner_points[c].z;
					}
					scene_data["marker_coordinates"][std::to_string(tmap->first)][me::utility::dictEnumToString(dmap->first)][std::to_string(mmap->first)]["tvec"][0] = mmap->second.center_point.x;
					scene_data["marker_coordinates"][std::to_string(tmap->first)][me::utility::dictEnumToString(dmap->first)][std::to_string(mmap->first)]["tvec"][1] = mmap->second.center_point.y;
					scene_data["marker_coordinates"][std::to_string(tmap->first)][me::utility::dictEnumToString(dmap->first)][std::to_string(mmap->first)]["tvec"][2] = mmap->second.center_point.z;
					scene_data["marker_coordinates"][std::to_string(tmap->first)][me::utility::dictEnumToString(dmap->first)][std::to_string(mmap->first)]["rvec"][0] = mmap->second.center_rot.x;
					scene_data["marker_coordinates"][std::to_string(tmap->first)][me::utility::dictEnumToString(dmap->first)][std::to_string(mmap->first)]["rvec"][1] = mmap->second.center_rot.y;
					scene_data["marker_coordinates"][std::to_string(tmap->first)][me::utility::dictEnumToString(dmap->first)][std::to_string(mmap->first)]["rvec"][2] = mmap->second.center_rot.z;
				}
			}
		}

		for (auto gmap = marker_point_brackets.begin(); gmap != marker_point_brackets.end(); gmap++) {
			cv::Point3d pos;
			cv::Point3d rot;
			if (gmap->first == base) {
				pos = world_pos;
				rot = world_rot;
			}
			else {
				pos = view_coords.at(gmap->first);
				rot = view_rotations.at(gmap->first);
			}
			scene_data["camera_coordinates"][gmap->first]["tvec"][0] = pos.x;
			scene_data["camera_coordinates"][gmap->first]["tvec"][1] = pos.y;
			scene_data["camera_coordinates"][gmap->first]["tvec"][2] = pos.z;
			scene_data["camera_coordinates"][gmap->first]["rvec"][0] = rot.x;
			scene_data["camera_coordinates"][gmap->first]["rvec"][1] = rot.y;
			scene_data["camera_coordinates"][gmap->first]["rvec"][2] = rot.z;
		}

		if (use_bson_format) {
			std::vector<uchar> bson_data = json::to_bson(scene_data);
			scene.write((char*)bson_data.data(), bson_data.size());
		}
		else
			scene << scene_data.dump(4);


		me::utility::log("Triangulation complete.");

		return true;

	}
	catch (cv::Exception& e) {
		std::string what(e.what());
		me::utility::log(what, "ERROR");
	}
	catch (const std::exception& ex) {
		std::string what(ex.what());
		me::utility::log(what, "ERROR");
	}
	return false;
}

bool repose_with_target(char* data_cache_paths, char* pose_groups, char* calibration_groups) {
	try {

		me::utility::log("Starting pose recalculation...");

		std::string arg1(data_cache_paths);
		std::string arg2(pose_groups);
		std::string arg3(calibration_groups);
		std::vector<std::string> paths;
		std::vector<std::string> p_groups;
		std::vector<std::string> c_groups;
		std::stringstream ss;

		// Get data cache paths
		for (int i = 0; i < arg1.size(); i++) {
			if (ss.str().size() > 0 && arg1[i] == delim) {
				paths.push_back(ss.str());
				ss.str(std::string());
			}
			else
				ss << arg1[i];
		}
		if (ss.str().size() > 0)
			paths.push_back(ss.str());

		// Get pose groups
		ss.str(std::string());
		for (int i = 0; i < arg2.size(); i++) {
			if (ss.str().size() > 0 && arg2[i] == delim) {
				p_groups.push_back(ss.str());
				ss.str(std::string());
			}
			else
				ss << arg2[i];
		}
		if (ss.str().size() > 0)
			p_groups.push_back(ss.str());

		// Get calibration groups
		ss.str(std::string());
		for (int i = 0; i < arg3.size(); i++) {
			if (ss.str().size() > 0 && arg3[i] == delim) {
				c_groups.push_back(ss.str());
				ss.str(std::string());
			}
			else
				ss << arg3[i];
		}
		if (ss.str().size() > 0)
			c_groups.push_back(ss.str());
		ss.str(std::string());

		if (p_groups.size() != c_groups.size())
			throw std::runtime_error("Must specify an equal number of pose and calibration groups");

		if(p_groups.empty())
			throw std::runtime_error("No groups specified for operation");

		if(paths.empty())
			throw std::runtime_error("No cache paths specified for operation");

		std::vector<std::string> targets;
		for (auto it = p_groups.begin(); it != p_groups.end(); it++) {
			targets.push_back(*it);
		}
		for (auto it = c_groups.begin(); it != c_groups.end(); it++) {
			targets.push_back(*it);
		}

		std::map<std::string, me::tracking::TrackingData> datasets;
		std::map<std::string, std::string> out_map;
		std::map<std::string, bool> use_bson;

		for (auto it = paths.begin(); it != paths.end(); it++) {
			me::utility::log("Loading cache \"" + *it + "\"...");
			use_bson.emplace(*it, false);
			json cache_data;
			std::ifstream cache_json(*it, std::ifstream::in);
			if (cache_json.is_open()) { // Check if exists
				cache_data = json::parse(cache_json, nullptr, false, true); // Parse as json
				if (cache_data.is_discarded()) { // Secondary check as bson if parse failed
					cache_data = json();
					cache_json.close();
					cache_json.open(*it, std::ifstream::in | std::ifstream::binary); // Load as binary file
					if (cache_json.is_open()) { // Check AGAIN just in case it really doesn't exist
						cache_data = json::from_bson(cache_json, true, false); // Parse as bson
						if (cache_data.is_discarded()) {// Shoot it can't be read. Just make a new file I guess
							std::string msg = "The file \"";
							msg += *it;
							msg += "\" could not be read";
							throw std::runtime_error(msg);
						}
						use_bson.at(*it) = true;
					}
				}
			}
			me::utility::log("Cache file \"" + *it + "\" loaded.");
			if (cache_json.is_open()) // Ensure the cache file is closed since we won't need it anymore
				cache_json.close();
			me::utility::log("Parsing cache data...");
			me::CacheData data;
			data.from_json(cache_data);
			if (!data.is_defined()) {
				std::string msg = "The file \"";
				msg += *it;
				msg += "\" was not recognized as a valid analysis file";
				throw std::runtime_error(msg);
			}
			me::utility::log("Cache data parsed.");
			me::utility::log("Adding tracking groups to target groups...");
			// THIS NEEDS TO CHECK FOR PROPER VERSIONING AND ALSO FIX from_json IN motion_engine.cpp FOR TrackingData
			// IT DOESN'T DO THAT RN SO THATS WHY IM MENTIONING IT HERE RETARD
			// WE DID SPEND A CONSIDERABLE CHUNK OF TIME MAKING SURE WE HAD A SYSTEM FOR THAT IN THE CACHE FILE STRUCTURE
			// DON'T KNOW WHY WE WOULD JUST, YA KNOW, NOT ACTUALLY USE IT
			std::vector<std::string> purge_list;
			for (auto itt = targets.begin(); itt != targets.end(); itt++) {
				if (data.has_tracking_group_name(*itt)) {
					me::tracking::TrackingData tracking_data = data.get_tracking_data(*itt);
					if (!tracking_data.is_defined()) {
						std::string msg = "Target dataset \"";
						msg += *itt;
						msg += "\" was found in file \"";
						msg += *it;
						msg += "\" but failed an integrity check";
						throw std::runtime_error(msg);
					}
					purge_list.push_back(*itt);
					bool corners = false;
					if (std::find(p_groups.begin(), p_groups.end(), *itt) != p_groups.end() && tracking_data.has_frames()) {
						std::vector<int> frames = tracking_data.get_frame_ids();
						for (auto fid = frames.begin(); fid != frames.end(); fid++) {
							me::tracking::FrameData f_data = tracking_data.get_frame_data(*fid);
							if (f_data.get_analysis_data().is_defined() && f_data.get_analysis_data().has_charuco_corners() && f_data.get_pose_data().is_defined()) {
								corners = true;
								break;
							}
						}
					}
					if (corners || (std::find(c_groups.begin(), c_groups.end(), *itt) != c_groups.end() && tracking_data.get_calibration_data().is_defined())) {
						datasets.emplace(*itt, tracking_data);
						if (corners)
							out_map.emplace(*itt, *it);
					}
					else
						me::utility::log("Group \"" + *itt + "\" contains no relevant data. Group will be ignored", "WARN");
				}
			}
			for (auto itt = purge_list.begin(); itt != purge_list.end(); itt++) {
				targets.erase(std::remove(targets.begin(), targets.end(), *itt));
			}
		}

		me::utility::log("Tracking groups added.");

		std::map<std::string, std::string> group_mappings;
		for (size_t i = 0; i < p_groups.size(); i++) {
			if (datasets.count(p_groups[i]) != 0 && datasets.count(c_groups[i]) != 0) {
				me::tracking::TrackingData p_data = datasets.at(p_groups[i]);
				me::tracking::TrackingData c_data = datasets.at(c_groups[i]);
				bool valid = false;
				if (p_data.get_calibration_data().is_defined()) {
					me::calibration::CharucoProperties prop1 = p_data.get_calibration_data().get_charuco_properties();
					me::calibration::CharucoProperties prop2 = c_data.get_calibration_data().get_charuco_properties();
					valid = prop1.is_defined() && prop2.is_defined() && 
						prop1.get_dictionary() == prop2.get_dictionary() && 
						prop1.get_marker_length() == prop2.get_marker_length() && 
						prop1.get_size() == prop2.get_size() && prop1.get_square_length() == 
						prop2.get_square_length();
				}
				if(valid)
					group_mappings.emplace(p_groups[i], c_groups[i]);
			}
		}

		if(group_mappings.empty())
			throw std::runtime_error("No valid tracking data found");

		for (auto gpair = group_mappings.begin(); gpair != group_mappings.end(); gpair++) {
			me::utility::log("Recalculating pose data in group \"" + gpair->first + "\" using calibration data from group \"" + gpair->second + "\"...");
			json cache_data;
			std::ifstream cache_json(out_map.at(gpair->first), std::ifstream::in);
			if (cache_json.is_open()) { // Check if exists
				cache_data = json::parse(cache_json, nullptr, false, true); // Parse as json
				if (cache_data.is_discarded()) { // Secondary check as bson if parse failed
					cache_data = json();
					cache_json.close();
					cache_json.open(out_map.at(gpair->first), std::ifstream::in | std::ifstream::binary); // Load as binary file
					if (cache_json.is_open()) { // Check AGAIN just in case it really doesn't exist
						cache_data = json::from_bson(cache_json, true, false); // Parse as bson
						if (cache_data.is_discarded()) {// Shoot it can't be read. Just make a new file I guess
							std::string msg = "The file \"";
							msg += out_map.at(gpair->first);
							msg += "\" could not be read";
							throw std::runtime_error(msg);
						}
						use_bson.at(out_map.at(gpair->first)) = true;
					}
				}
			}
			if(cache_json.is_open())
				cache_json.close();
			me::tracking::TrackingData data = datasets.at(gpair->first);
			std::vector<int> frames = data.get_frame_ids();
			me::calibration::CalibrationData c_data = datasets.at(gpair->second).get_calibration_data();
			cv::Mat cam_mtx = c_data.get_camera_matrix();
			cv::Mat cam_dist = c_data.get_distortion_coefficients();
			me::calibration::CharucoProperties c_props = c_data.get_charuco_properties();
			cv::Ptr<cv::aruco::CharucoBoard> board = cv::aruco::CharucoBoard::create(c_props.get_size().width, c_props.get_size().height,
				c_props.get_square_length(), c_props.get_marker_length(), cv::aruco::getPredefinedDictionary(c_props.get_dictionary()));

			// Write new camera matrix to cache
			for (int i = 0; i < cam_mtx.rows; i++) {
				for (int j = 0; j < cam_mtx.cols; j++) {
					cache_data[gpair->first]["calibration_data"]["camera_matrix"][i][j] = cam_mtx.ptr<double>(i)[j];
				}
			}
			for (int i = 0; i < cam_dist.cols; i++) {
				cache_data[gpair->first]["calibration_data"]["distortion_coefficients"][i] = cam_dist.ptr<double>(0)[i];
			}

			for (auto fid = frames.begin(); fid != frames.end(); fid++) {
				me::tracking::FrameData f_data = data.get_frame_data(*fid);
				if (f_data.get_analysis_data().is_defined() && f_data.get_analysis_data().has_charuco_corners()) {
					std::vector<int> board_ids = f_data.get_analysis_data().get_charuco_corner_ids();
					std::vector<cv::Point2f> board_corners;
					for (auto corner = board_ids.begin(); corner != board_ids.end(); corner++) {
						board_corners.push_back(f_data.get_analysis_data().get_charuco_corner(*corner));
					}
					cv::Mat rvec;
					cv::Mat tvec;
					cv::aruco::estimatePoseCharucoBoard(board_corners, board_ids, board, cam_mtx, cam_dist, rvec, tvec);

					// Further refinement
					for (int i = 0; i < 10; i++) {
						cv::aruco::estimatePoseCharucoBoard(board_corners, board_ids, board, cam_mtx, cam_dist, rvec, tvec, true);
					}

					for (int j = 0; j < tvec.rows; j++) {
						cache_data[gpair->first]["frames"][std::to_string(*fid)]["pose_data"]["charuco_board"]["tvec"][j] = tvec.ptr<double>(j)[0];
					}
					for (int j = 0; j < rvec.rows; j++) {
						cache_data[gpair->first]["frames"][std::to_string(*fid)]["pose_data"]["charuco_board"]["rvec"][j] = rvec.ptr<double>(j)[0];
					}
				}
			}

			me::utility::log("Writing results to disk...");

			std::ofstream cache; // Open a new output stream to the cache
			if (use_bson.at(out_map.at(gpair->first)))
				cache.open(out_map.at(gpair->first), std::ofstream::out | std::ofstream::binary | std::ofstream::trunc); // If in bson format, open in binary mode
			else
				cache.open(out_map.at(gpair->first), std::ofstream::out | std::ofstream::trunc); // Otherwise use plaintext

			if (cache.fail())
				throw std::runtime_error("Could not access output path \"" + out_map.at(gpair->first) + "\"");

			if (use_bson.at(out_map.at(gpair->first))) {
				std::vector<uchar> bson_data = json::to_bson(cache_data);
				cache.write((char*)bson_data.data(), bson_data.size());
			}
			else
				cache << cache_data.dump(4);

		}

		me::utility::log("Pose recalculation complete.");

		return true;

	}
	catch (cv::Exception& e) {
		std::string what(e.what());
		me::utility::log(what, "ERROR");
	}
	catch (const std::exception& ex) {
		std::string what(ex.what());
		me::utility::log(what, "ERROR");
	}
	return false;
}

void set_schema_directory(char* schema_dir) {
	schemas.clear();
	schemas.append(schema_dir);
}

void extern_create_window(char* window_name) {
	try {
		cv::namedWindow(window_name, cv::WINDOW_NORMAL);
	}
	catch (const std::exception& ex) {
		me::utility::log(ex.what(), "ERROR");
	}
}

void extern_destroy_window(char* window_name) {
	try {
		cv::destroyWindow(window_name);
	}
	catch (const std::exception& ex) {
		me::utility::log(ex.what(), "ERROR");
	}
}

void extern_show_image(char* window_name, uint8_t* img_mat_r, uint8_t* img_mat_g, uint8_t* img_mat_b, uint8_t* img_mat_a, bool alpha, int rows, int cols, bool flip_x, bool flip_y) {
	try {
		cv::Mat img;
		if(alpha)
			img = cv::Mat(rows, cols, CV_8UC4);
		else
			img = cv::Mat(rows, cols, CV_8UC3);
		std::vector<cv::Mat> channels;
		channels.push_back(cv::Mat(rows, cols, CV_8UC1, img_mat_b));
		channels.push_back(cv::Mat(rows, cols, CV_8UC1, img_mat_g));
		channels.push_back(cv::Mat(rows, cols, CV_8UC1, img_mat_r));
		if(alpha)
			channels.push_back(cv::Mat(rows, cols, CV_8UC1, img_mat_a));
		cv::merge(channels, img);
		if (flip_x) {
			cv::Mat img_flip;
			cv::flip(img, img_flip, 0);
			img = img_flip;
		}
		if (flip_y) {
			cv::Mat img_flip;
			cv::flip(img, img_flip, 1);
			img = img_flip;
		}
		cv::imshow(window_name, img);
	}
	catch (const std::exception& ex) {
		me::utility::log(ex.what(), "ERROR");
	}
}

void extern_destroy_windows() {
	try {
		cv::destroyAllWindows();
	}
	catch (const std::exception& ex) {
		me::utility::log(ex.what(), "ERROR");
	}
}

void getQuaternion(cv::Mat R, double Q[])
{
	double trace = R.at<double>(0, 0) + R.at<double>(1, 1) + R.at<double>(2, 2);

	if (trace > 0.0)
	{
		double s = sqrt(trace + 1.0);
		Q[3] = (s * 0.5);
		s = 0.5 / s;
		Q[0] = ((R.at<double>(2, 1) - R.at<double>(1, 2)) * s);
		Q[1] = ((R.at<double>(0, 2) - R.at<double>(2, 0)) * s);
		Q[2] = ((R.at<double>(1, 0) - R.at<double>(0, 1)) * s);
	}

	else
	{
		int i = R.at<double>(0, 0) < R.at<double>(1, 1) ? (R.at<double>(1, 1) < R.at<double>(2, 2) ? 2 : 1) : (R.at<double>(0, 0) < R.at<double>(2, 2) ? 2 : 0);
		int j = (i + 1) % 3;
		int k = (i + 2) % 3;

		double s = sqrt(R.at<double>(i, i) - R.at<double>(j, j) - R.at<double>(k, k) + 1.0);
		Q[i] = s * 0.5;
		s = 0.5 / s;

		Q[3] = (R.at<double>(k, j) - R.at<double>(j, k)) * s;
		Q[j] = (R.at<double>(j, i) + R.at<double>(i, j)) * s;
		Q[k] = (R.at<double>(k, i) + R.at<double>(i, k)) * s;
	}
}

void generate_synchronization_video(char* out_file, int dictionary, int marker_id, float size_ratio, int size, float delay_interval, int start_delays, int end_delays) {
	cv::VideoWriter output(out_file, cv::VideoWriter::fourcc('m', 'p', '4', 'v'), 60, cv::Size(size, size));
	if (output.isOpened()) {
		if (size_ratio > 1)
			size_ratio = 1 / size_ratio;
		int marker_size = (int)(size * size_ratio);
		cv::Mat mkr;
		int delay_frames = (int)(60 * delay_interval);
		cv::aruco::drawMarker(cv::aruco::getPredefinedDictionary(dictionary), marker_id, marker_size, mkr, 1);
		cv::Mat marker;
		cv::cvtColor(mkr, marker, cv::COLOR_GRAY2BGR);
		cv::Mat begin_frame(size, size, CV_8UC3, cv::Scalar(0, 0, 0));
		cv::Mat end_frame(size, size, CV_8UC3, cv::Scalar(0, 0, 0));
		cv::Mat marker_frame(size, size, CV_8UC3, cv::Scalar(255, 255, 255));
		int offset = (size - marker_size) / 2;
		cv::Rect roi(cv::Point(offset, offset), marker.size());
		marker.copyTo(marker_frame(roi));
		float display_scale = (float)size / 100;
		cv::Size clear_size = cv::getTextSize("STAND CLEAR", cv::FONT_HERSHEY_SIMPLEX, display_scale / 4, 1, 0);
		cv::Size sync_size = cv::getTextSize("Synchronization in progress", cv::FONT_HERSHEY_SIMPLEX, display_scale / 8, 1, 0);
		cv::Size complete_size = cv::getTextSize("SYNC COMPLETE", cv::FONT_HERSHEY_SIMPLEX, display_scale / 4, 1, 0);
		cv::Size remove_size = cv::getTextSize("This marker can now be removed", cv::FONT_HERSHEY_SIMPLEX, display_scale / 8, 1, 0);
		cv::putText(begin_frame, "STAND CLEAR", cv::Point((size - clear_size.width) / 2, (int)((size / 2) - 1 * display_scale)), cv::FONT_HERSHEY_SIMPLEX, display_scale / 4, cv::Scalar(0, 255, 0), 2);
		cv::putText(begin_frame, "Synchronization in progress", cv::Point((size - sync_size.width) / 2, (int)((size / 2) + 1 * display_scale + sync_size.height)), cv::FONT_HERSHEY_SIMPLEX, display_scale / 8, cv::Scalar(0, 255, 0), 2);
		cv::putText(end_frame, "SYNC COMPLETE", cv::Point((size - complete_size.width) / 2, (int)((size / 2) - 1 * display_scale)), cv::FONT_HERSHEY_SIMPLEX, display_scale / 4, cv::Scalar(0, 255, 0), 2);
		cv::putText(end_frame, "This marker can now be removed", cv::Point((size - remove_size.width) / 2, (int)((size / 2) + 1 * display_scale + remove_size.height)), cv::FONT_HERSHEY_SIMPLEX, display_scale / 8, cv::Scalar(0, 255, 0), 2);
		for (int i = 0; i < delay_frames * start_delays; i++) {
			output.write(begin_frame);
		}
		for (int i = 0; i < delay_frames; i++) {
			output.write(marker_frame);
		}
		for (int i = 0; i < delay_frames * end_delays; i++) {
			output.write(end_frame);
		}
	}
	output.release();
}

void computeDownsample(cv::Mat& source, cv::Mat& destination, bool resize, float downsample_scale, int downsample_width, int downsample_height, bool scale, bool width, bool height, bool& complete, json& cache_data) {
	cv::Size sample_size = cv::Size(source.cols, source.rows);
	if (resize) {
		cv::Size frame_size = cv::Size(source.cols, source.rows);
		int hcf = 0;
		int h = frame_size.height;
		int l = frame_size.width;
		if (h < l) {
			int t = h;
			h = l;
			l = t;
		}
		int rem = -1;
		while (rem != 0) {
			rem = h % l;
			if (rem == 0) {
				hcf = l;
				break;
			}
			h = l;
			l = rem;
		}
		int aspect_h = 1;
		int aspect_w = 1;
		if (rem > 0) {
			aspect_h = frame_size.height / rem;
			aspect_w = frame_size.width / rem;
		}
		double base_h = 0;
		double base_w = 0;
		if (scale) {
			base_h = frame_size.height * (double)downsample_scale;
			base_w = frame_size.width * (double)downsample_scale;
		}
		else if (width) {
			if (downsample_width > frame_size.width) {
				complete = false;
				cache_data["errors"]["bad_value"].push_back("downsample_width is greater than the source width");
				throw std::invalid_argument("received invalid config args");
			}
			double factor = downsample_width / frame_size.width;
			base_h = frame_size.height * factor;
			base_w = downsample_width;
		}
		else if (height) {
			if (downsample_height > frame_size.height) {
				complete = false;
				cache_data["errors"]["bad_value"].push_back("downsample_height is greater than the source height");
				throw std::invalid_argument("received invalid config args");
			}
			double factor = downsample_height / frame_size.height;
			base_h = downsample_height;
			base_w = frame_size.width * factor;
		}
		int trunc_h = (int)base_h;
		int trunc_w = (int)base_w;
		if (trunc_h < aspect_h || trunc_w < aspect_w) {
			sample_size.width = aspect_w;
			sample_size.height = aspect_h;
		}
		else if (trunc_h % aspect_h != 0 || trunc_w % aspect_w != 0) {
			int rem_h = trunc_h % aspect_h;
			int rem_w = trunc_w % aspect_w;
			int offset_h = aspect_h - rem_h;
			int offset_w = aspect_w - rem_w;
			trunc_h += offset_h;
			trunc_w += offset_w;
			if (trunc_h > frame_size.height || trunc_w > frame_size.width) {
				sample_size.width = frame_size.width;
				sample_size.height = frame_size.height;
			}
			else {
				sample_size.width = trunc_w;
				sample_size.height = trunc_h;
			}
		}
		else {
			sample_size.width = trunc_w;
			sample_size.height = trunc_h;
		}
	}
	cv::resize(source, destination, sample_size);
}

