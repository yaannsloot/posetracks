#ifdef MECORE_EXPORTS

#define MECORE __declspec(dllexport)

#else

#define MECORE __declspec(dllimport)

#endif

#include <nlohmann/json.hpp>
#include <opencv2/aruco.hpp>
#include <string>
#include <vector>
#include <istream>
#include <ostream>
#include <iostream>
#include <stack>
#include <set>
using json = nlohmann::json;

#ifndef MOTION_ENGINE_HPP
#define MOTION_ENGINE_HPP

namespace me {

	namespace calibration {

		class CharucoProperties {
		public:
			int get_dictionary() const;
			cv::Size2i get_size() const;
			float get_square_length() const;
			float get_marker_length() const;
			bool is_defined() const { return defined; }
			void to_json(json& j) const;
			void from_json(const json& j);
		private:
			// raw values
			std::string charuco_dictionary;
			int charuco_squares_x;
			int charuco_squares_y;
			float charuco_square_length;
			float charuco_marker_length;
			bool defined = false;
		};

		class MarkerProperties {
		public:
			int get_dictionary() const;
			float get_length() const;
			size_t get_total_ids() const;
			int get_id_at(unsigned int _index) const;
			bool is_defined() const { return defined; }
			void to_json(json& j) const;
			void from_json(const json& j);
		private:
			// raw values
			std::string marker_dictionary;
			float marker_length;
			std::vector<int> marker_ids;
			bool defined = false;
		};

		class SyncProperties {
		public:
			int get_dictionary() const;
			int get_id() const;
			bool is_defined() const { return defined; }
			void to_json(json& j) const;
			void from_json(const json& j);
		private:
			// raw values
			std::string marker_dictionary;
			int marker_id;
			bool defined = false;
		};

		class PatternDimensions {
		public:
			cv::Size2i get_size() const;
			bool is_defined() const { return defined; }
			void to_json(json& j) const;
			void from_json(const json& j);
		private:
			// raw values
			int width;
			int height;
			bool defined = false;
		};

		class PredefinedRange {
		public:
			int get_begin() const;
			int get_end() const;
			bool is_defined() const { return defined; }
			void to_json(json& j) const;
			void from_json(const json& j);
		private:
			// raw values
			int begin;
			int end;
			bool defined = false;
		};

		// CharucoProperties conversion wrapper functions
		void to_json(json& j, const CharucoProperties& p);
		void from_json(const json& j, CharucoProperties& p);

		// MarkerProperties conversion wrapper functions
		void to_json(json& j, const MarkerProperties& p);
		void from_json(const json& j, MarkerProperties& p);

		// SyncProperties conversion wrapper functions
		void to_json(json& j, const SyncProperties& p);
		void from_json(const json& j, SyncProperties& p);

		// PatternDimensions conversion wrapper functions
		void to_json(json& j, const PatternDimensions& p);
		void from_json(const json& j, PatternDimensions& p);

		// PredefinedRange conversion wrapper functions
		void to_json(json& j, const PredefinedRange& p);
		void from_json(const json& j, PredefinedRange& p);

		class SyncData {
		public:
			int get_frame_number() const;
			int get_dictionary() const;
			int get_marker_id() const;
			bool is_defined() const { return defined; }
			void to_json(json& j) const;
			void from_json(const json& j);
		private:
			// raw values
			int frame_number;
			std::string marker_dictionary;
			int marker_id;
			bool defined = false;
		};

		class CalibrationData {
		public:
			cv::Mat get_camera_matrix() const;
			cv::Mat get_distortion_coefficients() const;
			SyncData get_sync_data() const;
			CharucoProperties get_charuco_properties() const;
			bool is_defined() const { return defined; }
			void to_json(json& j) const;
			void from_json(const json& j);
		private:
			// raw values
			cv::Mat camera_matrix;
			cv::Mat distortion_coefficients;
			CharucoProperties charuco_properties;
			SyncData sync_data;
			bool defined = false;
		};

		// SyncData conversion wrapper functions
		void to_json(json& j, const SyncData& p);
		void from_json(const json& j, SyncData& p);

		// CalibrationData conversion wrapper functions
		void to_json(json& j, const CalibrationData& p);
		void from_json(const json& j, CalibrationData& p);

	}

	enum class SequenceType {
		SEQUENCE_VIDEO = 0,
		SEQUENCE_IMAGES = 1
	};

	class CalibrationConfig {
	public:
		calibration::CharucoProperties get_charuco_properties() const;
		calibration::MarkerProperties get_marker_properties() const;
		calibration::SyncProperties get_sync_properties() const;
		SequenceType get_sequence_type() const;
		size_t get_total_sequence_files() const;
		std::string get_sequence_file_at(unsigned int _index) const;
		bool do_use_threading() const;
		bool do_use_cuda() const;
		calibration::PatternDimensions get_pattern_dimensions() const;
		bool do_calibrate_camera() const;
		int get_calibration_frame_count() const;
		calibration::PredefinedRange get_calibration_range() const;
		calibration::PredefinedRange get_analysis_range() const;
		float get_downsample_scale() const;
		int get_downsample_width() const;
		int get_downsample_height() const;
		bool do_clear_cache() const;
		bool do_calculate_pose_means() const;
		bool do_downsampling() const;
		bool is_defined() const { return defined; }
		void to_json(json& j) const;
		void from_json(const json& j);
	private:
		// raw values
		calibration::CharucoProperties charuco_properties;
		calibration::MarkerProperties marker_properties;
		calibration::SyncProperties sync_properties;
		std::string sequence_type;
		std::vector<std::string> sequence_files;
		bool use_threading = true;
		bool use_cuda = true;
		calibration::PatternDimensions pattern_dimensions;
		bool calibrate_camera = true;
		int calibration_frames = 10;
		calibration::PredefinedRange calibration_range;
		calibration::PredefinedRange analysis_range;
		float downsample_scale = -1;
		int downsample_width = -1;
		int downsample_height = -1;
		bool clear_point_cache = false;
		bool use_pose_means = false;
		bool defined = false;
	};

	// CalibrationConfig conversion wrapper functions
	void to_json(json& j, const CalibrationConfig& p);
	void from_json(const json& j, CalibrationConfig& p);

	namespace tracking {

		class AnalysisData {
		public:
			cv::Point2f get_charuco_corner(int id) const;
			std::vector<int> get_charuco_corner_ids() const;
			bool has_charuco_corners() const;
			bool has_charuco_corner(int id) const;
			cv::Point2f get_marker_corner(int dict_id, int marker_id, int corner_id) const;
			std::vector<cv::Point2f> get_marker_corners(int dict_id, int marker_id) const;
			std::vector<int> get_marker_ids(int dict_id) const;
			std::vector<int> get_dictionary_ids() const;
			bool has_markers(int dict_id) const;
			bool has_markers() const;
			bool has_marker(int dict_id, int marker_id) const;
			bool is_defined() const { return defined; }
			void to_json(json& j) const;
			void from_json(const json& j);
		private:
			// raw values
			std::map<int, cv::Point2f> charuco_corners;
			std::map<int, std::map<int, std::vector<cv::Point2f>>> marker_corners;
			bool defined = false;
		};

		// AnalysisData conversion wrapper functions
		void to_json(json& j, const AnalysisData& p);
		void from_json(const json& j, AnalysisData& p);

		class PoseData {
		public:
			cv::Mat get_charuco_rvec() const;
			cv::Mat get_charuco_tvec() const;
			bool is_defined() const { return defined; }
			void to_json(json& j) const;
			void from_json(const json& j);
		private:
			cv::Mat board_rvec;
			cv::Mat board_tvec;
			bool defined = false;
			// raw values
		};

		// PoseData conversion wrapper functions
		void to_json(json& j, const PoseData& p);
		void from_json(const json& j, PoseData& p);

		class FrameData {
		public:
			AnalysisData get_analysis_data() const;
			PoseData get_pose_data() const;
			bool is_defined() const { return defined; }
			void to_json(json& j) const;
			void from_json(const json& j);
		private:
			// raw values
			AnalysisData analysis_data;
			PoseData pose_data;
			bool defined = false;
		};

		// FrameData conversion wrapper functions
		void to_json(json& j, const FrameData& p);
		void from_json(const json& j, FrameData& p);

		class TrackingData {
		public:
			FrameData get_frame_data(int frame_id) const;
			std::vector<int> get_frame_ids() const;
			bool has_frames() const;
			bool has_frame(int frame_id) const;
			calibration::CalibrationData get_calibration_data() const;
			std::string get_file_name() const;
			std::string get_runtime_ver() const;
			float get_frames_per_second() const;
			CalibrationConfig get_previous_config() const;
			bool is_defined() const { return defined; }
			void to_json(json& j) const;
			void from_json(const json& j);
		private:
			// raw values
			std::map<int, FrameData> frames;
			calibration::CalibrationData calibration_data;
			CalibrationConfig previous_config;
			std::string file_name;
			std::string runtime_ver;
			float frames_per_second;
			bool defined = false;
		};

		// TrackingData conversion wrapper functions
		void to_json(json& j, const TrackingData& p);
		void from_json(const json& j, TrackingData& p);

	}

	class CacheData {
	public:
		std::vector<std::string> get_tracking_group_names() const;
		tracking::TrackingData get_tracking_data(std::string group_id) const;
		bool has_tracking_group_name(std::string group_id) const;
		bool has_tracking_data() const;
		json get_errors() const;
		bool was_successful() const { return success; }
		bool is_defined() const { return defined; }
		void to_json(json& j) const;
		void from_json(const json& j);
	private:
		std::map<std::string, tracking::TrackingData> tracking_data;
		json errors;
		bool success = false;
		bool defined = false;
		// raw values
	};

	// CacheData conversion wrapper functions
	void to_json(json& j, const CacheData& p);
	void from_json(const json& j, CacheData& p);

}

#endif
