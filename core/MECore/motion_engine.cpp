#include <pch.h>
#include <motion_engine.hpp>
#include <fstream>
#include <buildkey_asset.h>
#include <iterator>
#include <stdexcept>
#include <algorithm>
#include <me_crypto.hpp>
#include <me_utils.hpp>

namespace me {

	namespace calibration {

		int CharucoProperties::get_dictionary() const {
			if (!defined)
				throw std::runtime_error("CharucoProperties not defined");
			return utility::dictStringToEnum(charuco_dictionary);
		}

		cv::Size2i CharucoProperties::get_size() const {
			if (!defined)
				throw std::runtime_error("CharucoProperties not defined");
			return cv::Size2i(charuco_squares_x, charuco_squares_y);
		}

		float CharucoProperties::get_square_length() const {
			if (!defined)
				throw std::runtime_error("CharucoProperties not defined");
			return charuco_square_length;
		}

		float CharucoProperties::get_marker_length() const {
			if (!defined)
				throw std::runtime_error("CharucoProperties not defined");
			return charuco_marker_length;
		}

		void CharucoProperties::to_json(json& j) const {
			if (!defined)
				throw std::runtime_error("CharucoProperties not defined");
			j = json{
				{"charuco_dictionary", charuco_dictionary},
				{"charuco_squares_x", charuco_squares_x},
				{"charuco_squares_y", charuco_squares_y},
				{"charuco_square_length", charuco_square_length},
				{"charuco_marker_length", charuco_marker_length}
			};
		}

		void CharucoProperties::from_json(const json& j) {
			if (j.contains("charuco_dictionary") && j.contains("charuco_squares_x") && j.contains("charuco_squares_y") && j.contains("charuco_square_length") && j.contains("charuco_marker_length")) {
				j.at("charuco_dictionary").get_to(charuco_dictionary);
				j.at("charuco_squares_x").get_to(charuco_squares_x);
				j.at("charuco_squares_y").get_to(charuco_squares_y);
				j.at("charuco_square_length").get_to(charuco_square_length);
				j.at("charuco_marker_length").get_to(charuco_marker_length);
				defined = true;
			}
			else
				defined = false;
		}

		int MarkerProperties::get_dictionary() const {
			if (!defined)
				throw std::runtime_error("MarkerProperties not defined");
			return utility::dictStringToEnum(marker_dictionary);
		}

		float MarkerProperties::get_length() const {
			if (!defined)
				throw std::runtime_error("MarkerProperties not defined");
			return marker_length;
		}

		size_t MarkerProperties::get_total_ids() const {
			if (!defined)
				throw std::runtime_error("MarkerProperties not defined");
			return marker_ids.size();
		}

		int MarkerProperties::get_id_at(unsigned int _index) const {
			if (!defined)
				throw std::runtime_error("MarkerProperties not defined");
			if (!(_index < marker_ids.size()))
				throw std::runtime_error("index out of range");
			return marker_ids[_index];
		}

		void MarkerProperties::to_json(json& j) const {
			if (!defined)
				throw std::runtime_error("MarkerProperties not defined");
			j = json{
				{"marker_dictionary", marker_dictionary},
				{"marker_length", marker_length},
				{"marker_ids", marker_ids}
			};
		}

		void MarkerProperties::from_json(const json& j) {
			if (j.contains("marker_dictionary") && j.contains("marker_length")) {
				j.at("marker_dictionary").get_to(marker_dictionary);
				j.at("marker_length").get_to(marker_length);
				if (j.contains("marker_ids"))
					j.at("marker_ids").get_to(marker_ids);
				defined = true;
			}
			else
				defined = false;
		}

		int SyncProperties::get_dictionary() const {
			if (!defined)
				throw std::runtime_error("SyncProperties not defined");
			return utility::dictStringToEnum(marker_dictionary);
		}

		int SyncProperties::get_id() const {
			if (!defined)
				throw std::runtime_error("SyncProperties not defined");
			return marker_id;
		}

		void SyncProperties::to_json(json& j) const {
			if (!defined)
				throw std::runtime_error("SyncProperties not defined");
			j = json{
				{"marker_dictionary", marker_dictionary},
				{"marker_id", marker_id}
			};
		}

		void SyncProperties::from_json(const json& j) {
			if (j.contains("marker_dictionary") && j.contains("marker_id")) {
				j.at("marker_dictionary").get_to(marker_dictionary);
				j.at("marker_id").get_to(marker_id);
				defined = true;
			}
			else
				defined = false;
		}

		cv::Size2i PatternDimensions::get_size() const {
			if (!defined)
				throw std::runtime_error("PatternDimensions not defined");
			return cv::Size2i(width, height);
		}

		void PatternDimensions::to_json(json& j) const {
			if (!defined)
				throw std::runtime_error("PatternDimensions not defined");
			j = json{
				{"width", width},
				{"height", height}
			};
		}

		void PatternDimensions::from_json(const json& j) {
			if (j.contains("width") && j.contains("height")) {
				j.at("width").get_to(width);
				j.at("height").get_to(height);
				defined = true;
			}
			else
				defined = false;
		}

		int PredefinedRange::get_begin() const {
			if (!defined)
				throw std::runtime_error("PredefinedRange not defined");
			return begin;
		}

		int PredefinedRange::get_end() const {
			if (!defined)
				throw std::runtime_error("PredefinedRange not defined");
			return end;
		}

		void PredefinedRange::to_json(json& j) const {
			if (!defined)
				throw std::runtime_error("PredefinedRange not defined");
			j = json{
				{"begin", begin},
				{"end", end}
			};
		}

		void PredefinedRange::from_json(const json& j) {
			if (j.contains("begin") && j.contains("end")) {
				j.at("begin").get_to(begin);
				j.at("end").get_to(end);
				defined = true;
			}
			else
				defined = false;
		}

		void to_json(json& j, const CharucoProperties& p) {
			p.to_json(j);
		}

		void from_json(const json& j, CharucoProperties& p) {
			p.from_json(j);
		}

		void to_json(json& j, const MarkerProperties& p) {
			p.to_json(j);
		}

		void from_json(const json& j, MarkerProperties& p) {
			p.from_json(j);
		}

		void to_json(json& j, const SyncProperties& p) {
			p.to_json(j);
		}

		void from_json(const json& j, SyncProperties& p) {
			p.from_json(j);
		}

		void to_json(json& j, const PatternDimensions& p) {
			p.to_json(j);
		}

		void from_json(const json& j, PatternDimensions& p) {
			p.from_json(j);
		}

		void to_json(json& j, const PredefinedRange& p) {
			p.to_json(j);
		}

		void from_json(const json& j, PredefinedRange& p) {
			p.from_json(j);
		}

		int SyncData::get_frame_number() const {
			if (!defined)
				throw std::runtime_error("SyncData not defined");
			return frame_number;
		}

		int SyncData::get_dictionary() const {
			if (!defined)
				throw std::runtime_error("SyncData not defined");
			return utility::dictStringToEnum(marker_dictionary);
		}

		int SyncData::get_marker_id() const {
			if (!defined)
				throw std::runtime_error("SyncData not defined");
			return marker_id;
		}

		void SyncData::to_json(json& j) const {
			if (!defined)
				throw std::runtime_error("SyncData not defined");
			j = json{
				{"frame_number", frame_number},
				{"marker_dictionary", marker_dictionary},
				{"marker_id", marker_id}
			};
		}

		void SyncData::from_json(const json& j) {
			if (j.contains("frame_number") && j.contains("marker_dictionary") && j.contains("marker_id")) {
				j.at("frame_number").get_to(frame_number);
				j.at("marker_dictionary").get_to(marker_dictionary);
				j.at("marker_id").get_to(marker_id);
				defined = true;
			}
		}

		cv::Mat CalibrationData::get_camera_matrix() const {
			if (!defined)
				throw std::runtime_error("CalibrationData not defined");
			return camera_matrix;
		}

		cv::Mat CalibrationData::get_distortion_coefficients() const {
			if (!defined)
				throw std::runtime_error("CalibrationData not defined");
			return distortion_coefficients;
		}

		SyncData CalibrationData::get_sync_data() const {
			if (!defined)
				throw std::runtime_error("CalibrationData not defined");
			return sync_data;
		}

		CharucoProperties CalibrationData::get_charuco_properties() const {
			if (!defined)
				throw std::runtime_error("CalibrationData not defined");
			return charuco_properties;
		}

		void CalibrationData::to_json(json& j) const {
			if (!defined)
				throw std::runtime_error("CalibrationData not defined");
			j = json{
				{"sync_data", sync_data},
				{"charuco_properties", charuco_properties}
			};
			for (int i = 0; i < camera_matrix.rows; i++) {
				for (int k = 0; k < camera_matrix.cols; k++) {
					j["camera_matrix"][i][k] = camera_matrix.ptr<double>(i)[k];
				}
			}
			for (int i = 0; i < distortion_coefficients.cols; i++) {
				j["distortion_coefficients"][i] = distortion_coefficients.ptr<double>(0)[i];
			}
		}

		void CalibrationData::from_json(const json& j) {
			if (j.contains("camera_matrix") && j.contains("distortion_coefficients") && j.contains("charuco_properties")) {
				if(j.contains("sync_data"))
					j.at("sync_data").get_to(sync_data);
				j.at("charuco_properties").get_to(charuco_properties);
				camera_matrix = cv::Mat((int)j.at("camera_matrix").size(), (int)j.at("camera_matrix").at(0).size(), CV_64FC1);
				distortion_coefficients = cv::Mat(1, (int)j.at("distortion_coefficients").size(), CV_64FC1);
				for (int i = 0; i < j.at("camera_matrix").size(); i++) {
					for (int k = 0; k < j.at("camera_matrix").at(0).size(); k++) {
						camera_matrix.ptr<double>(i)[k] = j.at("camera_matrix").at(i).at(k);
					}
				}
				for (int i = 0; i < j.at("distortion_coefficients").size(); i++) {
					distortion_coefficients.ptr<double>(0)[i] = j.at("distortion_coefficients").at(i);
				}
				defined = true;
			}
		}

		void to_json(json& j, const SyncData& p) {
			p.to_json(j);
		}

		void from_json(const json& j, SyncData& p) {
			p.from_json(j);
		}

		void to_json(json& j, const CalibrationData& p) {
			p.to_json(j);
		}

		void from_json(const json& j, CalibrationData& p) {
			p.from_json(j);
		}

	}

	calibration::CharucoProperties CalibrationConfig::get_charuco_properties() const {
		if (!defined)
			throw std::runtime_error("CalibrationConfig not defined");
		return charuco_properties;
	}

	calibration::MarkerProperties CalibrationConfig::get_marker_properties() const {
		if (!defined)
			throw std::runtime_error("CalibrationConfig not defined");
		return marker_properties;
	}

	calibration::SyncProperties CalibrationConfig::get_sync_properties() const {
		if (!defined)
			throw std::runtime_error("CalibrationConfig not defined");
		return sync_properties;
	}

	SequenceType CalibrationConfig::get_sequence_type() const {
		if (!defined)
			throw std::runtime_error("CalibrationConfig not defined");
		if (sequence_type == "video")
			return SequenceType::SEQUENCE_VIDEO;
		else
			return SequenceType::SEQUENCE_IMAGES;
		throw std::runtime_error("sequence_type argument is invalid");
	}

	size_t CalibrationConfig::get_total_sequence_files() const {
		if (!defined)
			throw std::runtime_error("CalibrationConfig not defined");
		return sequence_files.size();
	}

	std::string CalibrationConfig::get_sequence_file_at(unsigned int _index) const {
		if (!defined)
			throw std::runtime_error("CalibrationConfig not defined");
		if (!(_index < sequence_files.size()))
			throw std::runtime_error("index out of range");
		return sequence_files[_index];
	}

	bool CalibrationConfig::do_use_threading() const {
		if (!defined)
			throw std::runtime_error("CalibrationConfig not defined");
		return use_threading;
	}

	bool CalibrationConfig::do_use_cuda() const {
		if (!defined)
			throw std::runtime_error("CalibrationConfig not defined");
		return use_cuda;
	}

	calibration::PatternDimensions CalibrationConfig::get_pattern_dimensions() const {
		if (!defined)
			throw std::runtime_error("CalibrationConfig not defined");
		return pattern_dimensions;
	}

	bool CalibrationConfig::do_calibrate_camera() const {
		if (!defined)
			throw std::runtime_error("CalibrationConfig not defined");
		return calibrate_camera;
	}

	int CalibrationConfig::get_calibration_frame_count() const {
		if (!defined)
			throw std::runtime_error("CalibrationConfig not defined");
		return calibration_frames;
	}

	calibration::PredefinedRange CalibrationConfig::get_calibration_range() const {
		if (!defined)
			throw std::runtime_error("CalibrationConfig not defined");
		return calibration_range;
	}

	calibration::PredefinedRange CalibrationConfig::get_analysis_range() const {
		if (!defined)
			throw std::runtime_error("CalibrationConfig not defined");
		return analysis_range;
	}

	float CalibrationConfig::get_downsample_scale() const {
		if (!defined)
			throw std::runtime_error("CalibrationConfig not defined");
		return downsample_scale;
	}

	int CalibrationConfig::get_downsample_width() const {
		if (!defined)
			throw std::runtime_error("CalibrationConfig not defined");
		return downsample_width;
	}

	int CalibrationConfig::get_downsample_height() const {
		if (!defined)
			throw std::runtime_error("CalibrationConfig not defined");
		return downsample_height;
	}

	bool CalibrationConfig::do_clear_cache() const {
		if (!defined)
			throw std::runtime_error("CalibrationConfig not defined");
		return clear_point_cache;
	}

	bool CalibrationConfig::do_calculate_pose_means() const {
		if (!defined)
			throw std::runtime_error("CalibrationConfig not defined");
		return use_pose_means;
	}

	bool CalibrationConfig::do_downsampling() const {
		if (!defined)
			throw std::runtime_error("CalibrationConfig not defined");
		return downsample_scale > 0 && downsample_width > 0 && downsample_height > 0;
	}

	void CalibrationConfig::to_json(json& j) const {
		if (!defined)
			throw std::runtime_error("CalibrationConfig not defined");
		j = json{
			{"sequence_type", sequence_type},
			{"sequence_files", sequence_files},
			{"use_threading", use_threading},
			{"use_cuda", use_cuda},
			{"calibrate_camera", calibrate_camera},
			{"calibration_frames", calibration_frames},
			{"clear_point_cache", clear_point_cache},
			{"use_pose_means", use_pose_means}
		};
		if (downsample_scale > 0)
			j["downsample_scale"] = downsample_scale;
		if (downsample_width > 0)
			j["downsample_width"] = downsample_width;
		if (downsample_height > 0)
			j["downsample_height"] = downsample_height;
		if (charuco_properties.is_defined())
			j["charuco_properties"] = charuco_properties;
		if (marker_properties.is_defined())
			j["marker_properties"] = marker_properties;
		if (sync_properties.is_defined())
			j["sync_properties"] = sync_properties;
		if (pattern_dimensions.is_defined())
			j["pattern_dimensions"] = pattern_dimensions;
		if (calibration_range.is_defined())
			j["calibration_range"] = calibration_range;
		if (analysis_range.is_defined())
			j["analysis_range"] = analysis_range;
	}

	void CalibrationConfig::from_json(const json& j) {
		if (j.contains("sequence_type") && j.contains("sequence_files")) {
			j.at("sequence_type").get_to(sequence_type);
			j.at("sequence_files").get_to(sequence_files);
			defined = true;
		}
		else
			defined = false;
		if (j.contains("charuco_properties"))
			j.at("charuco_properties").get_to(charuco_properties);
		if (j.contains("marker_properties"))
			j.at("marker_properties").get_to(marker_properties);
		if (j.contains("sync_properties"))
			j.at("sync_properties").get_to(sync_properties);
		if (j.contains("use_threading"))
			j.at("use_threading").get_to(use_threading);
		if (j.contains("use_cuda"))
			j.at("use_cuda").get_to(use_cuda);
		if (j.contains("pattern_dimensions"))
			j.at("pattern_dimensions").get_to(pattern_dimensions);
		if (j.contains("calibrate_camera"))
			j.at("calibrate_camera").get_to(calibrate_camera);
		if (j.contains("calibration_frames"))
			j.at("calibration_frames").get_to(calibration_frames);
		if (j.contains("analysis_range"))
			j.at("analysis_range").get_to(analysis_range);
		if (j.contains("calibration_range"))
			j.at("calibration_range").get_to(calibration_range);
		if (j.contains("downsample_scale"))
			j.at("downsample_scale").get_to(downsample_scale);
		if (j.contains("downsample_width"))
			j.at("downsample_width").get_to(downsample_width);
		if (j.contains("downsample_height"))
			j.at("downsample_height").get_to(downsample_height);
		if (j.contains("clear_point_cache"))
			j.at("clear_point_cache").get_to(clear_point_cache);
		if (j.contains("use_pose_means"))
			j.at("use_pose_means").get_to(use_pose_means);
	}

	void to_json(json& j, const CalibrationConfig& p) {
		p.to_json(j);
	}

	void from_json(const json& j, CalibrationConfig& p) {
		p.from_json(j);
	}

	namespace tracking {

		cv::Point2f AnalysisData::get_charuco_corner(int id) const {
			if (!defined)
				throw std::runtime_error("AnalysisData not defined");
			if (!has_charuco_corners())
				throw std::runtime_error("AnalysisData contains no charuco corners");
			if (!has_charuco_corner(id))
				throw std::runtime_error("AnalysisData contains no charuco corners with id " + std::to_string(id));
			return charuco_corners.at(id);
		}

		std::vector<int> AnalysisData::get_charuco_corner_ids() const {
			if (!defined)
				throw std::runtime_error("AnalysisData not defined");
			std::vector<int> corner_ids;
			for (auto it = charuco_corners.begin(); it != charuco_corners.end(); it++) {
				corner_ids.push_back(it->first);
			}
			return corner_ids;
		}

		bool AnalysisData::has_charuco_corners() const {
			if (!defined)
				throw std::runtime_error("AnalysisData not defined");
			return !charuco_corners.empty();
		}

		bool AnalysisData::has_charuco_corner(int id) const {
			if (!defined)
				throw std::runtime_error("AnalysisData not defined");
			return charuco_corners.count(id) > 0;
		}

		cv::Point2f AnalysisData::get_marker_corner(int dict_id, int marker_id, int corner_id) const {
			if (!defined)
				throw std::runtime_error("AnalysisData not defined");
			if (!has_markers())
				throw std::runtime_error("AnalysisData has no markers");
			if (!has_markers(dict_id))
				throw std::runtime_error("AnalysisData does not contain markers from dictionary " + utility::dictEnumToString(dict_id));
			if (!has_marker(dict_id, marker_id))
				throw std::runtime_error("AnalysisData does not contain marker " + std::to_string(marker_id) + " from dictionary " + utility::dictEnumToString(dict_id));
			if (!(0 <= corner_id && corner_id < 4))
				throw std::runtime_error("Specified corder id is out of bounds");
			return marker_corners.at(dict_id).at(marker_id).at(corner_id);
		}

		std::vector<cv::Point2f> AnalysisData::get_marker_corners(int dict_id, int marker_id) const {
			if (!defined)
				throw std::runtime_error("AnalysisData not defined");
			if (!has_markers())
				throw std::runtime_error("AnalysisData has no markers");
			if (!has_markers(dict_id))
				throw std::runtime_error("AnalysisData does not contain markers from dictionary " + utility::dictEnumToString(dict_id));
			if (!has_marker(dict_id, marker_id))
				throw std::runtime_error("AnalysisData does not contain marker " + std::to_string(marker_id) + " from dictionary " + utility::dictEnumToString(dict_id));
			return marker_corners.at(dict_id).at(marker_id);
		}

		std::vector<int> AnalysisData::get_marker_ids(int dict_id) const {
			if (!defined)
				throw std::runtime_error("AnalysisData not defined");
			if (!has_markers())
				throw std::runtime_error("AnalysisData has no markers");
			if (!has_markers(dict_id))
				throw std::runtime_error("AnalysisData does not contain markers from dictionary " + utility::dictEnumToString(dict_id));
			std::vector<int> marker_ids;
			for (auto it = marker_corners.at(dict_id).begin(); it != marker_corners.at(dict_id).end(); it++) {
				marker_ids.push_back(it->first);
			}
			return marker_ids;
		}

		std::vector<int> AnalysisData::get_dictionary_ids() const {
			if (!defined)
				throw std::runtime_error("AnalysisData not defined");
			std::vector<int> dict_ids;
			for (auto it = marker_corners.begin(); it != marker_corners.end(); it++) {
				dict_ids.push_back(it->first);
			}
			return dict_ids;
		}

		bool AnalysisData::has_markers(int dict_id) const {
			return marker_corners.count(dict_id) > 0;
		}

		bool AnalysisData::has_markers() const {
			return !marker_corners.empty();
		}

		bool AnalysisData::has_marker(int dict_id, int marker_id) const {
			if (has_markers(dict_id))
				return marker_corners.at(dict_id).count(marker_id) > 0;
			return false;
		}

		void AnalysisData::to_json(json& j) const {
			if (!defined)
				throw std::runtime_error("AnalysisData not defined");
			j = json();
			for (auto it = marker_corners.begin(); it != marker_corners.end(); it++) {
				for (auto itt = it->second.begin(); itt != it->second.end(); itt++) {
					for (int i = 0; i < itt->second.size(); i++) {
						j["marker_corners"][utility::dictEnumToString(it->first)][std::to_string(itt->first)][i][0] = itt->second.at(i).x;
						j["marker_corners"][utility::dictEnumToString(it->first)][std::to_string(itt->first)][i][1] = itt->second.at(i).y;
					}
				}
			}
			for (auto it = charuco_corners.begin(); it != charuco_corners.end(); it++) {
				j["charuco_corners"][std::to_string(it->first)][0] = it->second.x;
				j["charuco_corners"][std::to_string(it->first)][1] = it->second.y;
			}
		}

		void AnalysisData::from_json(const json& j) {
			if (j.contains("marker_corners") || j.contains("charuco_corners")) {
				if (j.contains("marker_corners")) {
					for (auto it = j.at("marker_corners").begin(); it != j.at("marker_corners").end(); it++) {
						std::map<int, std::vector<cv::Point2f>> markers;
						for (auto itt = it->begin(); itt != it->end(); itt++) {
							std::vector<cv::Point2f> marker;
							for (int i = 0; i < 4; i++) {
								marker.push_back(cv::Point2f(itt->at(i).at(0), itt->at(i).at(1)));
							}
							markers.emplace(std::stoi(itt.key()), marker);
						}
						marker_corners.emplace(utility::dictStringToEnum(it.key()), markers);
					}
				}
				if (j.contains("charuco_corners")) {
					for (auto it = j.at("charuco_corners").begin(); it != j.at("charuco_corners").end(); it++) {
						charuco_corners.emplace(std::stoi(it.key()), cv::Point2f(it->at(0), it->at(1)));
					}
				}
				defined = true;
			}
		}

		void to_json(json& j, const AnalysisData& p) {
			p.to_json(j);
		}

		void from_json(const json& j, AnalysisData& p) {
			p.from_json(j);
		}

		cv::Mat PoseData::get_charuco_rvec() const {
			if (!defined)
				throw std::runtime_error("PoseData not defined");
			return board_tvec;
		}

		cv::Mat PoseData::get_charuco_tvec() const {
			if (!defined)
				throw std::runtime_error("PoseData not defined");
			return board_rvec;
		}

		void PoseData::to_json(json& j) const {
			if (!defined)
				throw std::runtime_error("PoseData not defined");
			j = json();
			for (int i = 0; i < board_tvec.rows; i++) {
				j["charuco_board"]["tvec"][i] = board_tvec.ptr<double>(i)[0];
			}
			for (int i = 0; i < board_rvec.rows; i++) {
				j["charuco_board"]["rvec"][i] = board_rvec.ptr<double>(i)[0];
			}
		}

		void PoseData::from_json(const json& j) {
			if (j.contains("charuco_board")) {
				cv::Mat tvec(3, 1, CV_64FC1);
				cv::Mat rvec(3, 1, CV_64FC1);
				for (int i = 0; i < 3; i++) {
					tvec.ptr<double>(i)[0] = j.at("charuco_board").at("tvec").at(i);
					rvec.ptr<double>(i)[0] = j.at("charuco_board").at("rvec").at(i);
				}
				board_tvec = tvec;
				board_rvec = rvec;
				defined = true;
			}
		}

		void to_json(json& j, const PoseData& p) {
			p.to_json(j);
		}

		void from_json(const json& j, PoseData& p) {
			p.from_json(j);
		}

		AnalysisData FrameData::get_analysis_data() const {
			if (!defined)
				throw std::runtime_error("FrameData not defined");
			return analysis_data;
		}

		PoseData FrameData::get_pose_data() const {
			if (!defined)
				throw std::runtime_error("FrameData not defined");
			return pose_data;
		}

		void FrameData::to_json(json& j) const {
			if (!defined)
				throw std::runtime_error("FrameData not defined");
			j = json{
				{"analysis_data", analysis_data},
				{"pose_data", pose_data}
			};
		}

		void FrameData::from_json(const json& j) {
			if (j.contains("analysis_data") || j.contains("pose_data")) {
				if(j.contains("analysis_data"))
					j.at("analysis_data").get_to(analysis_data);
				if (j.contains("pose_data"))
					j.at("pose_data").get_to(pose_data);
				defined = true;
			}
		}

		void to_json(json& j, const FrameData& p) {
			p.to_json(j);
		}

		void from_json(const json& j, FrameData& p) {
			p.from_json(j);
		}

		FrameData TrackingData::get_frame_data(int frame_id) const {
			if (!defined)
				throw std::runtime_error("TrackingData not defined");
			if (!has_frames())
				throw std::runtime_error("TrackingData contains no frame data");
			if (!has_frame(frame_id))
				throw std::runtime_error("TrackingData has no frame data for frame " + std::to_string(frame_id));
			return frames.at(frame_id);
		}

		std::vector<int> TrackingData::get_frame_ids() const {
			if (!defined)
				throw std::runtime_error("TrackingData not defined");
			std::vector<int> frame_ids;
			for (auto it = frames.begin(); it != frames.end(); it++) {
				frame_ids.push_back(it->first);
			}
			return frame_ids;
		}

		bool TrackingData::has_frames() const {
			return !frames.empty();
		}

		bool TrackingData::has_frame(int frame_id) const {
			return frames.count(frame_id) > 0;
		}

		calibration::CalibrationData TrackingData::get_calibration_data() const {
			if (!defined)
				throw std::runtime_error("TrackingData not defined");
			return calibration_data;
		}

		std::string TrackingData::get_file_name() const {
			if (!defined)
				throw std::runtime_error("TrackingData not defined");
			return file_name;
		}

		std::string TrackingData::get_runtime_ver() const {
			if (!defined)
				throw std::runtime_error("TrackingData not defined");
			return runtime_ver;
		}

		float TrackingData::get_frames_per_second() const {
			if (!defined)
				throw std::runtime_error("TrackingData not defined");
			return frames_per_second;
		}

		CalibrationConfig TrackingData::get_previous_config() const {
			if (!defined)
				throw std::runtime_error("TrackingData not defined");
			return previous_config;
		}

		void TrackingData::to_json(json& j) const {
			if (!defined)
				throw std::runtime_error("TrackingData not defined");
			j = json{
				{"file_name", file_name},
				{"runtime_ver", runtime_ver},
				{"frames_per_second", frames_per_second},
				{"previous_config", previous_config}
			};
			for (auto it = frames.begin(); it != frames.end(); it++) {
				j["frames"][std::to_string(it->first)] = it->second;
			}
			if (calibration_data.is_defined())
				j["calibration_data"] = calibration_data;
		}

		void TrackingData::from_json(const json& j) {
			if (j.contains("file_name") && j.contains("runtime_ver") && j.contains("frames_per_second") && j.contains("previous_config")) {
				j.at("previous_config").get_to(previous_config);
				if (j.contains("calibration_data"))
					j.at("calibration_data").get_to(calibration_data);
				if (j.contains("frames")) {
					for (auto it = j.at("frames").begin(); it != j.at("frames").end(); it++) {
						FrameData f_data;
						it->get_to(f_data);
						frames.emplace(std::stoi(it.key()), f_data);
					}
				}
				j.at("file_name").get_to(file_name);
				j.at("runtime_ver").get_to(runtime_ver);
				j.at("frames_per_second").get_to(frames_per_second);
				defined = true;
			}
		}

		void to_json(json& j, const TrackingData& p) {
			p.to_json(j);
		}

		void from_json(const json& j, TrackingData& p) {
			p.from_json(j);
		}

	}

	std::vector<std::string> CacheData::get_tracking_group_names() const {
		if (!defined)
			throw std::runtime_error("CacheData not defined");
		std::vector<std::string> groups;
		for (auto it = tracking_data.begin(); it != tracking_data.end(); it++) {
			groups.push_back(it->first);
		}
		return groups;
	}

	tracking::TrackingData CacheData::get_tracking_data(std::string group_id) const {
		if (!defined)
			throw std::runtime_error("CacheData not defined");
		if (!has_tracking_data())
			throw std::runtime_error("CacheData contains no tracking data");
		if (!has_tracking_group_name(group_id))
			throw std::runtime_error("CacheData contains no tracking group named " + group_id);
		return tracking_data.at(group_id);
	}

	bool CacheData::has_tracking_group_name(std::string group_id) const {
		return tracking_data.count(group_id) > 0;
	}

	bool CacheData::has_tracking_data() const {
		return !tracking_data.empty();
	}

	json CacheData::get_errors() const {
		if (!defined)
			throw std::runtime_error("CacheData not defined");
		return errors;
	}

	void CacheData::to_json(json& j) const {
		if (!defined)
			throw std::runtime_error("CacheData not defined");
		j = json{
			{"success", success},
			{"errors", errors}
		};
		for (auto it = tracking_data.begin(); it != tracking_data.end(); it++) {
			j[it->first] = it->second;
		}
	}

	void CacheData::from_json(const json& j) {
		if (j.contains("success") && j.contains("errors")) {
			me::utility::memstream key(keyfile_bin, keyfile_bin_len);
			std::string ver = me::crypto::StreamToHMAC_SHA1(key, key).to_string();
			j.at("success").get_to(success);
			errors = j.at("errors");
			for (auto it = j.begin(); it != j.end(); it++) {
				if (it.key().length() == 40 && ver == it->at("runtime_ver")) {
					tracking::TrackingData t_data;
					it->get_to(t_data);
					tracking_data.emplace(it.key(), t_data);
				}
			}
			defined = true;
		}
	}

	void to_json(json& j, const CacheData& p) {
		p.to_json(j);
	}

	void from_json(const json& j, CacheData& p) {
		p.from_json(j);
	}

}