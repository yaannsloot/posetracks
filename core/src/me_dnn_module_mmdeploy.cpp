/*
me_dnn_module_mmdeploy.cpp
DLL source code for MMDeploy deep neural network framework module

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

#include <me_dnn_module.hpp>
#include <mmdeploy/detector.hpp>
#include <mmdeploy/pose_detector.hpp>
#include <string>

namespace me {

	namespace dnn {

		bool stringToBool(const std::string& s) {
			if (s == "true" || s == "1")
				return true;
			else if (s == "false" || s == "0")
				return false;
			else
				throw std::runtime_error("Invalid boolean string");
		}

		enum TargetDevice {
			CPU = 0,
			GPU = 1
		};

		class MEDNNModuleMMDeploy : public MEDNNModule {
		public:
			std::unordered_map<std::string, std::string> GetModuleDetails() override;
			std::vector<std::vector<cv::Mat>> Forward(std::vector<std::vector<cv::Mat>>& inputs) override;
			bool LoadModel(std::unordered_map<std::string, std::string>& args) override;
			bool SetForwardRules(std::unordered_map<std::string, std::string>& args) override;
			void UnloadModel() override;
			~MEDNNModuleMMDeploy() override;
		private:
		    mmdeploy::cxx::Detector *detector = nullptr;
			mmdeploy::cxx::PoseDetector *pose_detector = nullptr;
			TargetDevice target_device = TargetDevice::GPU;
			int device_id = 0;
		};

		std::unordered_map<std::string, std::string> MEDNNModuleMMDeploy::GetModuleDetails() {
			std::unordered_map<std::string, std::string> details;
			details["name"] = "MMDeploy";
			details["version"] = "1.1.0";
			details["description"] = "MMDeploy is a module for deploying models trained by MMDetection. It is currently configured exclusively for deploying the multi-person pose detection pipeline";
			details["framework"] = "ONNX;TensorRT";
			details["module_version"] = "1.0";
			return details;
		}

		std::vector<std::vector<cv::Mat>> MEDNNModuleMMDeploy::Forward(std::vector<std::vector<cv::Mat>>& inputs) {
			std::vector<std::vector<cv::Mat>> outputs;
			if (pose_detector != nullptr) {
				for (auto &batch : inputs) {
					std::vector<cv::Mat> batch_outputs;
					for (auto& image : batch) {
						auto bbox_results = detector->Apply(image);
						std::vector<mmdeploy_rect_t> bboxes;
						for (const mmdeploy_detection_t& det : bbox_results) {
							if (det.label_id == 0 && det.score > 0.5) {
								bboxes.push_back(det.bbox);
							}
						}
						auto pose_results = pose_detector->Apply(image, bboxes);
						assert(bboxes.size() == poses.size());
						int total_points = 0;
						for (const auto& pose : pose_results) {
							total_points += pose.length;
						}
						cv::Mat mat_out(total_points, 5, CV_32F);
						int row = 0;
						for (int i = 0; i < pose_results.size(); i++) {
							auto &pose = pose_results[i];
							for (int kp = 0; kp < pose.length; kp++) {
								mat_out.ptr<float>(row)[0] = i;
								mat_out.ptr<float>(row)[1] = kp;
								mat_out.ptr<float>(row)[2] = pose.point[kp].x;
								mat_out.ptr<float>(row)[3] = pose.point[kp].y;
								mat_out.ptr<float>(row)[4] = pose.score[kp];
								row++;
							}
						}
						batch_outputs.push_back(mat_out);
					}
					outputs.push_back(batch_outputs);
				}
			}
			return outputs;
		}

		bool MEDNNModuleMMDeploy::LoadModel(std::unordered_map<std::string, std::string>& args) {
			bool result = false;
			try {
				if (args.count("detector_model_path") && args.count("pose_model_path")) {
					UnloadModel();
					mmdeploy::cxx::Model det_model(args["detector_model_path"]);
					mmdeploy::cxx::Model pose_model(args["pose_model_path"]);
					std::string target_device_str = "cuda";
					if(target_device == CPU)
						target_device_str = "cpu";
					mmdeploy::cxx::Device device(target_device_str, device_id);
					detector = new mmdeploy::cxx::Detector(det_model, device);
					pose_detector = new mmdeploy::cxx::PoseDetector(pose_model, device);
				}
			}
			catch (const std::exception& exa) {
				std::string what = exa.what();
				throw std::runtime_error("Encountered exception when loading model: \"" + what + "\"");
			}
			return result;
		}

		bool MEDNNModuleMMDeploy::SetForwardRules(std::unordered_map<std::string, std::string>& args) {
			bool result = false;
			try {
				if (args.count("device")) {
					if(args["device"] == "cpu")
						target_device = TargetDevice::CPU;
					else if(args["device"] == "gpu")
						target_device = TargetDevice::GPU;
					else
						throw std::runtime_error("Invalid target device");
				}
				if (args.count("device_id")) {
					device_id = std::stoi(args["device_id"]);
				}
			}
			catch (const std::exception& exa) {
				std::string what = exa.what();
				throw std::runtime_error("Encountered exception when setting forward rules: \"" + what + "\"");
			}
			return result;
		}

		void MEDNNModuleMMDeploy::UnloadModel() {
			if (detector != nullptr) {
				delete detector;
				detector = nullptr;
			}
			if (pose_detector != nullptr) {
				delete pose_detector;
				pose_detector = nullptr;
			}
		}

		MEDNNModuleMMDeploy::~MEDNNModuleMMDeploy() {
			UnloadModel();
		}

	}

}

me::dnn::MEDNNModule* createModuleInstance() {
	return new me::dnn::MEDNNModuleMMDeploy();
}

void destroyModuleInstance(me::dnn::MEDNNModule* instance) {
	delete instance;
}