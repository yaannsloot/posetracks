/*
me_dnn_module_opencv.cpp
DLL source code for OpenCV deep neural network framework module

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
#include <me_utils.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/core/ocl.hpp>

using namespace me::utility;

namespace me {

	namespace dnn {

		enum NetType {
			YOLO,
			GENERIC
		};

		bool stringToBool(const std::string& s) {
			if (s == "true" || s == "1")
				return true;
			else if (s == "false" || s == "0")
				return false;
			else
				throw std::invalid_argument("Invalid boolean string");
		}

		class MEDNNModuleOpenCV : public MEDNNModule {
		public:
			std::unordered_map<std::string, std::string> GetModuleDetails() override;
			std::vector<std::vector<cv::Mat>> Forward(std::vector<std::vector<cv::Mat>>& inputs) override;
			bool LoadModel(std::unordered_map<std::string, std::string>& args) override;
			bool SetForwardRules(std::unordered_map<std::string, std::string>& args) override;
			void UnloadModel() override;
			~MEDNNModuleOpenCV() override;
		private:
			cv::dnn::Net net;
			double tensor_scale = 1 / 255.0; // Default scale for INT8 which is expected for OpenCV images
			float conf_thresh = 0.5;
			float nms_thresh = 0.5;
			float do_nms = true;
			NetType net_type = YOLO;
		};

		std::unordered_map<std::string, std::string> MEDNNModuleOpenCV::GetModuleDetails() {
			std::unordered_map<std::string, std::string> result;
			result["framework"] = "OpenCV ";
			result["framework_version"] = CV_VERSION;
			result["module_version"] = "1.0";
			return result;
		}

		// THIS WORKS PRETTY MUCH EXCLUSIVELY WITH YOLO. MAY WANT TO ADJUST FOR MORE COMPATABILITY IN THE FUTURE
		std::vector<std::vector<cv::Mat>> MEDNNModuleOpenCV::Forward(std::vector<std::vector<cv::Mat>>& inputs) {
			std::vector<std::vector<cv::Mat>> outputs;
			try {
				// Batch size check
				const int batch_size = inputs.size();
				if (batch_size < 1)
					throw std::invalid_argument("Received an empty input vector");
				for (auto &batch : inputs) {
					if (!batch.empty()) {
						if (net.empty())
							throw std::runtime_error("MODEL IS NOT LOADED!");

						cv::Size img_size(batch[0].cols, batch[0].rows);
						cv::Mat blob = cv::dnn::blobFromImages(batch, tensor_scale, img_size, cv::Scalar(), true, false);
						auto out_names = net.getUnconnectedOutLayersNames();
						net.setInput(blob);
						std::vector<cv::Mat> batch_out;
						net.forward(batch_out, out_names); // [ H, W, X, X ] with batch = 1 and [ N, H, W, C ] with batch > 1. X means unused and unstable
						const auto num_layers = out_names.size();

						//Standard output handling
						if (net_type == GENERIC) {
							// Blob separation and layer output concatenation
							std::vector<cv::Mat>* processed_out = new std::vector<cv::Mat>[num_layers];
							for (int l = 0; l < num_layers; l++) {

								if (batch.size() == 1) {
									processed_out[l].push_back(batch_out[l]);
								}
								else {
									for (int i = 0; i < batch_out[l].size[0]; ++i) {  // blob.size[0] gives the batch size.
										cv::Mat mat(batch_out[l].size[1], batch_out[l].size[2], CV_32F, batch_out[l].ptr<float>(i));  // Create a new cv::Mat for each image in the batch.
										processed_out[l].push_back(mat);
									}
								}
							}
							// Concatenation
							std::vector<cv::Mat> concatenated_out;
							for (int i = 0; i < batch.size(); ++i) {
								cv::Mat concat_mat;
								std::vector<cv::Mat> mats_to_concat;
								for (int l = 0; l < num_layers; l++) {
									mats_to_concat.push_back(processed_out[l][i]);
								}
								cv::vconcat(mats_to_concat, concat_mat);
								concatenated_out.push_back(concat_mat);
							}
							delete[] processed_out;
							outputs.push_back(concatenated_out);
						}
						// YOLO output handling
						else if (net_type == YOLO) {

							std::vector<std::vector<Detection>> detections;
							std::vector<cv::Mat> out_mats;
							for (cv::Mat& layer_blob : batch_out) {
								auto batch_mats = SplitBatchesYOLO(layer_blob);
								auto layer_detections = PostprocessYOLO(batch_mats, conf_thresh, img_size);
								const auto batch_size = layer_detections.size();
								if (detections.empty())
									detections = layer_detections;
								else {
									for (int b = 0; b < batch_size; b++) {
										detections[b].insert(detections[b].end(), layer_detections[b].begin(), layer_detections[b].end());
									}
								}
							}
							const auto batch_size = detections.size();
							for (int b = 0; b < batch_size; b++) {
								std::vector<Detection>& batch_detections = detections[b];

								if (do_nms) {
									std::vector<Detection> filtered_detections;
									std::unordered_map<int, std::vector<Detection>> class_map;
									for (Detection& det : batch_detections) {
										if (class_map.count(det.class_id) == 0)
											class_map[det.class_id] = std::vector<Detection>();
										class_map[det.class_id].push_back(det);
									}
									for (auto& class_pair : class_map) {
										std::vector<int> indices;
										std::vector<cv::Rect2d> boxes;
										std::vector<float> scores;
										for (auto& box : class_pair.second) {
											boxes.push_back(box.bbox);
											scores.push_back(box.score);
										}
										cv::dnn::NMSBoxes(boxes, scores, 0.0, nms_thresh, indices);
										for (int i : indices) {
											filtered_detections.push_back(class_pair.second[i]);
										}
									}
									out_mats.push_back(PackDetections(filtered_detections));
								}
								else {
									out_mats.push_back(PackDetections(batch_detections));
								}
							}

							outputs.push_back(out_mats);
						}

					}
				}
			}
			catch (std::exception& exa) {
				std::string what = exa.what();
				throw std::runtime_error("Encountered exception when forwarding through model: \"" + what + "\"");
			}
			return outputs;
		}

		bool MEDNNModuleOpenCV::LoadModel(std::unordered_map<std::string, std::string>& args) {
			bool result = false;
			try {
				if (args.count("format")) {
					if (args["format"] == "darknet") {
						std::string cfg_file = args["cfg_file"];
						std::string weights_file = args["weights_file"];
						net = cv::dnn::readNetFromDarknet(cfg_file, weights_file);
						result = true;
					}
					else if (args["format"] == "onnx") {
						std::string onnx_file = args["model_file"];
						net = cv::dnn::readNet(onnx_file);
						result = true;
					}
					else if (args["format"] == "pytorch") {
						std::string model_file = args["model_file"];
						bool is_binary = args.count("is_binary") && stringToBool(args["is_binary"]);
						net = cv::dnn::readNetFromTorch(model_file, is_binary);
						result = true;
					}
					else if (args["format"] == "tensorflow") {
						std::string model_file = args["model_file"];
						std::string config_file = args["config_file"];
						net = cv::dnn::readNetFromTensorflow(model_file, config_file);
						result = true;
					}
					else if (args["format"] == "caffe") {
						std::string prototxt_file = args["prototxt_file"];
						std::string caffe_model_file = args["caffe_model_file"];
						net = cv::dnn::readNetFromCaffe(prototxt_file, caffe_model_file);
						result = true;
					}
					else
						throw std::invalid_argument("Unsupported format \"" + args["format"] + "\"");
					return result;
				}
				else
					throw std::invalid_argument("Format not defined in load args");
			}
			catch (std::exception& exa) {
				std::string what = exa.what();
				throw std::runtime_error("Encountered exception when loading model: \"" + what + "\"");
			}
			return result;
		}

		bool MEDNNModuleOpenCV::SetForwardRules(std::unordered_map<std::string, std::string>& args) {
			bool result = false;
			try {
				if (args.count("device")) {
					const bool has_opencl = cv::ocl::haveOpenCL();
					const bool has_cuda = cv::cuda::getCudaEnabledDeviceCount() > 0;
					net.setPreferableBackend(cv::dnn::DNN_BACKEND_DEFAULT);
					net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
					if (args["device"] == "gpu") {
						if (has_cuda) {
							net.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
							net.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);
						}
						else if(has_opencl) {
							net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
							net.setPreferableTarget(cv::dnn::DNN_TARGET_OPENCL);
						}
					}
					else if (args["device"] == "gpu16") {
						if (has_cuda) {
							net.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
							net.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA_FP16);
						}
						else if (has_opencl) {
							net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
							net.setPreferableTarget(cv::dnn::DNN_TARGET_OPENCL_FP16);
						}
					}
				}
				if (args.count("conf_thresh"))
					conf_thresh = std::stof(args["conf_thresh"]);
				if (args.count("nms_thresh"))
					conf_thresh = std::stof(args["nms_thresh"]);
				if (args.count("do_nms"))
					conf_thresh = stringToBool(args["do_nms"]);
				result = true;
			}
			catch (std::exception& exa) {
				std::string what = exa.what();
				throw std::runtime_error("Encountered exception when setting forward rules: \"" + what + "\"");
			}
			return result;
		}

		void MEDNNModuleOpenCV::UnloadModel() {
			net = cv::dnn::Net();
		}

		MEDNNModuleOpenCV::~MEDNNModuleOpenCV() {
			UnloadModel();
		}

	}

}

me::dnn::MEDNNModule* createModuleInstance() {
	return new me::dnn::MEDNNModuleOpenCV();
}

void destroyModuleInstance(me::dnn::MEDNNModule* instance) {
	delete instance;
}