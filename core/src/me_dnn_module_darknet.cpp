/*
me_dnn_module_darknet.cpp
DLL source code for Darknet deep neural network framework module

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
#include <darknet.h>
#include <string>
#include <map>
#include <utility>
#include <vector>
using namespace cv;

namespace me {

	namespace dnn {

		image mat_to_image(cv::Mat mat) {
			int w = mat.cols;
			int h = mat.rows;
			int c = mat.channels();
			image img = make_image(w, h, c);

			unsigned char* data = (unsigned char*)mat.data;

			for (int i = 0; i < h; ++i) {
				for (int j = 0; j < w; ++j) {
					for (int k = 0; k < c; ++k) {
						int src_index = i * w * c + j * c + k;
						int dst_index = k * w * h + i * w + j;
						img.data[dst_index] = (float)data[src_index] / 255.0f;
					}
				}
			}

			return img;
		}

		image copy_image(image src) {
			int width = src.w;
			int height = src.h;
			int channels = src.c;
			image dst = make_image(width, height, channels);

			// Copy data from src to dst
			memcpy(dst.data, src.data, width * height * channels * sizeof(float));

			return dst;
		}

		float max_class_prob(float* probs, int num_classes, int* index) {
			float max_prob = -1.0;
			*index = -1;
			for (int i = 0; i < num_classes; ++i) {
				if (probs[i] > max_prob) {
					max_prob = probs[i];
					*index = i;
				}
			}
			return max_prob;
		}

		bool stringToBool(const std::string& s) {
			if (s == "true" || s == "1")
				return true;
			else if (s == "false" || s == "0")
				return false;
			else
				throw std::invalid_argument("Invalid boolean string");
		}

		class MEDNNModuleDarknet : public MEDNNModule {
		public:
			std::unordered_map<std::string, std::string> GetModuleDetails() override;
			std::vector<std::vector<cv::Mat>> Forward(std::vector<std::vector<cv::Mat>>& inputs) override;
			bool LoadModel(std::unordered_map<std::string, std::string>& args) override;
			bool SetForwardRules(std::unordered_map<std::string, std::string>& args) override;
			void UnloadModel() override;
			~MEDNNModuleDarknet() override;
		private:
			network* net = nullptr;
			float thresh = 0.5f;
			float hier_thresh = 0.5;
			float nms = 0.4f;
			int batch_size = 1;
			bool do_letterboxing = false;
		};

		std::unordered_map<std::string, std::string> MEDNNModuleDarknet::GetModuleDetails() {
			std::unordered_map<std::string, std::string> result;
			result["framework"] = "Darknet";
			result["module_version"] = "1.0";
			return result;
		}

		std::vector<std::vector<cv::Mat>> MEDNNModuleDarknet::Forward(std::vector<std::vector<cv::Mat>>& inputs) {
			std::vector<std::vector<cv::Mat>> outputs;
			try {
				// Batch size check
				const int batch_size = inputs.size();
				if (batch_size < 1)
					throw std::invalid_argument("Received an empty input vector");
				// Arg count check
				const int arg_count = inputs[0].size();
				for (auto it = inputs.begin(); it != inputs.end(); ++it) {
					if ((*it).size() != arg_count) {
						std::stringstream ss;
						ss << "Arg count is inconsistent between batches. Expected " << arg_count << " but got " << (*it).size();
						throw std::invalid_argument(ss.str());
					}
				}
				//TODO: If darknet actually allows batch inferencing, replace this loop
				for (auto batch = inputs.begin(); batch != inputs.end(); ++batch) {
					std::vector<cv::Mat> batch_out;
					for (auto cv_image = (*batch).begin(); cv_image != (*batch).end(); ++cv_image) {
						if (net == nullptr)
							throw std::runtime_error("MODEL IS NOT LOADED!");
						layer l = net->layers[net->n - 1];
						image im = mat_to_image(*cv_image);
						image sized;
						if (do_letterboxing)
							sized = letterbox_image(im, net->w, net->h);
						else
							sized = copy_image(im);
						network_predict(*net, sized.data);
						int nboxes = 0;
						detection* dets = get_network_boxes(net, im.w, im.h, thresh, hier_thresh, 0, 1, &nboxes, 0);

						// TODO: Add support for other nms functions do_nms_sort and cv::dnn::NMSBoxes
						if (nms) do_nms_obj(dets, nboxes, l.classes, nms);
						std::unordered_map<int, std::vector<std::pair<cv::Rect2f, float>>> detections;
						for (int i = 0; i < nboxes; ++i) {
							int class_id;
							float prob = max_class_prob(dets[i].prob, l.classes, &class_id);
							if (prob > thresh) {
								box b = dets[i].bbox;
								float left = (b.x - b.w / 2.f) * (float)im.w;
								float right = (b.x + b.w / 2.f) * (float)im.w;
								float top = (b.y - b.h / 2.f) * (float)im.h;
								float bottom = (b.y + b.h / 2.f) * (float)im.h;
								std::pair<cv::Rect2f, float> box_out(cv::Rect2f(Point2f(left, top), Point2f(right, bottom)), prob);
								if (detections.count(class_id) == 0) {
									std::vector<std::pair<cv::Rect2f, float>> v;
									detections[class_id] = v;
								}
								detections[class_id].push_back(box_out);
							}
						}
						free_image(im);
						free_image(sized);
						free_detections(dets, nboxes);

						int mat_size = 0;
						for (auto det_class : detections) {
							mat_size += det_class.second.size();
						}
						// initialize matrix with size of detections
						cv::Mat output(mat_size, 6, CV_32F);
						int e_row = 0;
						for (auto det_class : detections) {
							for (auto det_box : det_class.second) {
								output.at<float>(e_row, 0) = det_class.first;
								output.at<float>(e_row, 1) = det_box.first.tl().y;
								output.at<float>(e_row, 2) = det_box.first.tl().x;
								output.at<float>(e_row, 3) = det_box.first.br().y;
								output.at<float>(e_row, 4) = det_box.first.br().x;
								output.at<float>(e_row, 5) = det_box.second;
								e_row += 1;
							}
						}
						batch_out.push_back(output);
					}
					outputs.push_back(batch_out);
				}
			}
			catch (std::exception& exa) {
				std::string what = exa.what();
				throw std::runtime_error("Encountered exception when forwarding through model: \"" + what + "\"");
			}
			return outputs;
		}

		bool MEDNNModuleDarknet::LoadModel(std::unordered_map<std::string, std::string>& args) {
			bool result = false;
			try {
				std::string cfg_file = args["cfg_file"];
				std::string weights_file = args["weights_file"];
				net = load_network_custom((char*)cfg_file.c_str(), (char*)weights_file.c_str(), 0, batch_size);
				result = true;
			}
			catch (std::runtime_error& exa) {
				std::string what = exa.what();
				throw std::runtime_error("Encountered exception when loading model: \"" + what + "\"");
			}
			return result;
		}

		bool MEDNNModuleDarknet::SetForwardRules(std::unordered_map<std::string, std::string>& args) {
			bool result = false;
			try {
				if (args.count("do_letterboxing"))
					do_letterboxing = stringToBool(args["do_letterboxing"]);
				if(args.count("thresh"))
					thresh = std::stof(args["thresh"]);
				if (args.count("hier_thresh"))
					hier_thresh = std::stof(args["hier_thresh"]);
				if (args.count("nms"))
					nms = std::stof(args["nms"]);
				if (args.count("batch_size"))
					batch_size = std::stoi(args["batch_size"]);
				if (args.count("gpu_index")) {
					int _gpu_index = std::stoi(args["gpu_index"]);
					if (_gpu_index > -1)
						cuda_set_device(_gpu_index);
				}
				result = true;
			}
			catch (std::invalid_argument& exa) {
				std::string what = exa.what();
				throw std::runtime_error("Encountered exception when setting forward rules: \"" + what + "\"");
			}
			catch (std::out_of_range& exo) {
				std::string what = exo.what();
				throw std::runtime_error("Encountered exception when setting forward rules: \"" + what + "\"");
			}
			return result;
		}

		void MEDNNModuleDarknet::UnloadModel() {
			if (net != nullptr) {
				free_network(*net);
				net = nullptr;
			}
		}

		MEDNNModuleDarknet::~MEDNNModuleDarknet() {
			UnloadModel();
		}

	}

}

me::dnn::MEDNNModule* createModuleInstance() {
	return new me::dnn::MEDNNModuleDarknet();
}

void destroyModuleInstance(me::dnn::MEDNNModule* instance) {
	delete instance;
}