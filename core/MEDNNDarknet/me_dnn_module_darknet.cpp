#include <pch.h>
#include <string>
#include <map>
#include <utility>
#include <vector>
#include <opencv2/opencv.hpp>
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

		class MEDNNModuleDarknet : public MEDNNModule {
		public:
			std::any GetModuleDetails() override;
			std::any Forward(std::any& input) override;
			std::any LoadModel(std::any& args) override;
			std::any SetForwardRules(std::any& args) override;
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

		std::any MEDNNModuleDarknet::GetModuleDetails() {
			std::unordered_map<std::string, std::any> result;
			result["framework"] = "Darknet";
			result["module_version"] = "1.0";
			return result;
		}

		std::any MEDNNModuleDarknet::Forward(std::any& input) {
			std::unordered_map<std::string, std::any> result;
			result["success"] = false;
			result["error"] = "";
			try {
				cv::Mat img = std::any_cast<cv::Mat>(input);
				if (net == nullptr)
					throw std::runtime_error("MODEL IS NOT LOADED!");
				layer l = net->layers[net->n - 1];
				image im = mat_to_image(img);
				image sized;
				if (do_letterboxing)
					sized = letterbox_image(im, net->w, net->h);
				else
					sized = copy_image(im);
				network_predict(*net, sized.data);
				int nboxes = 0;
				detection* dets = get_network_boxes(net, im.w, im.h, thresh, hier_thresh, 0, 1, &nboxes, 0);
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
				result["detections"] = detections;
				result["success"] = true;
			}
			catch (std::bad_any_cast& exa) {
				std::string what = exa.what();
				result["error"] = what;
			}
			catch (std::runtime_error& exa) {
				std::string what = exa.what();
				result["error"] = what;
			}
			return result;
		}

		std::any MEDNNModuleDarknet::LoadModel(std::any& args) {
			std::unordered_map<std::string, std::any> result;
			result["success"] = false;
			result["error"] = "";
			try {
				std::unordered_map<std::string, std::any> a = std::any_cast<std::unordered_map<std::string, std::any>>(args);
				std::string cfg_file = std::any_cast<std::string>(a["cfg_file"]);
				std::string weights_file = std::any_cast<std::string>(a["weights_file"]);
				net = load_network_custom((char*)cfg_file.c_str(), (char*)weights_file.c_str(), 0, batch_size);
			}
			catch (std::bad_any_cast& exa) {
				std::string what = exa.what();
				result["error"] = what;
			}
			return result;
		}

		std::any MEDNNModuleDarknet::SetForwardRules(std::any& args) {
			std::unordered_map<std::string, std::any> result;
			result["success"] = false;
			result["error"] = "";
			try {
				std::unordered_map<std::string, std::any> a = std::any_cast<std::unordered_map<std::string, std::any>>(args);
				if (a.count("do_letterboxing"))
					do_letterboxing = std::any_cast<bool>(a["do_letterboxing"]);
				if(a.count("thresh"))
					thresh = std::any_cast<float>(a["thresh"]);
				if (a.count("hier_thresh"))
					hier_thresh = std::any_cast<float>(a["hier_thresh"]);
				if (a.count("nms"))
					nms = std::any_cast<float>(a["nms"]);
				if (a.count("batch_size"))
					batch_size = std::any_cast<int>(a["batch_size"]);
				if (a.count("gpu_index")) {
					gpu_index = std::any_cast<int>(a["gpu_index"]);
					if (gpu_index > -1)
						cuda_set_device(gpu_index);
				}
				result["success"] = true;
			}
			catch (std::bad_any_cast& exa) {
				std::string what = exa.what();
				result["error"] = what;
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