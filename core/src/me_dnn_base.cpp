/*
me_dnn_base.cpp
Source code for the base module that loads individual framework DLLs

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
#include <stdexcept>

namespace me {

	namespace dnn {

		MEDNNModuleInstance::MEDNNModuleInstance(const std::string module_name) {
			dll = LoadLibraryA(module_name.c_str());
			if (dll == NULL)
				throw std::runtime_error("Could not find module \"" + module_name + "\"");
			createInstancePrototype dllCreateFunc = (createInstancePrototype)GetProcAddress(dll, "createModuleInstance");
			dllDestroyFunc = (destroyInstancePrototype)GetProcAddress(dll, "destroyModuleInstance");
			if(dllCreateFunc == NULL || dllDestroyFunc == NULL)
				throw std::runtime_error("Module \"" + module_name + "\" does not contain correct instancing functions");
			instance = dllCreateFunc();
		}

		std::unordered_map<std::string, std::string> MEDNNModuleInstance::GetModuleDetails() {
			return instance->GetModuleDetails();
		}

		std::vector<std::vector<cv::Mat>> MEDNNModuleInstance::Forward(std::vector<std::vector<cv::Mat>>& inputs) {
			return instance->Forward(inputs);
		}

		std::vector<cv::Mat> MEDNNModuleInstance::Forward(std::vector<cv::Mat>& inputs) {
			std::vector<std::vector<cv::Mat>> batch_input = { inputs };
			std::vector<std::vector<cv::Mat>> outputs = Forward(batch_input);
			return outputs[0];
		}

		cv::Mat MEDNNModuleInstance::Forward(cv::Mat& input) {
			std::vector<cv::Mat> input_collection = { input };
			std::vector<cv::Mat> output_collection = Forward(input_collection);
			return output_collection[0];
		}

		bool MEDNNModuleInstance::LoadModel(std::unordered_map<std::string, std::string>& args) {
			return instance->LoadModel(args);
		}

		bool MEDNNModuleInstance::SetForwardRules(std::unordered_map<std::string, std::string>& args) {
			return instance->SetForwardRules(args);
		}

		void MEDNNModuleInstance::UnloadModel() {
			return instance->UnloadModel();
		}

		MEDNNModuleInstance::~MEDNNModuleInstance() {
			dllDestroyFunc(instance);
			FreeLibrary(dll);
		}

	}

}
