#include <me_dnn_module.hpp>
#include <pch.h>
#include <stdexcept>

namespace me {

	namespace dnn {

		MEDNNModuleInstance::MEDNNModuleInstance(std::string module_name) {
			int requiredSize = MultiByteToWideChar(CP_UTF8, 0, module_name.c_str(), -1, nullptr, 0);
			std::wstring wideStr(requiredSize, L'\0');
			MultiByteToWideChar(CP_UTF8, 0, module_name.c_str(), -1, &wideStr[0], requiredSize);
			dll = LoadLibrary(wideStr.c_str());
			if (dll == NULL)
				throw std::runtime_error("Could not find module \"" + module_name + "\"");
			createInstancePrototype dllCreateFunc = (createInstancePrototype)GetProcAddress(dll, "createModuleInstance");
			dllDestroyFunc = (destroyInstancePrototype)GetProcAddress(dll, "destroyModuleInstance");
			if(dllCreateFunc == NULL || dllDestroyFunc == NULL)
				throw std::runtime_error("Module \"" + module_name + "\" does not contain correct instancing functions");
			instance = dllCreateFunc();
		}

		std::any MEDNNModuleInstance::GetModuleDetails() {
			return instance->GetModuleDetails();
		}

		std::any MEDNNModuleInstance::Forward(std::any& input) {
			return instance->Forward(input);
		}

		std::any MEDNNModuleInstance::LoadModel(std::any& args) {
			return instance->LoadModel(args);
		}

		std::any MEDNNModuleInstance::SetForwardRules(std::any& args) {
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