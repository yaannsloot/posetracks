/*
me_dnn_module.cpp
This is the base header that defines functions to be implemented in each framework module

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

#ifdef ME_DNN_MODULE_PROVIDER

#define MECOREDNN __declspec(dllexport)

#else

#define MECOREDNN __declspec(dllimport)

#ifdef ME_DNN_BASE_EXPORTS

#define MEDNN __declspec(dllexport)

#else

#define MEDNN __declspec(dllimport)

#endif

#endif

#ifndef ME_DNN_MODULE_HPP
#define ME_DNN_MODULE_HPP

#include <opencv2/opencv.hpp>

#ifndef ME_DNN_MODULE_PROVIDER

#include <string>
#include <Windows.h>

#endif

namespace me {

	namespace dnn {

		/**
		 * @class MEDNNModule
		 * @brief Class prototype for deep neural network framework modules.
		 *
		 * @remarks
		 * Contains a set of functions that handle the loading, unloading, and processing of deep neural network models.
		 * This class is an abstract base class, and must be inherited by a derived class that implements the
		 * pure virtual functions.
		 */
		class MEDNNModule {
		public:
			
			virtual std::unordered_map<std::string, std::string> GetModuleDetails() = 0;

			virtual std::vector<std::vector<cv::Mat>> Forward(std::vector<std::vector<cv::Mat>>& inputs) = 0;

			virtual bool LoadModel(std::unordered_map<std::string, std::string> &args) = 0;

			virtual bool SetForwardRules(std::unordered_map<std::string, std::string> &args) = 0;

			virtual void UnloadModel() = 0;

			virtual ~MEDNNModule() {}
		};

#ifndef ME_DNN_MODULE_PROVIDER

		typedef MEDNNModule* (*createInstancePrototype)();
		typedef void (*destroyInstancePrototype)(MEDNNModule*);

		/**
		 * @class MEDNNModuleInstance
		 * @brief Class that handles the loading and unloading of the module DLL and the creation and destruction of the module instance.
		 *
		 * @remarks
		 * This class is responsible for loading the module DLL, creating an instance of the module, and unloading the DLL when the instance
		 * is destroyed. It also provides a set of functions that can be used to interact with the module instance.
		 */
		class MEDNN MEDNNModuleInstance {
		public:
			/**
			* @brief Creates an instance of the module.
			*
			* @remarks
			* This function creates an instance of the module by calling the createInstance() function in the module DLL.
			* The module DLL is loaded into memory if it is not already loaded.
			*
			* @param module_name Name of the module DLL. Can be the full path to the DLL or just the name of the DLL.
			*/
			MEDNNModuleInstance(const std::string module_name);

			/**
			 * @brief Retrieves information about the module, such as the framework name and version.
			 *
			 * @remarks
			 * This function returns an unordered map containing essential information about the module,
			 * such as the name of the underlying framework (e.g., Darknet, TensorFlow, PyTorch), versioning
			 * details, and any other relevant specifics.
			 *
			 * @return std::unordered_map<std::string, std::string> map containing information about the module.
			 */
			std::unordered_map<std::string, std::string> GetModuleDetails();

			/**
			 * @brief Processes the input data through the network and returns the resulting output.
			 *
			 * @remarks
			 * Takes a vector of vectors of cv::Mat objects as input, forwards each vector through the network
			 * as a batch, and returns a vector of vectors of cv::Mat objects containing the output data.
			 * The size of the nested vectors should be equal to the batch size of the network unless the
			 * network is configured to accept variable batch sizes.
			 *
			 * @param input std::vector<std::vector<cv::Mat>> object containing the input data to be processed.
			 * @return std::vector<std::vector<cv::Mat>> object containing the output data.
			 */
			std::vector<std::vector<cv::Mat>> Forward(std::vector<std::vector<cv::Mat>>& inputs);


			std::vector<cv::Mat> Forward(std::vector<cv::Mat>& inputs);

			cv::Mat Forward(cv::Mat& input);

			/**
			* @brief Takes an unordered map of arguments containing information about the model, such as the path to the model file,
			* configuration files, and weights, and loads the model into memory.
			*
			* @remarks
			* The arguments are passed as an unordered map of strings, where the key is the name of the argument and the value is the
			* value of the argument. For example, if the module accepts a path to the model file, the key would be "model_path" and the
			* value would be the path to the model file. If the module accepts different model formats, the format would also be specified
			* in the arguments.
			* The map can contain any number of arguments. As long as the arguments are relevant to the context of loading a model, they
			* will be accepted. Irrelevant arguments will be ignored.
			*
			* @param args std::unordered_map<std::string, std::string> object containing the arguments.
			*/
			bool LoadModel(std::unordered_map<std::string, std::string>& args);

			/**
			* @brief Sets any forward rules for the network, such as the input and output layer names, threshold values,
			* target device, etc.
			*
			* @remarks
			* The arguments are passed as an unordered map of strings, where the key is the name of the argument and the value is the
			* value of the argument. For example, if the module needs to know the name of the input layer, the key would be "input_layer"
			* and the value would be the name of the input layer.
			* This map can contain any number of arguments. As long as the arguments are relevant to the context of setting forward rules,
			* they will be accepted. Irrelevant arguments will be ignored.
			*
			* @param args std::unordered_map<std::string, std::string> object containing the arguments.
			*/
			bool SetForwardRules(std::unordered_map<std::string, std::string>& args);

			/**
			 * @brief Unloads the model from memory.
			 *
			 * @remarks
			 * Once the model is unloaded, the module can load a different model. Both LoadModel() and SetForwardRules()
			 * should be called again if not reloading the same model.
			 */
			void UnloadModel();

			~MEDNNModuleInstance();
		private:
			destroyInstancePrototype dllDestroyFunc;
			MEDNNModule* instance;
			HMODULE dll;
		};

#endif

	}

}

extern "C" {
	MECOREDNN me::dnn::MEDNNModule* createModuleInstance();
	MECOREDNN void destroyModuleInstance(me::dnn::MEDNNModule* instance);
}

#endif