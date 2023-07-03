cmake_minimum_required (VERSION 3.10)

# Enable Hot Reload for MSVC compilers if supported.
if (POLICY CMP0141)
  cmake_policy(SET CMP0141 NEW)
  set(CMAKE_MSVC_DEBUG_INFORMATION_FORMAT "$<IF:$<AND:$<C_COMPILER_ID:MSVC>,$<CXX_COMPILER_ID:MSVC>>,$<$<CONFIG:Debug,RelWithDebInfo>:EditAndContinue>,$<$<CONFIG:Debug,RelWithDebInfo>:ProgramDatabase>>")
endif()

project ("MotionEngine")

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# Required packages
find_package(OpenCV REQUIRED)
find_package(CUDAToolkit REQUIRED)
find_package(CUDNN REQUIRED)
find_package(Darknet REQUIRED)
find_package(MMDeploy REQUIRED)

# include directories
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)
include_directories(${OpenCV_INCLUDE_DIRS})

# Source files
set(SOURCES_ME_CRYPTO
    ${CMAKE_CURRENT_SOURCE_DIR}/src/me_crypto.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/buildkey.cpp
)
set(SOURCES_ME_UTILS ${CMAKE_CURRENT_SOURCE_DIR}/src/me_utils.cpp)
set(SOURCES_MEDNN_BASE ${CMAKE_CURRENT_SOURCE_DIR}/src/me_dnn_base.cpp)
set(SOURCES_MEDNN_DARKNET ${CMAKE_CURRENT_SOURCE_DIR}/src/me_dnn_module_darknet.cpp)
set(SOURCES_MEDNN_OPENCV ${CMAKE_CURRENT_SOURCE_DIR}/src/me_dnn_module_opencv.cpp)
set(SOURCES_MEDNN_MMDEPLOY ${CMAKE_CURRENT_SOURCE_DIR}/src/me_dnn_module_mmdeploy.cpp)

# Python
set(SOURCES_ME_CRYPTO_PYBIND ${CMAKE_CURRENT_SOURCE_DIR}/src/me_crypto_bindings.cpp)
set(SOURCES_ME_UTILS_PYBIND ${CMAKE_CURRENT_SOURCE_DIR}/src/me_utils_bindings.cpp)
set(SOURCES_MEDNN_BASE_PYBIND ${CMAKE_CURRENT_SOURCE_DIR}/src/me_dnn_bindings.cpp)


# Target definitions
add_library(MECrypto SHARED ${SOURCES_ME_CRYPTO})
add_library(MEUtils SHARED ${SOURCES_ME_UTILS})
add_library(MEDNNBase SHARED ${SOURCES_MEDNN_BASE})
add_library(MEDNNDarknet SHARED ${SOURCES_MEDNN_DARKNET})
add_library(MEDNNOpenCV SHARED ${SOURCES_MEDNN_OPENCV})
add_library(MEDNNMMDeploy SHARED ${SOURCES_MEDNN_MMDEPLOY})


# Target linking
target_link_libraries(MEUtils PRIVATE
    ${OpenCV_LIBS}
)
target_link_libraries(MEDNNBase PRIVATE
    ${OpenCV_LIBS}
)
target_link_libraries(MEDNNDarknet PRIVATE
    ${OpenCV_LIBS}
    CUDA::cudart
    CUDA::curand
    CUDA::cublas
    CUDA::cuda_driver
    ${CUDNN_LIBRARIES}
    Darknet::dark
)
target_link_libraries(MEDNNOpenCV PRIVATE 
    ${OpenCV_LIBS}
    MEUtils
)
target_link_libraries(MEDNNMMDeploy PRIVATE
	${OpenCV_LIBS}
	mmdeploy
)
target_include_directories(MEDNNDarknet PRIVATE
    ${CUDA_INCLUDE_DIRS}
    ${Darknet_INCLUDE_DIR}
)

# Required compile flags
target_compile_definitions(MECrypto PRIVATE MECRYPTO_EXPORTS)
target_compile_definitions(MEUtils PRIVATE MEUTILS_EXPORTS)
target_compile_definitions(MEDNNBase PRIVATE ME_DNN_BASE_EXPORTS)
target_compile_definitions(MEDNNDarknet PRIVATE ME_DNN_MODULE_PROVIDER)
target_compile_definitions(MEDNNOpenCV PRIVATE ME_DNN_MODULE_PROVIDER)
target_compile_definitions(MEDNNMMDeploy PRIVATE ME_DNN_MODULE_PROVIDER)
target_compile_features(MECrypto PRIVATE cxx_std_17)
target_compile_features(MEUtils PRIVATE cxx_std_17)
target_compile_features(MEDNNBase PRIVATE cxx_std_17)
target_compile_features(MEDNNDarknet PRIVATE cxx_std_17)
target_compile_features(MEDNNOpenCV PRIVATE cxx_std_17)
target_compile_features(MEDNNMMDeploy PRIVATE cxx_std_17)

# Subdirectories
add_subdirectory(module/cp39)
add_subdirectory(module/cp310)

# INSTALL Target setup
install(DIRECTORY ${CMAKE_SOURCE_DIR}/module/
    DESTINATION ${CMAKE_BINARY_DIR}/redis
    PATTERN "CMakeLists.txt" EXCLUDE
)

install(TARGETS MECrypto RUNTIME DESTINATION ${CMAKE_BINARY_DIR}/redis)
install(TARGETS MEUtils RUNTIME DESTINATION ${CMAKE_BINARY_DIR}/redis)
install(TARGETS MEDNNBase RUNTIME DESTINATION ${CMAKE_BINARY_DIR}/redis)
install(TARGETS MEDNNDarknet RUNTIME DESTINATION ${CMAKE_BINARY_DIR}/redis)
install(TARGETS MEDNNOpenCV RUNTIME DESTINATION ${CMAKE_BINARY_DIR}/redis)
install(TARGETS MEDNNMMDeploy RUNTIME DESTINATION ${CMAKE_BINARY_DIR}/redis)

