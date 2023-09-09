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

# Find onnxruntime which has no cmake support
find_path(ONNXRUNTIME_ROOT_DIR NO_DEFAULT_PATH REQUIRED)
find_path(ONNXRUNTIME_INCLUDE_DIR ${ONNXRUNTIME_ROOT_DIR}/include/onnxruntime_cxx_api.h REQUIRED)
find_library(ONNXRUNTIME_LIBRARY_BASE
    NAMES onnxruntime
    HINTS ${ONNXRUNTIME_ROOT_DIR}/lib ${ONNXRUNTIME_ROOT_DIR}/lib/x64 ${ONNXRUNTIME_ROOT_DIR}/lib/x86
    REQUIRED
)
find_library(ONNXRUNTIME_LIBRARY_CUDA
    NAMES onnxruntime_providers_cuda
    HINTS ${ONNXRUNTIME_ROOT_DIR}/lib ${ONNXRUNTIME_ROOT_DIR}/lib/x64 ${ONNXRUNTIME_ROOT_DIR}/lib/x86
    REQUIRED
)
find_library(ONNXRUNTIME_LIBRARY_SHARED
    NAMES onnxruntime_providers_shared
    HINTS ${ONNXRUNTIME_ROOT_DIR}/lib ${ONNXRUNTIME_ROOT_DIR}/lib/x64 ${ONNXRUNTIME_ROOT_DIR}/lib/x86
    REQUIRED
)
find_library(ONNXRUNTIME_LIBRARY_TENSORRT
    NAMES onnxruntime_providers_tensorrt
    HINTS ${ONNXRUNTIME_ROOT_DIR}/lib ${ONNXRUNTIME_ROOT_DIR}/lib/x64 ${ONNXRUNTIME_ROOT_DIR}/lib/x86
    REQUIRED
)
set(ONNXRUNTIME_LIBRARIES
    ${ONNXRUNTIME_LIBRARY_BASE}
    ${ONNXRUNTIME_LIBRARY_CUDA}
    ${ONNXRUNTIME_LIBRARY_SHARED}
    ${ONNXRUNTIME_LIBRARY_TENSORRT}
)

# include directories
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${ONNXRUNTIME_INCLUDE_DIR})

# Primary source files
#set(SOURCES_MOTIONENGINE
#    ${CMAKE_CURRENT_SOURCE_DIR}/src/buildkey.cpp
#    ${CMAKE_CURRENT_SOURCE_DIR}/src/me_crypto.cpp
#    ${CMAKE_CURRENT_SOURCE_DIR}/src/me_dnn.cpp
#    ${CMAKE_CURRENT_SOURCE_DIR}/src/me_dnn_base.cpp
#    ${CMAKE_CURRENT_SOURCE_DIR}/src/me_dnn_module_mmdeploy.cpp
#    ${CMAKE_CURRENT_SOURCE_DIR}/src/me_dnn_module_opencv.cpp
#    ${CMAKE_CURRENT_SOURCE_DIR}/src/me_dnn_pose_model.cpp
#    ${CMAKE_CURRENT_SOURCE_DIR}/src/me_dnn_detection_model.cpp
#    ${CMAKE_CURRENT_SOURCE_DIR}/src/me_dnn_rtdetection_model.cpp
#    ${CMAKE_CURRENT_SOURCE_DIR}/src/me_dnn_detectpose_model.cpp
#    ${CMAKE_CURRENT_SOURCE_DIR}/src/me_dnn_marker_comparator.cpp
#    ${CMAKE_CURRENT_SOURCE_DIR}/src/me_utils.cpp
#)

# Source files for testing
set(SOURCES_ME_POSE_TEST 
    ${CMAKE_CURRENT_SOURCE_DIR}/src/me_core.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/me_core_simplepool.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/me_core_transcoder.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/me_dnn.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/me_dnn_pose_model.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/me_dnn_detection_model.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/me_dnn_rtdetection_model.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/me_dnn_detectpose_model.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/me_pose_test.cpp
)

# Python
set(SOURCES_MOTIONENGINE_PYBIND
    ${CMAKE_CURRENT_SOURCE_DIR}/src/me_python.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/me_core.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/me_core_simplepool.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/me_core_transcoder.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/me_dnn.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/me_dnn_pose_model.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/me_dnn_detection_model.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/me_dnn_rtdetection_model.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/me_dnn_detectpose_model.cpp
)


# Target definitions
#add_library(MotionEngine SHARED ${SOURCES_MOTIONENGINE})

# Testing targets
add_executable(MEPoseTest ${SOURCES_ME_POSE_TEST})


# Target linking
#target_link_libraries(MotionEngine PRIVATE
#    ${OpenCV_LIBS}
#    CUDA::cudart
#    CUDA::curand
#    CUDA::cublas
#    CUDA::cuda_driver
#    ${CUDNN_LIBRARIES}
#    ${ONNXRUNTIME_LIBRARIES}
#)

# Test linking
target_link_libraries(MEPoseTest PRIVATE
    ${OpenCV_LIBS}
    CUDA::cudart
    CUDA::curand
    CUDA::cublas
    CUDA::cuda_driver
    ${CUDNN_LIBRARIES}
    ${ONNXRUNTIME_LIBRARIES}
)

# Required compile flags
#target_compile_definitions(MotionEngine PRIVATE
#    MECRYPTO_EXPORTS
#    MEUTILS_EXPORTS
#    ME_DNN_BASE_EXPORTS
#    ME_DNN_MODULE_PROVIDER
#)
#target_compile_features(MotionEngine PRIVATE cxx_std_17)

# Test compile flags
target_compile_features(MEPoseTest PRIVATE cxx_std_17)

target_compile_options(MEPoseTest PRIVATE
    /arch:AVX2
    /Ob2
    /Oi
    /openmp
)

# Subdirectories
add_subdirectory(module/cp39)
add_subdirectory(module/cp310)

# INSTALL Target setup
install(DIRECTORY ${CMAKE_SOURCE_DIR}/module/
    DESTINATION ${CMAKE_BINARY_DIR}/redis
    PATTERN "CMakeLists.txt" EXCLUDE
)

install(FILES ${CMAKE_SOURCE_DIR}/LICENSE DESTINATION ${CMAKE_BINARY_DIR}/redis)

#install(TARGETS MotionEngine RUNTIME DESTINATION ${CMAKE_BINARY_DIR}/redis)

# Testing
install(TARGETS MEPoseTest RUNTIME DESTINATION ${CMAKE_BINARY_DIR}/tests)

