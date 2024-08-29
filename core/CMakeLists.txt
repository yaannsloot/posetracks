cmake_minimum_required (VERSION 3.10)

# Enable Hot Reload for MSVC compilers if supported.
if(POLICY CMP0141)
  cmake_policy(SET CMP0141 NEW)
  set(CMAKE_MSVC_DEBUG_INFORMATION_FORMAT "$<IF:$<AND:$<C_COMPILER_ID:MSVC>,$<CXX_COMPILER_ID:MSVC>>,$<$<CONFIG:Debug,RelWithDebInfo>:EditAndContinue>,$<$<CONFIG:Debug,RelWithDebInfo>:ProgramDatabase>>")
endif()

project ("posetracks_core" LANGUAGES CXX)

message(STATUS "Using ${CMAKE_CXX_COMPILER_ID}")

set(ADDITIONAL_COMPILE_OPTIONS)

if (CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
    add_compile_options(
        /bigobj
        /arch:AVX2
        /Ob2
        /Oi
    )
elseif (CMAKE_CXX_COMPILER_ID STREQUAL "Clang"
     OR CMAKE_CXX_COMPILER_ID STREQUAL "GNU"
)
    add_compile_options(
        -fPIC
        -save-temps
        -mavx2
        -O2
    )
endif()

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# Required packages
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)

# Find onnxruntime which has no cmake support
message("Looking for ONNXRuntime...")
find_path(ONNXRUNTIME_ROOT_DIR NO_DEFAULT_PATH REQUIRED)
find_path(ONNXRUNTIME_INCLUDE_DIR ${ONNXRUNTIME_ROOT_DIR}/include REQUIRED)
find_library(ONNXRUNTIME_LIBRARY_BASE
    NAMES onnxruntime
    HINTS ${ONNXRUNTIME_ROOT_DIR}/lib ${ONNXRUNTIME_ROOT_DIR}/lib/x64 ${ONNXRUNTIME_ROOT_DIR}/lib/x86
    REQUIRED
)
find_library(ONNXRUNTIME_LIBRARY_SHARED
    NAMES onnxruntime_providers_shared
    HINTS ${ONNXRUNTIME_ROOT_DIR}/lib ${ONNXRUNTIME_ROOT_DIR}/lib/x64 ${ONNXRUNTIME_ROOT_DIR}/lib/x86
    REQUIRED
)
set(ONNXRUNTIME_LIBRARIES
    ${ONNXRUNTIME_LIBRARY_BASE}
    ${ONNXRUNTIME_LIBRARY_SHARED}
)

option(USING_CUDA ON)

if(USING_CUDA)
    message("Looking for ONNXRuntime CUDA extensions...")
    find_library(ONNXRUNTIME_LIBRARY_CUDA
        NAMES onnxruntime_providers_cuda
        HINTS ${ONNXRUNTIME_ROOT_DIR}/lib ${ONNXRUNTIME_ROOT_DIR}/lib/x64 ${ONNXRUNTIME_ROOT_DIR}/lib/x86
        REQUIRED
    )
    find_library(ONNXRUNTIME_LIBRARY_TENSORRT
        NAMES onnxruntime_providers_tensorrt
        HINTS ${ONNXRUNTIME_ROOT_DIR}/lib ${ONNXRUNTIME_ROOT_DIR}/lib/x64 ${ONNXRUNTIME_ROOT_DIR}/lib/x86
        REQUIRED
    )
    set(ONNXRUNTIME_LIBRARIES
        ${ONNXRUNTIME_LIBRARIES}
        ${ONNXRUNTIME_LIBRARY_CUDA}
        ${ONNXRUNTIME_LIBRARY_TENSORRT}
    )
endif()

# include directories
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${ONNXRUNTIME_INCLUDE_DIR})

set_property(GLOBAL PROPERTY USE_FOLDERS ON)
set_property(GLOBAL PROPERTY PREDEFINED_TARGETS_FOLDER "CMakeTargets")

# Python
set(SOURCES_POSETRACKS_PYBIND)

set(MODULE_LIST)

add_library(pcore_objects INTERFACE)
add_subdirectory(src)

foreach(module ${MODULE_LIST})
    add_dependencies(pcore_objects ${module})
    target_link_libraries(pcore_objects INTERFACE $<TARGET_OBJECTS:${module}>)
    set_target_properties(${module} PROPERTIES FOLDER "modules")
endforeach()

set(LINK_LIST_BASE
    ${OpenCV_LIBS}
    ${ONNXRUNTIME_LIBRARIES}
    pcore_objects
)

if (CMAKE_CXX_COMPILER_ID STREQUAL "Clang"
     OR CMAKE_CXX_COMPILER_ID STREQUAL "GNU"
)
    set(LINK_LIST_BASE tbb ${LINK_LIST_BASE})
endif()

# Pybind Subdirectories
add_subdirectory(module/cp39)
add_subdirectory(module/cp310)
add_subdirectory(module/cp311)

# INSTALL Target setup
install(DIRECTORY ${CMAKE_SOURCE_DIR}/module/
    DESTINATION ${CMAKE_BINARY_DIR}/redis
    PATTERN "CMakeLists.txt" EXCLUDE
)

install(FILES ${CMAKE_SOURCE_DIR}/LICENSE DESTINATION ${CMAKE_BINARY_DIR}/redis)
