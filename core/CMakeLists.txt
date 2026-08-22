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
    set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
endif()

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/scripts")

# Required packages
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
find_package(OpenEXR REQUIRED)
find_package(ONNXRuntime REQUIRED)

# include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${OpenCV_INCLUDE_DIRS}
    ${ONNXRuntime_INCLUDE_DIR}
)

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
    ONNXRuntime::onnxruntime
    pcore_objects
)

if (CMAKE_CXX_COMPILER_ID STREQUAL "Clang"
     OR CMAKE_CXX_COMPILER_ID STREQUAL "GNU"
)
    set(LINK_LIST_BASE tbb ${LINK_LIST_BASE})
endif()
install(DIRECTORY ${CMAKE_SOURCE_DIR}/module/
    DESTINATION ${CMAKE_BINARY_DIR}/redis
    PATTERN "CMakeLists.txt" EXCLUDE
)

# Pybind Subdirectory
set(PYTHON_VERSIONS 3.9 3.10 3.11 3.12 3.13)
add_subdirectory(module)

# INSTALL Target setup
install(DIRECTORY ${CMAKE_SOURCE_DIR}/module/
    DESTINATION ${CMAKE_BINARY_DIR}/redis
    PATTERN "CMakeLists.txt" EXCLUDE
)

install(FILES ${CMAKE_SOURCE_DIR}/LICENSE DESTINATION ${CMAKE_BINARY_DIR}/redis)
install(FILES ${ONNXRuntime_SHARED_LIBS} DESTINATION ${CMAKE_BINARY_DIR}/redis/lib)
install(FILES ${ONNXRuntime_PROVIDERS} DESTINATION ${CMAKE_BINARY_DIR}/redis/lib)
