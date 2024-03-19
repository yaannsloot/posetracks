get_filename_component(module_name ${CMAKE_CURRENT_SOURCE_DIR} NAME)

message("  found ${module_name}")

file(GLOB MODULE_SOURCES "*.cpp")

add_library(${module_name} OBJECT ${MODULE_SOURCES})

target_compile_features(${module_name} PRIVATE cxx_std_17)

if(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
    target_compile_options(${module_name} PRIVATE
        /arch:AVX2
        /Ob2
        /Oi
        /openmp:llvm
    )
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    target_compile_options(${module_name} PRIVATE
        -mavx2
        -O2
        -fopenmp
    )
endif()

set(MODULE_LIST ${MODULE_LIST} ${module_name})
set(MODULE_LIST ${MODULE_LIST} PARENT_SCOPE)
