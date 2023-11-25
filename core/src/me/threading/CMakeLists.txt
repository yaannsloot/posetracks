get_filename_component(module_name ${CMAKE_CURRENT_SOURCE_DIR} NAME)

message("  found ${module_name}")

file(GLOB MODULE_SOURCES "*.cpp")

add_library(${module_name} OBJECT ${MODULE_SOURCES})

target_compile_features(${module_name} PRIVATE cxx_std_17)

target_compile_options(${module_name} PRIVATE
    /arch:AVX2
    /Ob2
    /Oi
    /openmp
)

set(MODULE_LIST ${MODULE_LIST} ${module_name})
set(MODULE_LIST ${MODULE_LIST} PARENT_SCOPE)
