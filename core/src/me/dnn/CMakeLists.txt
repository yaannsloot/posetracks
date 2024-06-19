get_filename_component(module_name ${CMAKE_CURRENT_SOURCE_DIR} NAME)

message("  found ${module_name}")

file(GLOB MODULE_SOURCES "*.cpp")

add_library(${module_name} OBJECT ${MODULE_SOURCES})

target_compile_features(${module_name} PRIVATE cxx_std_17)

if(USING_CUDA)
    target_compile_definitions(${module_name} PRIVATE ME_CUDA_ENABLED)
endif()

set(MODULE_LIST ${MODULE_LIST} ${module_name})
set(MODULE_LIST ${MODULE_LIST} PARENT_SCOPE)
