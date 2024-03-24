message("Configuring build target for Python 3.10...")

find_package(Python 3.10 EXACT COMPONENTS Interpreter Development)

if(Python_FOUND)

find_package(pybind11 REQUIRED)
add_library(MEPython.cp310 SHARED ${SOURCES_MOTIONENGINE_PYBIND})

target_link_libraries(MEPython.cp310 PRIVATE
    ${OpenCV_LIBS}
    Python::Python
    pybind11::module
    ${CUDA_LIBRARIES}
    ${ONNXRUNTIME_LIBRARIES}
    me_objects
)

target_compile_features(MEPython.cp310 PRIVATE cxx_std_17)

set_target_properties(MEPython.cp310 PROPERTIES
    OUTPUT_NAME "MEPython"
    SUFFIX ".pyd"
)

install(TARGETS MEPython.cp310 RUNTIME DESTINATION ${CMAKE_BINARY_DIR}/redis/cp310)

else()
message("No valid Python 3.10 installation found. Build target skipped.")

endif()