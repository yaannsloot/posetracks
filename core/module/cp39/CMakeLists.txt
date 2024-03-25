message("Configuring build target for Python 3.9...")

find_package(Python 3.9 EXACT COMPONENTS Interpreter Development)

if(Python_FOUND)

find_package(pybind11 REQUIRED)
add_library(MEPython.cp39 SHARED ${SOURCES_MOTIONENGINE_PYBIND})

target_link_libraries(MEPython.cp39 PRIVATE
    ${OpenCV_LIBS}
    Python::Python
    pybind11::module
    ${CUDA_LIBRARIES}
    ${ONNXRUNTIME_LIBRARIES}
    me_objects
)

target_compile_features(MEPython.cp39 PRIVATE cxx_std_17)

set_target_properties(MEPython.cp39 PROPERTIES
    OUTPUT_NAME "MEPython"
    PREFIX ""
)

if(UNIX AND NOT APPLE)
    set_target_properties(MEPython.cp39 PROPERTIES
        SUFFIX ".so"
    )
    install(TARGETS MEPython.cp39 LIBRARY DESTINATION ${CMAKE_BINARY_DIR}/redis/cp39)
else()
    set_target_properties(MEPython.cp39 PROPERTIES
        SUFFIX ".pyd"
    )
    install(TARGETS MEPython.cp39 RUNTIME DESTINATION ${CMAKE_BINARY_DIR}/redis/cp39)
endif()

else()
message("No valid Python 3.9 installation found. Build target skipped.")

endif()