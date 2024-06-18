message("Configuring build target for Python 3.11...")

find_package(Python 3.11 EXACT COMPONENTS Interpreter Development)

if(Python_FOUND)

find_package(pybind11 REQUIRED)
add_library(MEPython.cp311 SHARED ${SOURCES_MOTIONENGINE_PYBIND})

target_link_libraries(MEPython.cp311 PRIVATE
    ${OpenCV_LIBS}
    Python::Python
    pybind11::module
    ${ONNXRUNTIME_LIBRARIES}
    me_objects
)

target_compile_features(MEPython.cp311 PRIVATE cxx_std_17)

set_target_properties(MEPython.cp311 PROPERTIES
    OUTPUT_NAME "MEPython"
    PREFIX ""
)

if(UNIX AND NOT APPLE)
    set_target_properties(MEPython.cp311 PROPERTIES
        SUFFIX ".so"
        INSTALL_RPATH "$ORIGIN/../lib"
    )
    install(TARGETS MEPython.cp311 LIBRARY DESTINATION ${CMAKE_BINARY_DIR}/redis/cp311)
else()
    set_target_properties(MEPython.cp311 PROPERTIES
        SUFFIX ".pyd"
    )
    install(TARGETS MEPython.cp311 RUNTIME DESTINATION ${CMAKE_BINARY_DIR}/redis/cp311)
endif()

else()
message("No valid Python 3.11 installation found. Build target skipped.")

endif()