message("Configuring build target for Python 3.10...")

find_package(Python 3.10 EXACT COMPONENTS Interpreter Development)

if(Python_FOUND)

find_package(pybind11 REQUIRED)
add_library(MEPython.cp310 SHARED ${SOURCES_MOTIONENGINE_PYBIND})

target_link_libraries(MEPython.cp310 PRIVATE
    ${OpenCV_LIBS}
    Python::Python
    pybind11::module
    ${ONNXRUNTIME_LIBRARIES}
    me_objects
)

target_compile_features(MEPython.cp310 PRIVATE cxx_std_17)

set_target_properties(MEPython.cp310 PROPERTIES
    OUTPUT_NAME "MEPython"
    PREFIX ""
)

if(UNIX AND NOT APPLE)
    set_target_properties(MEPython.cp310 PROPERTIES
        SUFFIX ".so"
        INSTALL_RPATH "$ORIGIN/../lib"
    )
    install(TARGETS MEPython.cp310 LIBRARY DESTINATION ${CMAKE_BINARY_DIR}/redis/cp310)
else()
    set_target_properties(MEPython.cp310 PROPERTIES
        SUFFIX ".pyd"
    )
    install(TARGETS MEPython.cp310 RUNTIME DESTINATION ${CMAKE_BINARY_DIR}/redis/cp310)
endif()

else()
message("No valid Python 3.10 installation found. Build target skipped.")

endif()