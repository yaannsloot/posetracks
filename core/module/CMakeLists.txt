function(PyConfig ver)
message("Configuring build target for Python ${ver}...")
find_package(Python ${ver} EXACT COMPONENTS Interpreter Development)
if(Python_FOUND)
find_package(pybind11 REQUIRED)
string(REPLACE "." "" MOD_NAME ${ver})
set(MOD_NAME "ptcore_cp${MOD_NAME}")
add_library(${MOD_NAME} SHARED ${SOURCES_POSETRACKS_PYBIND})
target_link_libraries(${MOD_NAME} PRIVATE
    ${LINK_LIST_BASE}
    Python::Python
    pybind11::module
)
target_compile_features(${MOD_NAME} PRIVATE cxx_std_17)
target_compile_definitions(${MOD_NAME} PRIVATE PYBIND_MODULE_NAME=${MOD_NAME})
if(UNIX AND NOT APPLE)
    set_target_properties(${MOD_NAME} PROPERTIES
        SUFFIX ".so"
        BUILD_RPATH "\$ORIGIN/lib"
        INSTALL_RPATH "\$ORIGIN/lib"
    )
    install(TARGETS ${MOD_NAME} LIBRARY DESTINATION ${CMAKE_BINARY_DIR}/redis)
else()
    set_target_properties(${MOD_NAME} PROPERTIES
        SUFFIX ".pyd"
    )
    target_link_options(${MOD_NAME} PRIVATE "/DELAYLOAD:onnxruntime.dll")
    install(TARGETS ${MOD_NAME} RUNTIME DESTINATION ${CMAKE_BINARY_DIR}/redis)
endif()
else()
message("No valid Python ${ver} installation found. Build target skipped.")
endif()
endfunction()

foreach(VER IN LISTS PYTHON_VERSIONS)
    PyConfig(${VER})
endforeach()
