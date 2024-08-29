message("Configuring build target for Python 3.10...")

find_package(Python 3.10 EXACT COMPONENTS Interpreter Development)

if(Python_FOUND)

find_package(pybind11 REQUIRED)
add_library(posetracks_core.cp310 SHARED ${SOURCES_POSETRACKS_PYBIND})

target_link_libraries(posetracks_core.cp310 PRIVATE
    ${LINK_LIST_BASE}
    Python::Python
    pybind11::module
)

target_compile_features(posetracks_core.cp310 PRIVATE cxx_std_17)

set_target_properties(posetracks_core.cp310 PROPERTIES
    OUTPUT_NAME "posetracks_core"
    PREFIX ""
)

if(UNIX AND NOT APPLE)
    set_target_properties(posetracks_core.cp310 PROPERTIES
        SUFFIX ".so"
        INSTALL_RPATH "$ORIGIN/../lib"
    )
    install(TARGETS posetracks_core.cp310 LIBRARY DESTINATION ${CMAKE_BINARY_DIR}/redis/cp310)
else()
    set_target_properties(posetracks_core.cp310 PROPERTIES
        SUFFIX ".pyd"
    )
    install(TARGETS posetracks_core.cp310 RUNTIME DESTINATION ${CMAKE_BINARY_DIR}/redis/cp310)
endif()

else()
message("No valid Python 3.10 installation found. Build target skipped.")

endif()