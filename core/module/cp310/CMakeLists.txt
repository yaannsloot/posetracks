find_package(Python 3.10 EXACT REQUIRED COMPONENTS Interpreter Development)
find_package(pybind11 REQUIRED)
add_library(MECrypto.cp310 SHARED ${SOURCES_ME_CRYPTO_PYBIND})
add_library(MEUtils.cp310 SHARED ${SOURCES_ME_UTILS_PYBIND})
add_library(MEDNNBase.cp310 SHARED ${SOURCES_MEDNN_BASE_PYBIND})
target_link_libraries(MECrypto.cp310 PRIVATE
    Python::Python
    pybind11::module
    MECrypto
    MEUtils
)
target_link_libraries(MEUtils.cp310 PRIVATE
    ${OpenCV_LIBS}
    Python::Python
    pybind11::module
    MEUtils
)
target_link_libraries(MEDNNBase.cp310 PRIVATE 
    ${OpenCV_LIBS}
    Python::Python
    pybind11::module
    MEDNNBase
)
target_compile_features(MECrypto.cp310 PRIVATE cxx_std_17)
target_compile_features(MEUtils.cp310 PRIVATE cxx_std_17)
target_compile_features(MEDNNBase.cp310 PRIVATE cxx_std_17)
set_target_properties(MECrypto.cp310 PROPERTIES
    OUTPUT_NAME "MECrypto"
    SUFFIX ".pyd"
)
set_target_properties(MEUtils.cp310 PROPERTIES
    OUTPUT_NAME "MEUtils"
    SUFFIX ".pyd"
)
set_target_properties(MEDNNBase.cp310 PROPERTIES
    OUTPUT_NAME "MEDNNBase"
    SUFFIX ".pyd"
)

install(TARGETS MECrypto.cp310 RUNTIME DESTINATION ${CMAKE_BINARY_DIR}/redis/cp310)
install(TARGETS MEUtils.cp310 RUNTIME DESTINATION ${CMAKE_BINARY_DIR}/redis/cp310)
install(TARGETS MEDNNBase.cp310 RUNTIME DESTINATION ${CMAKE_BINARY_DIR}/redis/cp310)