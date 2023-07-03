find_package(Python 3.9 EXACT REQUIRED COMPONENTS Interpreter Development)
find_package(pybind11 REQUIRED)
add_library(MECrypto.cp39 SHARED ${SOURCES_ME_CRYPTO_PYBIND})
add_library(MEUtils.cp39 SHARED ${SOURCES_ME_UTILS_PYBIND})
add_library(MEDNNBase.cp39 SHARED ${SOURCES_MEDNN_BASE_PYBIND})
target_link_libraries(MECrypto.cp39 PRIVATE
    Python::Python
    pybind11::module
    MECrypto
    MEUtils
)
target_link_libraries(MEUtils.cp39 PRIVATE
    ${OpenCV_LIBS}
    Python::Python
    pybind11::module
    MEUtils
)
target_link_libraries(MEDNNBase.cp39 PRIVATE 
    ${OpenCV_LIBS}
    Python::Python
    pybind11::module
    MEDNNBase
)
target_compile_features(MECrypto.cp39 PRIVATE cxx_std_17)
target_compile_features(MEUtils.cp39 PRIVATE cxx_std_17)
target_compile_features(MEDNNBase.cp39 PRIVATE cxx_std_17)
set_target_properties(MECrypto.cp39 PROPERTIES
    OUTPUT_NAME "MECrypto"
    SUFFIX ".pyd"
)
set_target_properties(MEUtils.cp39 PROPERTIES
    OUTPUT_NAME "MEUtils"
    SUFFIX ".pyd"
)
set_target_properties(MEDNNBase.cp39 PROPERTIES
    OUTPUT_NAME "MEDNNBase"
    SUFFIX ".pyd"
)

install(TARGETS MECrypto.cp39 RUNTIME DESTINATION ${CMAKE_BINARY_DIR}/redis/cp39)
install(TARGETS MEUtils.cp39 RUNTIME DESTINATION ${CMAKE_BINARY_DIR}/redis/cp39)
install(TARGETS MEDNNBase.cp39 RUNTIME DESTINATION ${CMAKE_BINARY_DIR}/redis/cp39)