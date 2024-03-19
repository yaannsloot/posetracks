find_package(Python 3.10 EXACT REQUIRED COMPONENTS Interpreter Development)
find_package(pybind11 REQUIRED)
add_library(MEPython.cp310 SHARED ${SOURCES_MOTIONENGINE_PYBIND})

target_link_libraries(MEPython.cp310 PRIVATE
    ${OpenCV_LIBS}
    Python::Python
    pybind11::module
    CUDA::cudart
    CUDA::curand
    CUDA::cublas
    CUDA::cuda_driver
    ${CUDNN_LIBRARIES}
    ${ONNXRUNTIME_LIBRARIES}
    me_objects
    ArrayFire::af
)

target_compile_features(MEPython.cp310 PRIVATE cxx_std_17)

if(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
    target_compile_options(MEPython.cp310 PRIVATE
        /arch:AVX2
        /Ob2
        /Oi
        /openmp:llvm
    )
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    target_compile_options(MEPython.cp310 PRIVATE
        -mavx2
        -O2
        -fopenmp
    )
endif()

set_target_properties(MEPython.cp310 PROPERTIES
    OUTPUT_NAME "MEPython"
    SUFFIX ".pyd"
)

install(TARGETS MEPython.cp310 RUNTIME DESTINATION ${CMAKE_BINARY_DIR}/redis/cp310)