'''
Copyright (C) 2024 Ian Sloat

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
'''

import io
import os
import shutil
import zipfile
import tarfile
import platform
import subprocess
import urllib.request
from urllib.parse import urlparse

download_dir = '3rd_party'

# Download links to required packages and tools
vswhere_dl = "https://github.com/microsoft/vswhere/releases/download/3.1.7/vswhere.exe"
onnxruntime_dl_win = "https://github.com/microsoft/onnxruntime/releases/download/v1.18.0/onnxruntime-win-x64-gpu-cuda12-1.18.0.zip"
onnxruntime_dl_linux = "https://github.com/microsoft/onnxruntime/releases/download/v1.18.0/onnxruntime-linux-x64-gpu-cuda12-1.18.0.tgz"
opencv_dl = "https://github.com/opencv/opencv/archive/refs/tags/4.10.0.tar.gz"
opencv_contrib_dl = "https://github.com/opencv/opencv_contrib/archive/refs/tags/4.10.0.tar.gz"
eigen_dl = "https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz"
glog_dl = "https://github.com/google/glog/archive/refs/tags/v0.6.0.tar.gz"
gflags_dl = "https://github.com/gflags/gflags/archive/refs/tags/v2.2.2.tar.gz"
ceres_dl = "http://ceres-solver.org/ceres-solver-2.2.0.tar.gz"
pybind_dl = "https://github.com/pybind/pybind11/archive/refs/tags/v2.12.0.tar.gz"
zlib_dl_win = "http://www.winimage.com/zLibDll/zlib123dllx64.zip"

# Platform vars
pathsep = os.path.pathsep
if platform.system() == 'Linux':
    build_platform = 'linux'
elif platform.system() == 'Windows':
    build_platform = 'windows'
else:
    build_platform = 'mac'


def fullpath(path):
    path = os.path.expanduser(path)
    path = os.path.normpath(path)
    path = os.path.realpath(path)
    return os.path.abspath(path)


def prepare_directory(directory, delete_old=True):
    directory = fullpath(directory)
    if os.path.exists(directory):
        if delete_old:
            shutil.rmtree(directory)
            os.mkdir(directory)
        return
    os.mkdir(directory)


def invoke_command_output(cmd, *args, **kwargs):
    cmd = fullpath(shutil.which(cmd))
    arg_list = [cmd]
    for arg in args:
        arg_list.append(arg)
    for k, v in kwargs.items():
        kwarg_list.extend([k, v])
    return subprocess.check_output(arg_list)


def invoke_command(cmd, *args, **kwargs):
    cmd = fullpath(shutil.which(cmd))
    arg_list = [cmd]
    for arg in args:
        arg_list.append(arg)
    for k, v in kwargs.items():
        kwarg_list.extend([k, v])
    return subprocess.check_call(arg_list)


def download_file(url, out_dir='.', filename=None):
    a = urlparse(url)
    if filename is None:
        filename = os.path.basename(a.path)
    out_file = os.path.join(out_dir, filename)
    out_file = fullpath(out_file)
    urllib.request.urlretrieve(url, out_file)


def get_msvc_versions():
    vswhere_path = os.path.join(download_dir, 'vswhere.exe')
    vswhere_path = fullpath(vswhere_path)
    if not os.path.exists(vswhere_path):
        print('Downloading vswhere.exe...')
        download_file(vswhere_dl, download_dir)
    product_prefix = 'Microsoft.VisualStudio.Product'
    products = ['BuildTools', 'Community', 'Professional', 'Enterprise']
    products = [f'{product_prefix}.{prod}' for prod in products]
    products.insert(0, '-products')
    vswhere_output = invoke_command_output(vswhere_path, *products).decode('utf-8')
    buf = io.StringIO(vswhere_output)
    all_versions = {}
    current_ver = {}
    for line in buf.readlines():
        line = line.strip()
        pair = line.split(':')
        if len(pair) < 2:
            if len(current_ver) > 0:
                all_versions[current_ver['displayName']] = current_ver
                current_ver = {}
            continue
        key = pair[0].strip()
        value = ':'.join(pair[1:]).strip()
        current_ver[key] = value
    if len(current_ver) > 0:
        all_versions[current_ver['displayName']] = current_ver
    all_versions = list(all_versions.items())
    all_versions.sort(key=lambda item: item[0], reverse=True)
    for msvc_ver in all_versions:
        release_year = msvc_ver[1]['catalog_productLineVersion']
        major_version = msvc_ver[1]['catalog_buildVersion'].split('.')[0]
        msvc_ver[1]['cmake_generator'] = f"Visual Studio {major_version} {release_year}"
    return all_versions


def find_pkg_root(pkg_prefix):
    pkg_path = None
    dl_dir = fullpath(download_dir)
    for path in os.listdir(dl_dir):
        abs_path = os.path.join(dl_dir, path)
        if os.path.isdir(abs_path) and path.startswith(pkg_prefix):
            pkg_path = abs_path
            break
    return pkg_path


def find_package(name, source, root_dir=''):
    extract_dir = fullpath(download_dir)
    if len(root_dir) > 0:
        extract_dir = os.path.join(extract_dir, root_dir)
    a = urlparse(source)
    src_name = os.path.basename(a.path)
    src_ext = os.path.splitext(src_name)[1]
    if src_ext != '.zip':
        src_ext = '.tar.gz'
    pkg_name = name + src_ext
    pkg_abspath = fullpath(os.path.join(download_dir, pkg_name))
    if not os.path.exists(pkg_abspath):
        print(f'Downloading {pkg_name}...')
        download_file(source, download_dir, pkg_name)
    pkg_path = find_pkg_root(name)
    if pkg_path is None:
        print(f'Extracting {pkg_name}...')
        if src_ext.endswith('.zip'):
            with zipfile.ZipFile(pkg_abspath, 'r') as pkg:
                pkg.extractall(extract_dir)
        else:
            with tarfile.open(pkg_abspath, 'r') as pkg:
                pkg.extractall(extract_dir)
    pkg_path = find_pkg_root(name)
    pkg_build_path = os.path.join(pkg_path, 'build')
    if os.path.isfile(pkg_build_path):
        os.remove(pkg_build_path)
    if not os.path.exists(pkg_build_path):
        os.mkdir(pkg_build_path)
    return pkg_path, pkg_build_path


def check_ceres(cmake_generator):
    eigen_path, eigen_build_path = find_package('eigen', eigen_dl)
    glog_path, glog_build_path = find_package('glog', glog_dl)
    gflags_path, gflags_build_path = find_package('gflags', gflags_dl)
    ceres_path, ceres_build_path = find_package('ceres', ceres_dl)
    print('Building GLog...')
    invoke_command('cmake', '-S', glog_path, '-B', glog_build_path, '-G', cmake_generator,
                   '-DBUILD_SHARED_LIBS=OFF')
    invoke_command('cmake', '--build', glog_build_path, '--config', 'Release')
    invoke_command('cmake', '--install', glog_build_path, '--prefix',
                   os.path.join(glog_build_path, 'install'))
    print('Building GFlags...')
    invoke_command('cmake', '-S', gflags_path, '-B', gflags_build_path, '-G', cmake_generator,
                   '-DBUILD_SHARED_LIBS=OFF')
    invoke_command('cmake', '--build', gflags_build_path, '--config', 'Release')
    invoke_command('cmake', '--install', gflags_build_path, '--prefix',
                   os.path.join(gflags_build_path, 'install'))
    print('Setting up Eigen...')
    invoke_command('cmake', '-S', eigen_path, '-B', eigen_build_path, '-G', cmake_generator,
                   '-DBUILD_TESTING=OFF')
    invoke_command('cmake', '--build', eigen_build_path, '--config', 'Release')
    invoke_command('cmake', '--install', eigen_build_path, '--prefix',
                   os.path.join(eigen_build_path, 'install'))
    eigen_cmake_dir = os.path.join(eigen_build_path, 'install', 'share', 'eigen3', 'cmake')
    glog_cmake_dir = os.path.join(glog_build_path, 'install', 'lib', 'cmake', 'glog')
    gflags_cmake_dir = os.path.join(gflags_build_path, 'install', 'lib', 'cmake', 'gflags')
    print('Building Ceres Solver...')
    ceres_prefix_path = ';'.join([eigen_cmake_dir, glog_cmake_dir, gflags_cmake_dir])
    invoke_command('cmake', '-S', ceres_path, '-B', ceres_build_path, '-G', cmake_generator,
                   '-DBUILD_SHARED_LIBS=OFF', f'-DCMAKE_PREFIX_PATH={ceres_prefix_path}',
                   '-DBUILD_BENCHMARKS=OFF', '-DBUILD_DOCUMENTATION=OFF', '-DBUILD_EXAMPLES=OFF',
                   '-DBUILD_TESTING=OFF', '-DUSE_CUDA=OFF')
    invoke_command('cmake', '--build', ceres_build_path, '--config', 'Release')
    invoke_command('cmake', '--install', ceres_build_path, '--prefix',
                   os.path.join(ceres_build_path, 'install'))
    ceres_cmake_dir = os.path.join(ceres_build_path, 'install', 'lib', 'cmake', 'Ceres')
    return ';'.join([ceres_cmake_dir, ceres_prefix_path])


def check_opencv(cmake_generator, ceres_cmake):
    opencv_path, opencv_build_path = find_package('opencv-4.10.0', opencv_dl)
    opencv_contrib_path, _ = find_package('opencv_contrib-4.10.0', opencv_contrib_dl)
    print('Building OpenCV...')
    invoke_command('cmake', '-S', opencv_path, '-B', opencv_build_path, '-G', cmake_generator,
                   f'-DCMAKE_PREFIX_PATH={ceres_cmake}',
                   '-DBUILD_SHARED_LIBS=OFF', '-DBUILD_WITH_STATIC_CRT=OFF',
                   '-DBUILD_opencv_world=OFF', '-DBUILD_DOCS=OFF', '-DBUILD_EXAMPLES=OFF',
                   f'-DOPENCV_EXTRA_MODULES_PATH={os.path.join(opencv_contrib_path, "modules")}',
                   '-DBUILD_JAVA=OFF', '-DBUILD_opencv_python3=OFF', '-DBUILD_opencv_python_bindings_generator=OFF',
                   '-DBUILD_opencv_python_tests=OFF', '-DWITH_CUDA=OFF', '-DBUILD_PERF_TESTS=OFF', '-DBUILD_TESTS=OFF',
                   '-DBUILD_opencv_apps=OFF', '-DBUILD_opencv_wechat_qrcode=OFF')
    invoke_command('cmake', '--build', opencv_build_path, '--config', 'Release')
    invoke_command('cmake', '--install', opencv_build_path, '--prefix',
                   os.path.join(opencv_build_path, 'install'))
    if build_platform == 'windows':
        opencv_install_dir = os.path.join(opencv_build_path, 'install', 'x64')
        return os.path.join(opencv_install_dir, os.listdir(opencv_install_dir)[0], 'staticlib')
    elif build_platform == 'linux':
        return os.path.join(opencv_build_path, 'install', 'lib', 'cmake', 'opencv4')
    else:
        return # put macos stuff here


def check_onnx():
    if build_platform == 'windows':
        onnx_path, _ = find_package('onnxruntime', onnxruntime_dl_win)
    elif build_platform == 'linux':
        onnx_path, _ = find_package('onnxruntime', onnxruntime_dl_linux)
    onnx_path_lib = os.path.join(onnx_path, 'lib')
    onnx_path_include = os.path.join(onnx_path, 'include')
    if build_platform == 'windows':
        onnxruntime_dll = os.path.join(onnx_path_lib, 'onnxruntime.dll')
        onnxruntime_lib = os.path.join(onnx_path_lib, 'onnxruntime.lib')
        onnxruntime_providers_cuda_dll = os.path.join(onnx_path_lib, 'onnxruntime_providers_cuda.dll')
        onnxruntime_providers_cuda_lib = os.path.join(onnx_path_lib, 'onnxruntime_providers_cuda.lib')
        onnxruntime_providers_shared_dll = os.path.join(onnx_path_lib, 'onnxruntime_providers_shared.dll')
        onnxruntime_providers_shared_lib = os.path.join(onnx_path_lib, 'onnxruntime_providers_shared.lib')
        onnxruntime_providers_tensorrt_dll = os.path.join(onnx_path_lib, 'onnxruntime_providers_tensorrt.dll')
        onnxruntime_providers_tensorrt_lib = os.path.join(onnx_path_lib, 'onnxruntime_providers_tensorrt.lib')
    # For linux just set the .so files for both lib and dll vars
    elif build_platform == 'linux':
        onnxruntime_dll = os.path.join(onnx_path_lib, 'libonnxruntime.so.1.18.0')
        onnxruntime_lib = os.path.join(onnx_path_lib, 'libonnxruntime.so.1.18.0')
        onnxruntime_providers_cuda_dll = os.path.join(onnx_path_lib, 'libonnxruntime_providers_cuda.so')
        onnxruntime_providers_cuda_lib = os.path.join(onnx_path_lib, 'libonnxruntime_providers_cuda.so')
        onnxruntime_providers_shared_dll = os.path.join(onnx_path_lib, 'libonnxruntime_providers_shared.so')
        onnxruntime_providers_shared_lib = os.path.join(onnx_path_lib, 'libonnxruntime_providers_shared.so')
        onnxruntime_providers_tensorrt_dll = os.path.join(onnx_path_lib, 'libonnxruntime_providers_tensorrt.so')
        onnxruntime_providers_tensorrt_lib = os.path.join(onnx_path_lib, 'libonnxruntime_providers_tensorrt.so')
    include_check = os.path.exists(onnx_path_include)
    base_lib_check = os.path.exists(onnxruntime_dll) and os.path.exists(onnxruntime_lib)
    cuda_lib_check = os.path.exists(onnxruntime_providers_cuda_dll) and os.path.exists(onnxruntime_providers_cuda_lib)
    shared_lib_check = (os.path.exists(onnxruntime_providers_shared_dll) and
                        os.path.exists(onnxruntime_providers_shared_lib))
    tensorrt_lib_check = (os.path.exists(onnxruntime_providers_tensorrt_dll) and
                          os.path.exists(onnxruntime_providers_tensorrt_lib))
    success = include_check and base_lib_check and cuda_lib_check and shared_lib_check and tensorrt_lib_check
    if not success:
        raise FileNotFoundError('Some files were missing from onnxruntime')
    return (onnx_path, onnx_path_include, onnxruntime_dll, onnxruntime_lib, onnxruntime_providers_shared_dll,
            onnxruntime_providers_shared_lib, onnxruntime_providers_cuda_dll, onnxruntime_providers_cuda_lib,
            onnxruntime_providers_tensorrt_dll, onnxruntime_providers_tensorrt_lib)


def main():
    # Setup
    prepare_directory(download_dir, False)
    if build_platform == 'windows':
        print('Checking for MSVC...')
        msvc_vers = get_msvc_versions()
        if len(msvc_vers) == 0:
            print('ERROR: Could not find MSVC build tools. Aborting...')
        msvc = msvc_vers[0]
        generator = msvc[1]['cmake_generator']
        print(f'Found {msvc[0]}')
    else:
        generator = 'Unix Makefiles'
    print('Checking for Ceres Solver...')
    ceres_cmake = check_ceres(generator)
    print('Checking for OpenCV...')
    opencv_cmake = check_opencv(generator, ceres_cmake)
    print('Checking for ONNX Runtime...')
    onnx_vars = check_onnx()
    pybind_path, pybind_build_path = find_package('pybind', pybind_dl)
    print('Setting up pybind11...')
    invoke_command('cmake', '-S', pybind_path, '-B', pybind_build_path, '-G', generator,
                   '-DPYBIND11_TEST=OFF')
    invoke_command('cmake', '--build', pybind_build_path, '--config', 'Release')
    invoke_command('cmake', '--install', pybind_build_path, '--prefix',
                   os.path.join(pybind_build_path, 'install'))
    pybind_cmake = os.path.join(pybind_build_path, 'install', 'share', 'cmake', 'pybind11')
    if build_platform == 'windows':
        print('Checking for zlib...')
        zlib_path = find_package('zlib', zlib_dl_win, 'zlib')
        zlib_shared_path = os.path.join(zlib_path[0], 'dll_x64', 'zlibwapi.dll')

    # Build
    print('Building MotionEngine Core...')
    me_path = fullpath(os.path.dirname(os.path.realpath(__file__)))
    me_build_path = os.path.join(me_path, 'build')
    prepare_directory(me_build_path)
    me_prefix_path = ';'.join([opencv_cmake, ceres_cmake, pybind_cmake])
    print(me_prefix_path)
    invoke_command('cmake', '-S', me_path, '-B', me_build_path, '-G', generator,
                   f'-DCMAKE_PREFIX_PATH={me_prefix_path}', f'-DONNXRUNTIME_ROOT_DIR={onnx_vars[0]}',
                   f'-DONNXRUNTIME_INCLUDE_DIR={onnx_vars[1]}', '-DUSING_CUDA=ON')
    invoke_command('cmake', '--build', me_build_path, '--config', 'Release')
    invoke_command('cmake', '--install', me_build_path)
    me_redis_path = os.path.join(me_build_path, 'redis')
    if build_platform == 'windows':
        me_redis_bin_path = os.path.join(me_redis_path, 'bin')
    else:
        me_redis_bin_path = os.path.join(me_redis_path, 'lib')
    prepare_directory(me_redis_bin_path)
    shutil.copy(onnx_vars[2], me_redis_bin_path)
    shutil.copy(onnx_vars[4], me_redis_bin_path)
    shutil.copy(onnx_vars[6], me_redis_bin_path)
    shutil.copy(onnx_vars[8], me_redis_bin_path)
    if build_platform == 'windows':
        shutil.copy(zlib_shared_path, me_redis_bin_path)
    print('Build complete!')


if __name__ == "__main__":
    main()
