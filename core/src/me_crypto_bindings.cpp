/*
me_crypto_bindings.cpp
Pybind11 module definitions for me_crypto.hpp

Copyright (C) 2023 Ian Sloat

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
*/

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <me_crypto.hpp>
#include <me_utils.hpp>
#include <fstream>

namespace py = pybind11;
using namespace me::crypto;
using namespace me::utility;

const SHA1Hash hashFileSHA1(std::string in) {
    std::ifstream input_file(in, std::ios::binary);
    return hashStreamSHA1(input_file);
}

const SHA1Hash hashBytesSHA1(std::vector<char>& in) {
    uint8_t* ptr = reinterpret_cast<uint8_t*>(in.data());
    memstream input_stream(ptr, in.size());
    return hashStreamSHA1(input_stream);
}

const SHA1Hash fileToHMAC_SHA1(std::string in, std::vector<char>& key) {
    std::ifstream input_file(in, std::ios::binary);
    uint8_t* key_ptr = reinterpret_cast<uint8_t*>(key.data());
    memstream input_key(key_ptr, key.size());
    return StreamToHMAC_SHA1(input_file, input_key);
}

const SHA1Hash bytesToHMAC_SHA1(std::vector<char>& in, std::vector<char>& key) {
    uint8_t* ptr = reinterpret_cast<uint8_t*>(in.data());
    memstream input_stream(ptr, in.size());
    uint8_t* key_ptr = reinterpret_cast<uint8_t*>(key.data());
    memstream input_key(key_ptr, key.size());
    return StreamToHMAC_SHA1(input_stream, input_key);
}

const py::bytes wrap_generateSHA1PRNGKey(std::vector<char>& init) {
    uint8_t* ptr = reinterpret_cast<uint8_t*>(init.data());
    memstream input_stream(ptr, init.size());
    std::stringstream out;
    generateSHA1PRNGKey(out, input_stream);
    return py::bytes(out.str());
}

const py::bytes getBuildkey() {
    return py::bytes(reinterpret_cast<const char*>(keyfile_bin), keyfile_bin_len);
}

PYBIND11_MODULE(MECrypto, m) {
    // Bind the SHA1Hash class
    py::class_<SHA1Hash>(m, "SHA1Hash")
        .def(py::init<uint32_t, uint32_t, uint32_t, uint32_t, uint32_t>())
        .def("get_h0", &SHA1Hash::get_h0)
        .def("get_h1", &SHA1Hash::get_h1)
        .def("get_h2", &SHA1Hash::get_h2)
        .def("get_h3", &SHA1Hash::get_h3)
        .def("get_h4", &SHA1Hash::get_h4)
        .def("to_string", &SHA1Hash::to_string);

    // Bind the functions in the me::crypto namespace
    m.def("createParityFile", &createParityFile, "Generates a parity file using a list of file paths as input. Used to create a basic representation of a file group.");
    m.def("hashFileSHA1", &hashFileSHA1, "Creates a hash value from the provided input file using the SHA-1 algorithm");
    m.def("hashBytesSHA1", &hashBytesSHA1, "Creates a hash value from the provided byte array using the SHA-1 algorithm");
    m.def("fileToHMAC_SHA1", &fileToHMAC_SHA1, "Creates a HMAC code from the provided input file using a variation of the HMAC-SHA-1 algorithm");
    m.def("bytesToHMAC_SHA1", &bytesToHMAC_SHA1, "Creates a HMAC code from the provided byte array using a variation of the HMAC-SHA-1 algorithm");
    m.def("generateSHA1PRNGKey", &wrap_generateSHA1PRNGKey, 
        "Creates a 1000 byte long key by continuously hashing the previous output of an SHA-1 hash function. This hash chain is created off of an initial set of data of arbitrary length.");
    m.def("getBuildkey", &getBuildkey, "Retrieves the build key for this version of MotionEngine. This key is used for versioning information in save data and other functions.");
}
