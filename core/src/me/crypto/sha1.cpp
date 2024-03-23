/*
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
*/

#include "sha1.hpp"

#include <iomanip>
#include <sstream>
#include <random>
#include <cstring>

namespace me {

	namespace crypto {

		SHA1Hash::SHA1Hash(uint32_t h0, uint32_t h1, uint32_t h2, uint32_t h3, uint32_t h4) {
			this->h0 = h0;
			this->h1 = h1;
			this->h2 = h2;
			this->h3 = h3;
			this->h4 = h4;
			uint32_t hash[5] = { h0, h1, h2, h3, h4 };
			for (int i = 0; i < 5; i++) {
				for (int j = 0; j < 4; j++) {
					bytes[i * 4 + j] = (uint8_t)(hash[i] >> 8 * (3 - j));
				}
			}
		}

		uint32_t SHA1Hash::get_h0() const {
			return h0;
		}

		uint32_t SHA1Hash::get_h1() const {
			return h1;
		}

		uint32_t SHA1Hash::get_h2() const {
			return h2;
		}

		uint32_t SHA1Hash::get_h3() const {
			return h3;
		}

		uint32_t SHA1Hash::get_h4() const {
			return h4;
		}

		std::string SHA1Hash::to_string() const {
			std::stringstream ss;
			for (int i = 0; i < 20; i++) {
				ss << std::setfill('0') << std::setw(2) << std::right << std::hex << +bytes[i];
			}
			return ss.str();
		}

		const uint8_t* SHA1Hash::to_bytes() const {
			return &bytes[0];
		}

		const SHA1Hash hashStreamSHA1(std::istream& in) {
			// original message length
			in.seekg(0, in.end);
			uint64_t length = (uint64_t)in.tellg() * 8;
			uint32_t l0 = (uint32_t)(length >> 32);
			uint32_t l1 = (uint32_t)length;
			in.seekg(0, in.beg);
			size_t oi = (length % 512) / 8;
			length += 8;
			uint64_t o_frame = length + (512 - (length % 512));
			if (length % 512 == 0 || length % 512 > 448)
				length += 512;
			length += 512 - (length % 512);

			// Hash vars
			uint32_t h0 = 0x67452301;
			uint32_t h1 = 0xEFCDAB89;
			uint32_t h2 = 0x98BADCFE;
			uint32_t h3 = 0x10325476;
			uint32_t h4 = 0xC3D2E1F0;

			// Chunk processing
			for (uint64_t i = 512; i <= length; i += 512) {
				uint8_t chunk[64];
				memset(chunk, 0, 64);
				in.read((char*)chunk, 64);
				if (i == o_frame)
					chunk[oi] = 0x80;
				uint32_t schedule[80];
				memset(schedule, 0, 80);
				for (int j = 0; j < 16; j++) {
					for (int k = 0; k < 4; k++) {
						schedule[j] |= ((uint32_t)chunk[j * 4 + k] << 8 * (3 - k));
					}
				}
				if (i == length) {
					schedule[14] = l0;
					schedule[15] = l1;
				}
				for (int j = 16; j < 80; j++) {
					schedule[j] = schedule[j - 3] ^ schedule[j - 8] ^ schedule[j - 14] ^ schedule[j - 16];
					schedule[j] = (schedule[j] << 1) | (schedule[j] >> 31);
				}
				uint32_t a = h0;
				uint32_t b = h1;
				uint32_t c = h2;
				uint32_t d = h3;
				uint32_t e = h4;
				for (int j = 0; j < 80; j++) {
					uint32_t f;
					uint32_t k;
					if (0 <= j && j <= 19) {
						f = (b & c) | ((~b) & d);
						k = 0x5A827999;
					}
					else if (20 <= j && j <= 39) {
						f = b ^ c ^ d;
						k = 0x6ED9EBA1;
					}
					else if (40 <= j && j <= 59) {
						f = (b & c) | (b & d) | (c & d);
						k = 0x8F1BBCDC;
					}
					else {
						f = b ^ c ^ d;
						k = 0xCA62C1D6;
					}
					uint32_t t = ((a << 5) | (a >> 27)) + f + e + k + schedule[j];
					e = d;
					d = c;
					c = ((b << 30) | (b >> 2));
					b = a;
					a = t;
				}
				h0 += a;
				h1 += b;
				h2 += c;
				h3 += d;
				h4 += e;
			}
			in.seekg(0, in.beg);
			return SHA1Hash(h0, h1, h2, h3, h4);
		}

		const SHA1Hash generateRandomSHA1(int r_bytes_size) {
			std::random_device rd;
			std::mt19937 gen(rd());
			std::uniform_int_distribution<> dis(0, 255);

			std::stringstream random_bytes;
			for (int i = 0; i < r_bytes_size; ++i) {
				random_bytes << static_cast<char>(dis(gen));
			}

			return hashStreamSHA1(random_bytes);
		}

	}

}