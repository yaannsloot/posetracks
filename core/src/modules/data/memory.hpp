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

#pragma once

#include <istream>
#include <array>

namespace me::data {

	class membuf : public std::basic_streambuf<char> {
	public:
		membuf(const uint8_t* p, size_t l) {
			setg((char*)p, (char*)p, (char*)p + l);
		}
	};

	class memstream : public std::istream {
	public:
		memstream(const uint8_t* p, size_t l) :
			std::istream(&_buffer),
			_buffer(p, l) {
			rdbuf(&_buffer);
		}

	private:
		membuf _buffer;
	};

	// Data wrapper for operating on flattened blobs
	template <std::size_t N, typename T>
	class Accessor {
	public:
		Accessor(T* data, const std::array<size_t, N>& dim)
			: data(data), dimensions(dim) {
			calculate_strides();
		}

		template <typename... Args>
		inline T& operator()(Args... args) {
			std::array<size_t, N> indices{ (size_t)args... };
			size_t index = 0;

			for (size_t i = 0; i < N; i++)
				index += indices[i] * strides[i];

			return data[index];
		}

	private:
		T* data;
		std::array<size_t, N> dimensions;
		std::array<size_t, N> strides;

		void calculate_strides() {
			size_t stride = 1;
			for (size_t i = N; i-- > 0; ) {
				strides[i] = stride;
				stride *= dimensions[i];
			}
		}
	};

}