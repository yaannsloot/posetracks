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

#include <cstdint>
#include <istream>

namespace me::crypto {

	/// <summary>
	/// A class to represent a SHA1 hash
	/// </summary>
	class SHA1Hash {
	public:
		/// <summary>
		/// Constructs a SHA1Hash object from the provided hash values
		/// </summary>
		/// <param name="h0:">The first 32 bits of the hash</param>
		/// <param name="h1:">The second 32 bits of the hash</param>
		/// <param name="h2:">The third 32 bits of the hash</param>
		/// <param name="h3:">The fourth 32 bits of the hash</param>
		/// <param name="h4:">The fifth 32 bits of the hash</param>
		SHA1Hash(uint32_t h0, uint32_t h1, uint32_t h2, uint32_t h3, uint32_t h4);

		/// <summary>
		/// Returns the first 32 bits of the hash
		/// </summary>
		uint32_t get_h0() const;

		/// <summary>
		/// Returns the second 32 bits of the hash
		/// </summary>
		uint32_t get_h1() const;

		/// <summary>
		/// Returns the third 32 bits of the hash
		/// </summary>
		uint32_t get_h2() const;

		/// <summary>
		/// Returns the fourth 32 bits of the hash
		/// </summary>
		uint32_t get_h3() const;

		/// <summary>
		/// Returns the fifth 32 bits of the hash
		/// </summary>
		uint32_t get_h4() const;

		/// <summary>
		/// Returns the hash as a string of hexadecimal characters
		/// </summary>
		std::string to_string() const;

		/// <summary>
		/// Returns the hash as a byte array
		/// </summary>
		const uint8_t* to_bytes() const;
	private:
		uint32_t h0;
		uint32_t h1;
		uint32_t h2;
		uint32_t h3;
		uint32_t h4;
		uint8_t bytes[20];
	};

	/// <summary>
	/// Hashes all available data in an input stream using SHA1
	/// </summary>
	/// <param name="in:">The input stream to hash</param>
	/// <returns>The SHA1 hash of the input stream</returns>
	const SHA1Hash hashStreamSHA1(std::istream& in);

	/// <summary>
	/// Generates a new random SHA1 hash for use in systems such as identity generation
	/// </summary>
	/// <param name="r_bytes_size">Optional parameter to set the number of random bytes used in the hash function</param>
	/// <returns>A new random SHA1 hash</returns>
	const SHA1Hash generateRandomSHA1(int r_bytes_size = 100);

}