/*
* me_crypto.hpp
* A Collection of basic cryptography functions for use with the main library
* Written by Ian Sloat
*/

#ifdef MECORE_EXPORTS

#define MECORE __declspec(dllexport)

#else

#define MECORE __declspec(dllimport)

#endif

#include <vector>
#include <istream>

#ifndef ME_CRYPTO_HPP
#define ME_CRYPTO_HPP

namespace me {
	namespace crypto {

		class MECORE SHA1Hash {
		public:
			SHA1Hash(uint32_t h0, uint32_t h1, uint32_t h2, uint32_t h3, uint32_t h4);
			uint32_t get_h0() const;
			uint32_t get_h1() const;
			uint32_t get_h2() const;
			uint32_t get_h3() const;
			uint32_t get_h4() const;
			std::string to_string() const;
			const uint8_t* to_bytes() const;
		private:
			uint32_t h0;
			uint32_t h1;
			uint32_t h2;
			uint32_t h3;
			uint32_t h4;
			uint8_t bytes[20];
		};

		/**
		Generates a parity file using a vector of file paths as input. Used to create a basic representation of a file group.
		*/
		MECORE void createParityFile(const std::vector<std::string>& input, const std::string destination);

		/**
		Creates a hash value from the provided input stream using the SHA-1 algorithm
		*/
		MECORE const SHA1Hash hashStreamSHA1(std::istream& in);

		/**
		Creates a HMAC code from the provided input stream using a variation of the HMAC-SHA-1 algorithm
		*/
		MECORE const SHA1Hash StreamToHMAC_SHA1(std::istream& in, std::istream& key);

		/**
		Creates a 1000 byte long key by continuously hashing the previous output of an SHA-1 hash function.
		This hash chain is created off of an initial set of data of arbitrary length.
		*/
		MECORE void generateSHA1PRNGKey(std::ostream& out, std::istream& init);

	}
}

#endif