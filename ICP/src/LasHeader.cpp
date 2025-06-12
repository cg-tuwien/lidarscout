//
// Created by lherzberger on 6/19/2024.
//

#include <bit>
#include <print>
#include <string>
#include <format>
#include <iostream>


#include "LasHeader.h"

#include "arithmeticdecoder.hpp"
#include "bytestreamin_array.hpp"
#include "integercompressor.hpp"

namespace icp {
std::unique_ptr<ByteStreamInArray> makeByteStreamArray(const U8* buffer, size_t bufferSize) {
	if constexpr (std::endian::native == std::endian::big) {
		return std::make_unique<ByteStreamInArrayBE>(buffer, bufferSize);
	} else {
		// use little endian for LE and mixed endian systems
		return std::make_unique<ByteStreamInArrayLE>(buffer, bufferSize);
	}
}

void LasHeader::parseChunkOffsets(const std::byte* buffer, size_t bufferSize) {
	if (isChunkTableMissing()) {
		return;
	}
	auto numChunks = static_cast<size_t>(read<uint32_t>(buffer, sizeof(uint32_t)));

	constexpr size_t chunkTableHeaderSize = sizeof(uint32_t) * 2;
	auto byteStream = makeByteStreamArray(reinterpret_cast<const U8*>(buffer + chunkTableHeaderSize), bufferSize - chunkTableHeaderSize);
	auto decoder = std::make_unique<ArithmeticDecoder>();
	decoder->init(byteStream.get(), true);

	IntegerCompressor compressor(decoder.get(), 32, 2);
	compressor.initDecompressor();

	for (off_t i = 1; i < numChunks; ++i) {
		chunkOffsets.emplace_back(
				static_cast<off_t>(compressor.decompress(static_cast<int32_t>(i > 1 ? chunkOffsets[i - 1] : 0), 1)));
	}

	decoder->done();

	for (off_t i = 1; i < numChunks; ++i) {
		chunkOffsets[i] += chunkOffsets[i - 1];
	}

	for(int i = 0; i <= 60; i++){
		std::string str = std::format("chunk[{:3}] offset: {:10L}, size: -", i, chunkOffsets[i]);
		// std::println("{}", str);
		std::cout << str << std::endl;
	}
}
}
