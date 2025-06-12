
#include "ChunkPointLoader.h"

#include "arithmeticdecoder.hpp"
#include "bytestreamin_array.hpp"
#include "integercompressor.hpp"

namespace icp2{

	std::unique_ptr<ByteStreamInArray> makeByteStreamArray(const U8* buffer, size_t bufferSize) {
		if constexpr (std::endian::native == std::endian::big) {
			return std::make_unique<ByteStreamInArrayBE>(buffer, bufferSize);
		} else {
			// use little endian for LE and mixed endian systems
			return std::make_unique<ByteStreamInArrayLE>(buffer, bufferSize);
		}
	}

	vector<LazChunk> ChunkPointLoader::parseChunkTable(shared_ptr<Buffer> buffer_chunkTable, int64_t offsetToPointData){

		int64_t ctVersion = buffer_chunkTable->get<uint32_t>(0);
		int64_t numChunks = buffer_chunkTable->get<uint32_t>(4);

		// println("chunk table version: {:10}", ctVersion);
		// println("num chunks:          {:10}", numChunks);

		// laszip_stuff::ArithmeticDecoder* dec = new laszip_stuff::ArithmeticDecoder(buffer_chunkTable->data_u8, 8);

		// laszip_stuff::IntegerCompressor* ic = new laszip_stuff::IntegerCompressor();
		// ic->init(dec, 32, 2);
		// ic->initDecompressor();

		uint8_t* buffer = buffer_chunkTable->data_u8;
		size_t bufferSize = buffer_chunkTable->size;



		constexpr size_t chunkTableHeaderSize = sizeof(uint32_t) * 2;
		auto byteStream = makeByteStreamArray(reinterpret_cast<const U8*>(buffer + chunkTableHeaderSize), bufferSize - chunkTableHeaderSize);
		// auto byteStream = makeByteStreamArray(reinterpret_cast<const U8*>(buffer + chunkTableHeaderSize), bufferSize - chunkTableHeaderSize);
		auto decoder = std::make_unique<ArithmeticDecoder>();
		decoder->init(byteStream.get(), true);

		IntegerCompressor compressor(decoder.get(), 32, 2);
		compressor.initDecompressor();

		vector<LazChunk> chunks(numChunks);
		// read chunk byte sizes
		// for (int i = 0; i < numChunks; i++) {
		// 	int pred = (i == 0) ? 0 : chunks[i - 1].byteSize;
		// 	// int chunk_size = ic->decompress(pred, 1);
		// 	int chunk_size = compressor.decompress(pred, 0);
		// 	chunks[i].byteSize = chunk_size;
		// }
		for (int64_t i = 0; i < numChunks; ++i) {
			int32_t pred = (i > 0) ? chunks[i - 1].byteSize : 0;
		
			int64_t byteSize = compressor.decompress(pred, 1);
			chunks[i].byteSize = byteSize;
		}

		int64_t firstChunkOffset = offsetToPointData + 8;
		chunks[0].byteOffset = firstChunkOffset;
		vector<int64_t> chunkByteOffsets = {firstChunkOffset};
		// chunk_starts[0] = firstChunkOffset;
		for (int i = 1; i < numChunks; i++) {
			int64_t chunkStart = chunks[i - 1].byteOffset + int64_t(chunks[i - 1].byteSize);
			chunks[i].byteOffset = chunkStart;
		}

		decoder->done();

		// auto l = getSaneLocale();
		// for(int i = 0; i <= 60; i++){
		// 	LazChunk chunk = chunks[i];

		// 	string str = format(l, "chunk[{:3}] offset: {:10L}, size: {:7L}", i, chunk.byteOffset, chunk.byteSize);
		// 	println("{}", str);
		// }

		return chunks;
	}
};
