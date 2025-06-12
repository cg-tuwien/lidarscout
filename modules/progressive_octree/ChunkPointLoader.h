#pragma once

#include "instant_chunk_points.h"

#include <functional>
#include <vector>
#include <thread>
#include <mutex>
#include <algorithm>
#include <execution>
#include <atomic>
#include <format>

#include <glm/glm.hpp>

#include "unsuck.hpp"

// namespace laszip_stuff{
// 	#include "ArithmeticDecoder.h"
// 	#include "IntegerCompressor.h"
// };

using std::vector;
using std::string;
using std::function;
using std::atomic_uint64_t;
using std::for_each;
using std::println;
using std::jthread;
using std::format;
using std::lock_guard;
using std::mutex;

// have to put it in a namespace to avoid duplicate symbols from including laszip files

namespace icp2{

	struct LazChunk{
		int64_t byteOffset = 0;
		int64_t byteSize = 0;
	};

	struct LazMetadata{
		string path;

		int64_t versionMajor;
		int64_t versionMinor;
		int64_t numPoints;
		int64_t headerSize;
		int64_t offsetToPointData;
		int64_t numVariableLengthRecords;
		int64_t recordFormat;
		int64_t filesize;

		glm::dvec3 scale;
		glm::dvec3 offset;
		glm::dvec3 min;
		glm::dvec3 max;

		int64_t chunkSize = 0;
		int64_t chunkTableStart = 0;
		int64_t chunkTableSize = 0;
	};

struct ChunkPointLoader{

	bool isIdle = true;
	bool everythingLoaded = false;

	function<void(const vector<icp::LasFileInfo>&)> lasFileInfoCallback;
	function<void(const vector<icp::ChunkTableInfo>&)> chunkTableInfoCallback;
	function<void(const vector<icp::Point>&, bool)> chunkPointsCallback;
	vector<string> files;
	vector<icp::LasFileInfo> infos;
	vector<LazMetadata> metadatas;
	vector<icp::ChunkTableInfo> chunkTableInfos;
	vector<vector<LazChunk>> perTileLazChunks;
	vector<int64_t> indices;


	ChunkPointLoader(
		vector<string> files,
		function<void(const vector<icp::LasFileInfo>&)> lasFileInfoCallback,
		function<void(const vector<icp::ChunkTableInfo>&)> chunkTableInfoCallback,
		function<void(const vector<icp::Point>&, bool)> chunkPointsCallback,
		icp::IoConfig config = {}
	) 
	{
		this->lasFileInfoCallback = lasFileInfoCallback;
		this->chunkTableInfoCallback = chunkTableInfoCallback;
		this->chunkPointsCallback = chunkPointsCallback;
		this->files = files;
		this->infos = vector<icp::LasFileInfo>(files.size());
		this->metadatas = vector<LazMetadata>(files.size());
		this->chunkTableInfos = vector<icp::ChunkTableInfo>(files.size());
		this->perTileLazChunks = vector<vector<LazChunk>>(files.size());
		this->indices = vector<int64_t>(files.size(), 0);

		for(int i = 0; i < files.size(); i++) {
			this->indices[i] = i;
		}

		loadFileInfo();
	}


	~ChunkPointLoader(){

	}
	
	vector<LazChunk> parseChunkTable(shared_ptr<Buffer> buffer_chunkTable, int64_t offsetToPointData);

	void loadChunkPoints(){

		double t_start = now();

		mutex mtx;

		atomic_uint64_t counter = 0;
		for_each(std::execution::par, indices.begin(), indices.end(), [&](int64_t index) {

			// lock_guard<mutex> lock(mtx);
			
			string file = files[index];
			LazMetadata metadata = metadatas[index];
			vector<LazChunk>& chunks = perTileLazChunks[index];

			int64_t offset_rgb = 0;
			if(metadata.recordFormat == 2) offset_rgb = 20;
			if(metadata.recordFormat == 3) offset_rgb = 28;
			if(metadata.recordFormat == 5) offset_rgb = 28;
			if(metadata.recordFormat == 7) offset_rgb = 30;
			if(metadata.recordFormat == 8) offset_rgb = 30;
			if(metadata.recordFormat == 10) offset_rgb = 30;
			if(metadata.recordFormat > 10) {
				println("ERROR: unsupported record format {}", metadata.recordFormat);
				exit(63225);
			}

			Buffer buffer(40);

			FILE* pFile;
			pFile = fopen(file.c_str(), "rb");
			setbuf(pFile, nullptr);

			vector<icp::Point> points(chunks.size());
			for(int i = 0; i < chunks.size(); i++){
				LazChunk chunk = chunks[i];

				auto success = _fseeki64(pFile, chunk.byteOffset, SEEK_SET);
				auto readCount = fread(buffer.data_u8, 1, 36, pFile);

				int X = buffer.get<int32_t>(0);
				int Y = buffer.get<int32_t>(4);
				int Z = buffer.get<int32_t>(8);

				double x = double(X) * metadata.scale.x + metadata.offset.x;
				double y = double(Y) * metadata.scale.y + metadata.offset.y;
				double z = double(Z) * metadata.scale.z + metadata.offset.z;

				// bool isOutsideX = x < metadata.min.x || x > metadata.max.x;
				// bool isOutsideY = y < metadata.min.y || y > metadata.max.y;
				// bool isOutsideZ = z < metadata.min.z || z > metadata.max.z;
				//if(isOutsideX || isOutsideY || isOutsideZ){
				//	println("outside! i = {}, byteOffset: {}, byteSize: {}", i, chunk.byteOffset, chunk.byteSize);
				//	int a = 10;
				//}

				uint16_t R = buffer.get<uint16_t>(offset_rgb + 0);
				uint16_t G = buffer.get<uint16_t>(offset_rgb + 2);
				uint16_t B = buffer.get<uint16_t>(offset_rgb + 4);
				uint8_t r = R > 255 ? R / 256 : R;
				uint8_t g = G > 255 ? G / 256 : G;
				uint8_t b = B > 255 ? B / 256 : B;

				icp::Point point;
				point.x = x;
				point.y = y;
				point.z = z;
				point.rgba[0] = r;
				point.rgba[1] = g;
				point.rgba[2] = b;
				point.rgba[3] = 255;

				points[i] = point;
			}

			fclose(pFile);

			//int64_t count = counter.fetch_add(1);
			//bool isLast = count == files.size();
			bool isLast = false;

			
			chunkPointsCallback(points, isLast);
		});

		double seconds = now() - t_start;
		println("loaded chunk points of {} tiles in {:.3f} s.", files.size(), seconds);

		this->everythingLoaded = true;
	}

	void loadChunkTables(){

		println("start loading chunk tables");

		double t_start = now();

		for_each(std::execution::par, indices.begin(), indices.end(), [&](int64_t index) {

			string file = files[index];
			LazMetadata metadata = metadatas[index];

			if (metadata.chunkTableStart == 0 || metadata.chunkTableSize == 0) {
				println("chunk table was not found?");
				exit(6327236);
			}

			shared_ptr<Buffer> buffer_chunkTable = readBinaryFile(file, metadata.chunkTableStart, metadata.chunkTableSize);

			vector<LazChunk> chunks = parseChunkTable(buffer_chunkTable, metadata.offsetToPointData);

			chunkTableInfos[index].numChunkPoints = chunks.size();
			chunkTableInfos[index].path = file;

			perTileLazChunks[index] = chunks;
		});

		double seconds = now() - t_start;
		println("loaded chunk tables of {} tiles in {:.3f} s.", files.size(), seconds);

		chunkTableInfoCallback(chunkTableInfos);

		loadChunkPoints();

	}

	void loadFileInfo(){

		jthread t([=](){

			double t_start = now();

			for_each(std::execution::par, indices.begin(), indices.end(), [&](int64_t index) {

				string file = files[index];
				shared_ptr<Buffer> buffer = readBinaryFile(file, 0, 4096);

				LazMetadata header;

				header.path = file;
				header.filesize = fs::file_size(file);
				header.versionMajor = buffer->get<uint8_t>(24);
				header.versionMinor = buffer->get<uint8_t>(25);

				if(header.versionMajor == 1 && header.versionMinor <= 2){
					header.numPoints = buffer->get<uint32_t>(107);
				}else{
					header.numPoints = buffer->get<uint64_t>(247);
				}

				header.headerSize = buffer->get<uint16_t>(94);
				header.offsetToPointData = buffer->get<uint16_t>(96);
				header.numVariableLengthRecords = buffer->get<uint32_t>(100);
				header.recordFormat = buffer->get<uint8_t>(104) - 128;

				header.scale.x = buffer->get<double>(131);
				header.scale.y = buffer->get<double>(139);
				header.scale.z = buffer->get<double>(147);

				header.offset.x = buffer->get<double>(155);
				header.offset.y = buffer->get<double>(163);
				header.offset.z = buffer->get<double>(171);

				header.min.x = buffer->get<double>(187);
				header.min.y = buffer->get<double>(203);
				header.min.z = buffer->get<double>(219);

				header.max.x = buffer->get<double>(179);
				header.max.y = buffer->get<double>(195);
				header.max.z = buffer->get<double>(211);

				int64_t vlrOffset = header.headerSize;
				constexpr int64_t VLR_HEADER_SIZE = 54;
				constexpr int64_t LASZIP_VLR_ID = 22204;

				int64_t byteOffset = vlrOffset;
				for(int vlrIndex = 0; vlrIndex < header.numVariableLengthRecords; vlrIndex++){

					int64_t recordID = buffer->get<uint16_t>(byteOffset + 18);
					int64_t recordLengthAfterHeader = buffer->get<uint16_t>(byteOffset + 20);

					if(recordID == LASZIP_VLR_ID){
						header.chunkSize = buffer->get<uint32_t>(byteOffset + VLR_HEADER_SIZE + 12);
						header.chunkTableStart = buffer->get<int64_t>(header.offsetToPointData);
						header.chunkTableSize = header.filesize - header.chunkTableStart;
					}

					byteOffset = byteOffset + VLR_HEADER_SIZE + recordLengthAfterHeader;
				}


				icp::LasFileInfo info;
				info.numPoints = header.numPoints;
				info.bounds.min.x = header.min.x;
				info.bounds.min.y = header.min.y;
				info.bounds.min.z = header.min.z;
				info.bounds.max.x = header.max.x;
				info.bounds.max.y = header.max.y;
				info.bounds.max.z = header.max.z;
				info.path = file;

				infos[index] = info;
				metadatas[index] = header;
			});

			double seconds = now() - t_start;
			println("loaded metadata of {} tiles in {:.3f} s.", files.size(), seconds);

			lasFileInfoCallback(infos);

			loadChunkTables();
		});

		t.detach();

		


	}

	void sortRemainingFiles(const std::function<bool(const std::optional<icp::ChunkBounds>&, const std::optional<icp::ChunkBounds>&)>& compareOp){

	}

	bool isDone() const {
		return everythingLoaded;
	}

	bool terminate(){
		return isIdle;
	}

};

};