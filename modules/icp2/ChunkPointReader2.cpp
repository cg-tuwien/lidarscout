
#include <print>
#include <algorithm>
#include <execution>
#include <atomic>
#include <mutex>
#include <thread>

#include "./unsuck.hpp"
#include "./ArithmeticDecoder.h"
#include "./IntegerCompressor.h"

//#include "laszip/laszip_api.h"
//
//namespace laszip{
//	#include "../src/arithmeticdecoder.hpp"
//	#include "../src/bytestreamin_array.hpp"
//};

using namespace std;

struct LazChunk{
	int64_t byteOffset = 0;
	int64_t byteSize = 0;
};

struct Point{
	double x, y, z;
	uint8_t r, g, b, a;
};

vector<LazChunk> parseChunkTable(shared_ptr<Buffer> buffer_chunkTable, int64_t offsetToPointData){

	int64_t ctVersion = buffer_chunkTable->get<uint32_t>(0);
	int64_t numChunks = buffer_chunkTable->get<uint32_t>(4);

	// println("chunk table version: {:10}", ctVersion);
	// println("num chunks:          {:10}", numChunks);

	ArithmeticDecoder* dec = new ArithmeticDecoder(buffer_chunkTable->data_u8, 8);

	 IntegerCompressor* ic = new IntegerCompressor();
	 ic->init(dec, 32, 2);
	 ic->initDecompressor();




	vector<LazChunk> chunks(numChunks);
	// read chunk byte sizes
	for (int i = 0; i < numChunks; i++) {
		int pred = (i == 0) ? 0 : chunks[i - 1].byteSize;
		int chunk_size = ic->decompress(pred, 1);
		chunks[i].byteSize = chunk_size;
	}

	int64_t firstChunkOffset = offsetToPointData + 8;
	chunks[0].byteOffset = firstChunkOffset;
	vector<int64_t> chunkByteOffsets = {firstChunkOffset};
	// chunk_starts[0] = firstChunkOffset;
	for (int i = 1; i < numChunks; i++) {
		int64_t chunkStart = chunks[i - 1].byteOffset + int64_t(chunks[i - 1].byteSize);
		chunks[i].byteOffset = chunkStart;
	}

	auto l = getSaneLocale();
	for(int i = 0; i < chunks.size(); i++){
		LazChunk chunk = chunks[i];

		// string str = format(l, "chunk[{:3}] offset: {:10L}, size: {:7L}", i, chunk.byteOffset, chunk.byteSize);
		// println("{}", str);
	}

	return chunks;
}

vector<Point> loadLaz(string file) {
	shared_ptr<Buffer> buffer = readBinaryFile(file, 0, 2 * 4096);
	int64_t filesize = fs::file_size(file);

	// println("=== loadLaz ===");

	int versionMajor = buffer->get<uint8_t>(24);
	int versionMinor = buffer->get<uint8_t>(25);

	int64_t numPoints = 0;
	if(versionMajor == 1 && versionMinor <= 2){
		numPoints = buffer->get<uint32_t>(107);
	}else{
		numPoints = buffer->get<uint64_t>(247);
	}
	// println("numPoints: {}", numPoints);

	int64_t headerSize = buffer->get<uint16_t>(94);
	int64_t offsetToPointData = buffer->get<uint16_t>(96);
	int64_t numVariableLengthRecords = buffer->get<uint32_t>(100);
	int64_t recordFormat = buffer->get<uint8_t>(104) - 128;

	double scale_x = buffer->get<double>(131);
	double scale_y = buffer->get<double>(139);
	double scale_z = buffer->get<double>(147);
	double offset_x = buffer->get<double>(155);
	double offset_y = buffer->get<double>(163);
	double offset_z = buffer->get<double>(171);
	double min_x = buffer->get<double>(187);
	double min_y = buffer->get<double>(203);
	double min_z = buffer->get<double>(219);

	int64_t offset_rgb = 0;
	if(recordFormat == 2) offset_rgb = 20;
	if(recordFormat == 3) offset_rgb = 28;
	if(recordFormat == 5) offset_rgb = 28;
	if(recordFormat == 7) offset_rgb = 30;
	if(recordFormat == 8) offset_rgb = 30;
	if(recordFormat == 10) offset_rgb = 30;
	if(recordFormat > 10) {
		println("ERROR: unsupported record format {}", recordFormat);
		exit(63225);
	}
	

	int64_t vlrOffset = headerSize;
	constexpr int64_t VLR_HEADER_SIZE = 54;
	constexpr int64_t LASZIP_VLR_ID = 22204;


	for(int vlrIndex = 0; vlrIndex < numVariableLengthRecords; vlrIndex++){

		int64_t recordID = buffer->get<uint16_t>(vlrOffset + 18);
		int64_t recordLengthAfterHeader = buffer->get<uint16_t>(vlrOffset + 20);

		if(recordID == LASZIP_VLR_ID){
			// println("found laszip vlr. index: {}", vlrIndex);
			int64_t chunkSize = buffer->get<uint32_t>(vlrOffset + VLR_HEADER_SIZE + 12);
			int64_t chunkTableStart = buffer->get<int64_t>(offsetToPointData);
			int64_t chunkTableSize = filesize - chunkTableStart;

			// println("chunkSize:           {:10}", chunkSize);
			// println("chunkTableStart:     {:10}", chunkTableStart);
			// println("chunkTableSize:      {:10}", chunkTableSize);

			shared_ptr<Buffer> buffer_chunkTable = readBinaryFile(file, chunkTableStart, chunkTableSize);

			auto chunks = parseChunkTable(buffer_chunkTable, offsetToPointData);

			vector<Point> chunkPoints(chunks.size());

			FILE* pFile;
			Buffer buffer(36);
			// uint8_t buffer[36];
			pFile = fopen (file.c_str(), "r");

			setbuf(pFile, nullptr);

			stringstream ss;
			int i = 0;
			for(LazChunk chunk : chunks){
				// auto buffer = readBinaryFile(file, chunk.byteOffset, 36);
				
				fseek(pFile, chunk.byteOffset, SEEK_SET);
				fread(buffer.data, 1, 36, pFile);

				double x = double(buffer.get<int32_t>(0)) * scale_x + offset_x;
				double y = double(buffer.get<int32_t>(4)) * scale_y + offset_y;
				double z = double(buffer.get<int32_t>(8)) * scale_z + offset_z;

				uint16_t R = buffer.get<uint16_t>(offset_rgb + 0);
				uint16_t G = buffer.get<uint16_t>(offset_rgb + 2);
				uint16_t B = buffer.get<uint16_t>(offset_rgb + 4);
				uint8_t r = R > 255 ? R / 256 : R;
				uint8_t g = G > 255 ? G / 256 : G;
				uint8_t b = B > 255 ? B / 256 : B;

				Point point;
				point.x = x;
				point.y = y;
				point.z = z;
				point.r = r;
				point.g = g;
				point.b = b;

				chunkPoints[i] = point;

				i++;

				// string line = format("{:.1f}, {:.1f}, {:.1f}, {}, {}, {}, 255", x, y, z, r, g, b);
				// ss << line << "\n";
			}

			fclose(pFile);

			return chunkPoints;

			// string str = ss.str();
			// writeFile("./test.csv", str);

		}

		vlrOffset += VLR_HEADER_SIZE + recordLengthAfterHeader;
	}
}

int main() {

	double t_start = now();

	println("test");

	//  vector<string> files = listFilesWithExtensions("D:/resources/pointclouds/NZ23_Gisborne", "laz");
	// vector<string> files = listFilesWithExtensions("D:/resources/pointclouds/NZ23_Gisborne/Addendum1", "laz");
	// vector<string> files = listFilesWithExtensions("E:/resources/pointclouds/CA13", "laz");
	vector<string> files = listFilesWithExtensions("E:/resources/pointclouds/AZ12_USFS", "laz");
	// vector<string> files = listFilesWithExtensions("F:/LidarScout/Gisborne_subsets/BF44", "laz");
	// vector<string> files = listFilesWithExtensions("E:/resources/pointclouds/CA21_Bunds", "laz");
	// vector<string> files = listFilesWithExtensions("E:/resources/pointclouds/CA21_Ferrum", "laz");

	// atomic_uint64_t totalNumPoints = 0;
	// for_each(execution::par, files.begin(), files.end(), [&totalNumPoints](string file) {
	// 	shared_ptr<Buffer> buffer = readBinaryFile(file, 0, 4096);

	// 	int versionMajor = buffer->get<uint8_t>(24);
	// 	int versionMinor = buffer->get<uint8_t>(25);

	// 	int64_t numPoints = 0;
	// 	if(versionMajor == 1 && versionMinor <= 2){
	// 		numPoints = buffer->get<uint32_t>(107);
	// 	}else{
	// 		numPoints = buffer->get<uint64_t>(247);
	// 	}

	// 	totalNumPoints.fetch_add(numPoints);
	// });

	// auto locale = getSaneLocale();
    // string strPoints = format(locale, "#points: {:L}", totalNumPoints.load());
	// println("#files: {}", files.size());
    // println("{}", strPoints);

	// double seconds = now() - t_start;

	// println("read metadata of {} laz files in {:.3f} s", files.size(), seconds);

	// {
	// 	double t_start = now();
	// 	vector<Point> chunkPoints = loadLaz("D:/resources/pointclouds/NZ23_Gisborne/CL2_BF40_2023_1000_3848.laz");
	// 	double seconds = now() - t_start;

	// 	println("read {} chunk points in {:.3f} s", chunkPoints.size(), seconds);
	// }

	{
		double t_start_batch = now();
		int n = std::min(int(files.size()), 100'000);

		// for(int i = 0; i < n; i++){
		// 	double t_start = now();
		// 	vector<Point> chunkPoints = loadLaz(files[i]);
		// 	double seconds = now() - t_start;

		// 	println("read {} chunk points in {:.3f} s", chunkPoints.size(), seconds);
		// }

		vector<Point> chunkPoints;
		mutex mtx_chunkPoints;

		auto start = files.begin();
		auto end = start + std::min(int(files.size()), n);
		//atomic_uint64_t totalNumChunkPoints = 0;
		for_each(execution::par, start, end, [&](string file) {
			vector<Point> newChunkPoints = loadLaz(file);
			//totalNumChunkPoints.fetch_add(newChunkPoints.size());

			lock_guard<mutex> lock(mtx_chunkPoints);
			chunkPoints.insert(chunkPoints.end(), newChunkPoints.begin(), newChunkPoints.end());
		});

		double seconds_batch = now() - t_start_batch;
		//println("read {} chunkpoints of {} files in {:.3f} s", totalNumChunkPoints.load(), n, seconds_batch);
		println("read {} chunkpoints of {} files in {:.3f} s", chunkPoints.size(), n, seconds_batch);


		stringstream ss;
		for(auto point : chunkPoints){
			string line = format("{:.1f}, {:.1f}, {:.1f}, {}, {}, {}, 255", 
				point.x, 
				point.y, 
				point.z, 
				point.r, 
				point.g, 
				point.b
			);

			ss << line << "\n";
		}

		string str = ss.str();
		writeFile("./chunkpoints.csv", str);
	}
	


	return 0;
}