//
// Created by lherzberger on 06.05.24.
//

#pragma once

#ifndef ICP_LASHEADER_H
#define ICP_LASHEADER_H

#include <array>
#include <cstddef>
#include <optional>
#include <iostream>

#include "spdlog/spdlog.h"

#include "instant_chunk_points.h"
#include "utils.h"

namespace icp {
struct LasHeader {
	const uint8_t versionMajor;
	const uint8_t versionMinor;

	const size_t headerSize;

	/**
	 * The offset to the point data contained in the LAS/LAZ file
	 * For LAZ files this will be different than `offsetToFirstPoint` because the first 8 bytes of point data contain the chunk table offset.
	 */
	const off_t offsetToPointData;
	off_t offsetToFirstPoint;
	const size_t pointDataRecordLength;
	size_t numPoints;

	const uint8_t format;
	off_t rgbOffset;

	const std::array<double, 3> scale;
	const std::array<double, 3> offset;
	const std::array<double, 3> min;
	const std::array<double, 3> max;

	const size_t numVariableLengthRecords;

	// only LAS 1.4 and up
	off_t extendedVariableLengthRecordsOffset = 0;
	size_t numExtendedVariableLengthRecords = 0;

	size_t chunkSize = 50'000;

	struct VariableLengthRecordHeader {
		constexpr static size_t SIZE = 54;
		const uint16_t id;
		const off_t nextOffset;

		VariableLengthRecordHeader(const std::byte* buffer, off_t headerOffset):
					id(read<uint16_t>(buffer, headerOffset + 18)),
					nextOffset(static_cast<off_t>(read<uint16_t>(buffer, headerOffset + 20)) + static_cast<off_t>(SIZE) + headerOffset) {}
	};

	struct LasZipVlr {
		constexpr static uint16_t Id = 22204;
		const size_t chunkSize;

		LasZipVlr(const std::byte* buffer, off_t headerOffset):
					chunkSize(static_cast<size_t>(read<uint32_t>(buffer, headerOffset + static_cast<off_t>(VariableLengthRecordHeader::SIZE) + 12))) {}

		constexpr bool usesVariableSizeChunks() const {
			return static_cast<uint32_t>(chunkSize) == std::numeric_limits<uint32_t>::max();
		}
	};

	bool isCompressed = false;

	std::optional<off_t> chunkTableOffset = std::nullopt;

	std::vector<off_t> chunkOffsets;

	// see: https://github.com/ASPRSorg/LAS/issues/55#issuecomment-489424141
	explicit LasHeader(const std::byte* buffer, size_t bufferSize = 4096, bool isLazHint = false) :
				versionMajor(read<uint8_t>(buffer, 24)),
				versionMinor(read<uint8_t>(buffer, 25)),
				headerSize(static_cast<size_t>(read<uint16_t>(buffer, 94))),
				offsetToPointData(static_cast<off_t>(read<uint32_t>(buffer, 96))),
				pointDataRecordLength(static_cast<size_t>(read<uint16_t>(buffer, 105))),
				numPoints(static_cast<size_t>(read<uint32_t>(buffer, 107))),
				format(read<uint8_t>(buffer, 104) % 128), // LAZ adds 128 to the LAS formats
				scale({read<double>(buffer, 131), read<double>(buffer, 139), read<double>(buffer, 147)}),
				offset({read<double>(buffer, 155), read<double>(buffer, 163), read<double>(buffer, 171)}),
				min({read<double>(buffer, 187), read<double>(buffer, 203), read<double>(buffer, 219)}),
				max({read<double>(buffer, 179), read<double>(buffer, 195), read<double>(buffer, 211)}),
				numVariableLengthRecords(static_cast<size_t>(read<uint32_t>(buffer, 100))),
				extendedVariableLengthRecordsOffset(0),
				numExtendedVariableLengthRecords(0)
	{
		switch (format) {
			case 2:
				rgbOffset = 20;
				break;
			case 3:
			case 5:
				rgbOffset = 28;
				break;
			case 7:
				rgbOffset = 30;
				break;
			case 8:
				rgbOffset = 30;
				break;
			case 10:
				rgbOffset = 30;
				break;
			default:
				rgbOffset = 0;
				break;
		}

		if (versionMajor > 1 || (versionMajor == 1 && versionMinor >= 4)) {
			numPoints = static_cast<size_t>(read<uint64_t>(buffer, 247));
			extendedVariableLengthRecordsOffset = static_cast<off_t>(read<uint64_t>(buffer, 235));
			numExtendedVariableLengthRecords = static_cast<size_t>(read<uint32_t>(buffer, 243));
		}

		// chunk table stuff
		// https://github.com/LASzip/LASzip/blob/61c7574ee35a5e476fc62fca136be1dde21d79c5/src/lasreadpoint.cpp#L591
		// https://github.com/m-schuetz/CudaPlayground/blob/laszip/modules/cuda_laszip/cuda_laszip.cu#L131

		// if adaptive chunking was used, we need to read one chunk at a time but let's assume we get a chunk table for now

		auto nextVlrOffset = static_cast<off_t>(headerSize);
		for (size_t i = 0; i < numVariableLengthRecords; ++i) {
			if (nextVlrOffset + VariableLengthRecordHeader::SIZE >= bufferSize) {
				spdlog::warn("Variable length record at offset {} with header size {} exceeds buffer size {}",
											nextVlrOffset,
											VariableLengthRecordHeader::SIZE,
											bufferSize);
				spdlog::warn("Could not read all variable length records (exceeds buffer size {}). Did not find LasZIP VLR yet. Falling back to default chunk size: {}",
										 bufferSize,
										 chunkSize);
				break;
			}

			VariableLengthRecordHeader vlrHeader(buffer, nextVlrOffset);
			if (vlrHeader.id == LasZipVlr::Id) {
				LasZipVlr lasZipVlr(buffer, nextVlrOffset);
				isCompressed = true;
				chunkSize = lasZipVlr.chunkSize;
				break;
			}

			nextVlrOffset = vlrHeader.nextOffset;
		}

		isCompressed |= isLazHint;
		offsetToFirstPoint = offsetToPointData;
		if (isCompressed) {
			if (offsetToPointData + sizeof(uint64_t) >= bufferSize ) {
				spdlog::warn("Chunk table offset not in same buffer as header ({} + {} exceeds buffer size {})",
										 offsetToPointData, sizeof(uint64_t), bufferSize);
			} else {
				const auto chunkTableStart = read<int64_t>(buffer, offsetToPointData);
				if (chunkTableStart > 0) {
					chunkTableOffset = std::make_optional<off_t>(static_cast<off_t>(chunkTableStart));
				} else if (chunkTableStart == -1) {
					spdlog::warn("Chunk table offset is either in the last 8 bytes of the LAZ file or missing. Either way, we do not handle this case yet.");
				} else if (chunkTableStart == 0) {
					spdlog::warn("Chunk table offset not found!");
				}
				offsetToFirstPoint += static_cast<off_t>(sizeof(uint64_t));
				chunkOffsets.emplace_back(offsetToFirstPoint);
			}
		} else {
			for (off_t i = 0; i < numPoints; i += static_cast<off_t>(chunkSize)) {
				chunkOffsets.emplace_back((i * static_cast<off_t>(pointDataRecordLength)) + offsetToPointData);
			}
		}
	}

	constexpr bool usesVariableChunkSize() const {
		return static_cast<uint32_t>(chunkSize) == std::numeric_limits<uint32_t>::max();
	}

	constexpr bool isChunkTableMissing() const {
		return isCompressed && (chunkTableOffset.value_or(0) == 0);
	}

	void print() const {
		spdlog::info("LasHeader:\n\tnumPoints {},\n\tstride {},\n\tmin ({}, {}, {})\n\tmax ({}, {}, {})",
								 numPoints,
								 pointDataRecordLength, min[0], min[1], min[2], max[0], max[1],
								 max[2]);
	}

	/**
	 * @param buffer buffer[0] must be chunk table start
	 * @param bufferSize buffer[bufferSize - 1] must not be out-of-bounds
	 */
	void parseChunkOffsets(const std::byte* buffer, size_t bufferSize);

	Point parsePoint(const std::byte* buffer) const {
		Point point{};

		uint16_t r = 0;
		uint16_t g = 0;
		uint16_t b = 0;

		if(rgbOffset == 0){
			r = 255;
			g = 255;
			b = 255;
		}else if(rgbOffset > 0){
			r = read<uint16_t>(buffer, rgbOffset + 0);
			g = read<uint16_t>(buffer, rgbOffset + 2);
			b = read<uint16_t>(buffer, rgbOffset + 4);
			if (r > 255) r /= 256;
			if (g > 255) g /= 256;
			if (b > 255) b /= 256;
		}

		int32_t X = read<int32_t>(buffer, 0);
		int32_t Y = read<int32_t>(buffer, 4);
		int32_t Z = read<int32_t>(buffer, 8);

		point.x = static_cast<PointFloatType>((static_cast<double>(X) * scale[0]) + offset[0]);
		point.y = static_cast<PointFloatType>((static_cast<double>(Y) * scale[1]) + offset[1]);
		point.z = static_cast<PointFloatType>((static_cast<double>(Z) * scale[2]) + offset[2]);
		point.rgba[0] = static_cast<uint8_t>(r);
		point.rgba[1] = static_cast<uint8_t>(g);
		point.rgba[2] = static_cast<uint8_t>(b);
		point.rgba[3] = 255;

		static int count = 0;
		if(count < 60){
			
			std::string str = std::format("{:10L}, {:10L}, {:10L}    {:12.2f}, {:12.2f}, {:12.2f}",
				X, Y, Z, point.x, point.y, point.z
			);
			std::cout << str << std::endl;
	
			count++;
		}

		return point;
	}
};
}

#endif // ICP_LASHEADER_H
