//
// Created by lukas on 02.05.24.
//

#pragma once

#ifndef ICP_SPARSEPOINTLOADER_H
#define ICP_SPARSEPOINTLOADER_H

#include <deque>
#include <optional>

#include "instant_chunk_points.h"

#include "AsyncPointLoader.h"
#include "LasHeader.h"
#include "SharedWorkQueue.h"
#include "UnbufferedFile.h"
#include "utils.h"

struct ChunkBoundsSampleAccumulator {
	double min[3];
	double max[3];
	uint32_t color[3];
	size_t numPoints;
};

namespace icp {
class SparsePointLoader {
public:
	explicit SparsePointLoader(std::shared_ptr<SharedWorkManager> globalWorkQueue, size_t queueDepth = 32):
			 globalWork(std::move(globalWorkQueue)), innerLoader(queueDepth) {
		localWorkQueue = {};
		jobsInProcess = {};
	}

	std::vector<Point> loadChunkPoints() {
		std::vector<Point> chunkPoints;
		std::unordered_map<std::string, ChunkBoundsSampleAccumulator> chunkBounds;
		loadChunkPointsInner(chunkPoints, chunkBounds, false);
		return std::move(chunkPoints);
	}

	std::pair<std::vector<Point>, std::unordered_map<std::string, ChunkBoundsSampleAccumulator>> loadChunkPointsAndChunkBounds() {
		std::vector<Point> chunkPoints;
		std::unordered_map<std::string, ChunkBoundsSampleAccumulator> chunks;
		loadChunkPointsInner(chunkPoints, chunks, true);
		return {std::move(chunkPoints), std::move(chunks)};
	}

private:
 bool isIdle() const {
	 return jobsInProcess.empty() && localWorkQueue.empty();
 }

 std::shared_ptr<PointLoaderJob> makeHeaderRequest(const std::string& path) const {
	 auto job = std::make_shared<PointLoaderJob>();
	 job->file = std::make_shared<UnbufferedFile>(path);
	 job->requestType = HEADER_REQUEST;
	 job->dataSize = std::min(sectorSize(), job->file->size);
	 return std::move(job);
 }

 void greedilyClaimFiles() {
	 std::optional<std::string> path = std::nullopt;
	 do {
			if (path = globalWork->tryGetHeaderRequest(); path.has_value() && hasLasOrLazExtension(path.value())) {
				localWorkQueue.push_back(std::move(makeHeaderRequest(path.value())));
			}
	 } while (path);
 }

	void loadChunkPointsInner(std::vector<Point>& chunkPoints, std::unordered_map<std::string, ChunkBoundsSampleAccumulator>& chunkBounds, bool includeChunkBounds = false);

	bool enqueueChunkPointRequest(const std::shared_ptr<UnbufferedFile>& file, const std::shared_ptr<LasHeader>& header, off_t chunkPointOffset) {
		size_t secSize = sectorSize();
		off_t fileOffset = chunkPointOffset - (chunkPointOffset % static_cast<off_t>(secSize));
		off_t bufferOffset = chunkPointOffset - fileOffset;
		bool usesTwoSectors = (secSize - bufferOffset) < header->pointDataRecordLength;
		if ((fileOffset + (usesTwoSectors ? 2 : 1) * secSize) > file->size) {
			return false;
		}

		auto newJob = std::make_shared<PointLoaderJob>();
		newJob->file = file;
		newJob->header = header;
		newJob->requestType = CHUNK_POINT_REQUEST;
		newJob->chunkOffset = chunkPointOffset;
		newJob->fileOffset = fileOffset;
		newJob->bufferOffset = bufferOffset;
		newJob->usesTwoSectors = usesTwoSectors;

		size_t dataSize = (newJob->usesTwoSectors ? 2 : 1) * secSize;
		newJob->dataSize = std::min(dataSize, file->size - newJob->fileOffset);

		localWorkQueue.emplace_back(std::move(newJob));
		return true;
	}

	void enqueueChunkTableRequest(const std::shared_ptr<UnbufferedFile>& file, const std::shared_ptr<LasHeader>& header) {
		size_t secSize = sectorSize();
		auto newJob = std::make_shared<PointLoaderJob>();
		newJob->file = file;
		newJob->header = header;
		newJob->requestType = CHUNK_TABLE_REQUEST;
		newJob->chunkOffset = header->chunkTableOffset.value();
		newJob->fileOffset = newJob->chunkOffset - (newJob->chunkOffset % static_cast<off_t>(secSize));
		newJob->bufferOffset = newJob->chunkOffset - newJob->fileOffset;

		const size_t estimatedChunkTableSize = (header->extendedVariableLengthRecordsOffset != 0 ? header->extendedVariableLengthRecordsOffset : newJob->file->size) - newJob->fileOffset;
		newJob->usesTwoSectors = (secSize - newJob->bufferOffset) < estimatedChunkTableSize;

		size_t dataSize = (newJob->usesTwoSectors ? 2 : 1) * secSize;
		newJob->dataSize = std::min(dataSize, file->size - newJob->fileOffset);

		localWorkQueue.emplace_back(std::move(newJob));
	}

	static void handleNewChunkPoint(Point point, const std::string& boundsId, std::vector<Point>& chunkPoints, std::unordered_map<std::string, ChunkBoundsSampleAccumulator>& chunkBounds, bool includeChunkBounds = false) {
		chunkPoints.push_back(point);
		if (includeChunkBounds) {
			auto bounds = &chunkBounds[boundsId];
			for (size_t c = 0; c < 3; ++c) {
				bounds->color[c] += point.rgba[c];
			}
			++bounds->numPoints;
		}
	}

	void handleResult(const PointLoaderJob& job, std::vector<Point>& chunkPoints, std::unordered_map<std::string, ChunkBoundsSampleAccumulator>& chunkBounds, bool includeChunkBounds = false) {
		if (job.isHeaderRequest()) {
			const auto header = std::make_shared<LasHeader>(
					job.data + job.bufferOffset,
					job.dataSize,
					hasLazExtension(job.file->path));

			if (includeChunkBounds) {
				ChunkBoundsSampleAccumulator chunk{};
				for (size_t c = 0; c < 3; ++c) {
					chunk.min[c] = header->min[c];
					chunk.max[c] = header->max[c];
				}
				chunkBounds.insert({job.file->path, chunk});
			}

			bool containsFirstPoint = header->offsetToFirstPoint + header->pointDataRecordLength < job.dataSize;
			if (containsFirstPoint) {
				handleNewChunkPoint(
						header->parsePoint(job.data + header->offsetToFirstPoint),
						job.file->path,
						chunkPoints,
						chunkBounds,
						includeChunkBounds);
			}

			if (header->isCompressed) {
				if (header->isChunkTableMissing()) {
					// todo: start sequential reading of chunks
					spdlog::warn("Can not handle LAZ files without chunk table yet.");
					return;
				} else {
					enqueueChunkTableRequest(job.file, header);
				}
			} else {
				for (const auto& chunkPointOffset : header->chunkOffsets) {
					// if at all, this condition will only be fulfilled for the first chunk point
					if (chunkPointOffset + header->pointDataRecordLength < job.dataSize) {
						continue;
					}
					enqueueChunkPointRequest(job.file, header, chunkPointOffset);
				}
			}
		} else if (job.isChunkPointsRequest()) {
			handleNewChunkPoint(
					job.header->parsePoint(job.data + job.bufferOffset),
					job.file->path,
					chunkPoints,
					chunkBounds,
					includeChunkBounds);
			if (job.header->isCompressed && job.header->isChunkTableMissing()) {
				// todo: enqueue next chunk read
				// I think this should be it: https://github.com/LASzip/LASzip/blob/1b98e594fc10f39c8ad12b0adcc2183df1a555ca/src/lasreadpoint.cpp#L474
				spdlog::warn("Can not handle LAZ files without chunk table yet.");
				return;
			}
		} else if (job.isChunkTableRequest()) {
			job.header->parseChunkOffsets(job.data + job.bufferOffset, job.dataSize - job.bufferOffset);
			for (const auto& chunkPointOffset : job.header->chunkOffsets) {
				// if at all, this condition will only be fulfilled for the first chunk point
				if (chunkPointOffset + job.header->pointDataRecordLength < sectorSize()) {
					continue;
				}
				enqueueChunkPointRequest(job.file, job.header, chunkPointOffset);
			}
		}
	}

	std::optional<std::shared_ptr<PointLoaderJob>> tryGetWork() {
		if (!localWorkQueue.empty()) {
			auto job = localWorkQueue.front();
			localWorkQueue.pop_front();
			return std::make_optional(std::move(job));
		} else if (const auto path = globalWork->tryGetHeaderRequest(); path.has_value() && hasLasOrLazExtension(path.value())) {
			return std::make_optional(std::move(makeHeaderRequest(path.value())));
		} else if (auto jobs = globalWork->tryGetJobs(); !jobs.empty()) {
			localWorkQueue.insert(localWorkQueue.end(), jobs.begin(), jobs.end());
			if (!localWorkQueue.empty()) {
				starving = false;
				globalWork->reportNotStarvingAnymore();
				auto job = localWorkQueue.front();
				localWorkQueue.pop_front();
				return std::make_optional(std::move(job));
			}
		}
		return std::nullopt;
	}

	size_t sectorSize() const {
		return innerLoader.sectorSize();
	}

protected:
 bool starving = false;
	std::unordered_map<PointLoaderJob*, std::shared_ptr<PointLoaderJob>> jobsInProcess;

	std::shared_ptr<SharedWorkManager> globalWork;
	std::deque<std::shared_ptr<PointLoaderJob>> localWorkQueue;

	AsyncPointLoader innerLoader;
};
}

#endif //ICP_SPARSEPOINTLOADER_H
