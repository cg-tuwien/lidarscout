//
// Created by lherzberger on 12.06.24.
//

#pragma once

#ifndef ICP_CHUNKPOINTLOADERIMPL_H
#define ICP_CHUNKPOINTLOADERIMPL_H

#include <deque>
#include <ranges>

#include "instant_chunk_points.h"

#include "PointLoaderJob.h"
#include "LasFile.h"

#include "AsyncPointLoader.h"
#include "SharedWorkQueue.h"

/*
 * Needs:
 *	- from config: n loader threads with q depth
 *	- share file states & statistics between threads
 *			- LasHeader
 *			- open yes/no
 *			- chunk table yes/no
 *			- chunk points already read (also: = is done)
 */

namespace icp {
class WorkQueue {
 public:
	WorkQueue() = delete;
	WorkQueue(const std::vector<std::string>& filePaths, size_t numThreads) : files(), numThreads(numThreads), preparedJobs({}), unclaimedFiles({}), chunkTableInfos(std::make_shared<std::vector<ChunkTableInfo>>()) {
		std::ranges::copy(
				filePaths
					| std::views::filter([](const auto& path) { return hasLasOrLazExtension(path); })
					| std::views::transform([](const auto &path) { return std::move(std::make_shared<LasFile>(path)); }),
				std::back_inserter(files));

		std::vector<std::shared_ptr<LasFile>> headerRequestsVec{};
		std::copy(files.begin(), files.end(), std::back_inserter(headerRequestsVec));
		headerRequests = std::make_unique<FixedSizeWorkQueue<std::shared_ptr<LasFile>>>(headerRequestsVec);
	}

	void reportHeaderRead(const std::shared_ptr<LasFile>& lasFile) {
		++numHeadersRead;
		if (auto numChunkPoints = lasFile->numChunkPoints(); numChunkPoints.has_value()) {
			numChunkPointsTotal += numChunkPoints.value();
		}
		unclaimedFiles.push(lasFile);
	}

	void reportChunkTableRead(const std::string& path, std::optional<size_t> numChunkPoints) {
		++numChunkTablesRead;
		if (numChunkPoints.has_value()) {
			numChunkPointsTotal += numChunkPoints.value();
			{
				std::unique_lock<std::shared_mutex> lock(chunkTableInfoMutex);
				chunkTableInfos->emplace_back(path, numChunkPoints.value());
			}
		}
	}

	std::optional<std::shared_ptr<LasFile>> getWork() {
		if (auto headerRequest = headerRequests->pop(); headerRequest.has_value()) {
			return std::move(headerRequest);
		} else {
			return std::move(unclaimedFiles.pop());
		}
	}

	std::vector<std::shared_ptr<PointLoaderJob>> tryGetJobs() {
		return preparedJobs.pop_n(std::max(static_cast<int>(preparedJobs.size() / numThreads), 1));
	}

	void shareJobs(std::vector<std::shared_ptr<PointLoaderJob>> jobs) { preparedJobs.push(jobs); }

	void reportStarving() { ++numStarvingThreads; }

	void reportNotStarvingAnymore() { --numStarvingThreads; }

	bool hasStarving() const { return numStarvingThreads > 0; }

	bool allHeadersRead() const {
		return numHeadersRead.load() == files.size();
	}

	bool hasNewChunkTableInfos() const {
		std::shared_lock<std::shared_mutex> lock(chunkTableInfoMutex);
		return !chunkTableInfos->empty();
	}

	bool isDone() const {
		return headerRequests->empty() && unclaimedFiles.empty() && preparedJobs.empty() && numStarvingThreads == numThreads;
	}

	const std::vector<std::shared_ptr<LasFile>>& getFiles() const { return files; }

	void sortRemainingFiles(const std::function<bool(const std::optional<ChunkBounds>&, const std::optional<ChunkBounds>&)>& compareOp) {
		unclaimedFiles.sort([&](const auto& a, const auto& b) {
			return compareOp(a->asChunkBounds(), b->asChunkBounds());
		});
	}

	size_t getNumChunkPointsTotal() const { return numChunkPointsTotal; }

	std::vector<ChunkTableInfo> reapChunkTableInfos() {
		auto reap = std::make_shared<std::vector<ChunkTableInfo>>();
		{
			std::unique_lock<std::shared_mutex> lock(chunkTableInfoMutex);
			chunkTableInfos.swap(reap);
		}

		return {reap->cbegin(), reap->cend()};
	}

 private:
	const size_t numThreads;

	std::atomic_size_t numHeadersRead;
	std::atomic_size_t numChunkTablesRead;
	std::atomic_size_t numChunkPointsTotal;

	std::atomic_int numStarvingThreads;
	std::vector<std::shared_ptr<LasFile>> files;

	std::unique_ptr<FixedSizeWorkQueue<std::shared_ptr<LasFile>>> headerRequests;
	ResizableWorkQueue<std::shared_ptr<LasFile>> unclaimedFiles;
	ResizableWorkQueue<std::shared_ptr<PointLoaderJob>> preparedJobs;

	std::shared_ptr<std::vector<ChunkTableInfo>> chunkTableInfos;
	mutable std::shared_mutex chunkTableInfoMutex;
};

class LoaderThread {
 public:
	explicit LoaderThread(std::atomic<bool>& closing, const std::shared_ptr<WorkQueue>& workQueue, size_t queueDepth = 32, bool computeFileAverages = true):
				workQueue(workQueue),
				localWorkQueue({}),
				jobsInProcess({}),
				chunkPointsMutex(),
				chunkPoints(std::make_shared<std::vector<Point>>()),
				innerLoader(queueDepth),
				computeFileAverages(computeFileAverages)
	{
		thread = std::jthread([&]() {
			bool starving = false;
			while (!closing) {
				size_t newPendingReads = 0;
				if (auto job = tryGetWork(); job.has_value()) {
					std::shared_ptr<PointLoaderJob> toSubmit = *job;
					newPendingReads = innerLoader.submit(toSubmit.get());
					if (toSubmit->enqueued) {
						jobsInProcess[toSubmit.get()] = toSubmit;
					} else {
						spdlog::warn("could not enqueue read request, trying again later");
						localWorkQueue.push_back(toSubmit);
					}
				} else {
					newPendingReads = innerLoader.submitRemaining();
				}

				if (innerLoader.hasPendingReads()) {
					bool forceReaping = innerLoader.isQueueFull() || newPendingReads == 0;
					for (PointLoaderJob* job : innerLoader.reap(forceReaping ? 1 : 0)) {
						handleResult(*job);
						jobsInProcess.erase(job);
					}
				}

				bool shareWork = (localWorkQueue.size() > innerLoader.queueDepth * 2) && workQueue->hasStarving();
				if (shareWork) {
					size_t shareWorkSize = localWorkQueue.size() - innerLoader.queueDepth * 2;
					std::vector<std::shared_ptr<PointLoaderJob>> jobsToShare;
					jobsToShare.insert(
							jobsToShare.begin(),
							std::make_move_iterator(localWorkQueue.begin()),
							std::make_move_iterator(localWorkQueue.begin() + static_cast<long long>(shareWorkSize)));
					localWorkQueue.erase(localWorkQueue.begin(), localWorkQueue.begin() + static_cast<long long>(shareWorkSize));
					workQueue->shareJobs(jobsToShare);
				}

				if (!starving && (jobsInProcess.empty() && localWorkQueue.empty() && newPendingReads == 0)) {
					starving = true;
					workQueue->reportStarving();
				}
				if (starving && (!jobsInProcess.empty() || !localWorkQueue.empty() || newPendingReads != 0)) {
					starving = false;
					workQueue->reportNotStarvingAnymore();
				}
			}
			isDone_ = true;
		});
	}

	std::shared_ptr<std::vector<Point>> reapChunkPoints() {
		auto reap = std::make_shared<std::vector<Point>>();
		{
			std::lock_guard<std::mutex> lock(chunkPointsMutex);
			chunkPoints.swap(reap);
		}
		return std::move(reap);
	}

	bool isDone() const {
		return isDone_.load();
	}

 private:
	void addChunkPoint(Point point) {
		std::lock_guard<std::mutex> lock(chunkPointsMutex);
		chunkPoints->push_back(point);
	}

	void handleResult(const PointLoaderJob& job) {
		const auto buffer = job.data + job.bufferOffset;
		const auto bufferSize = job.dataSize - job.bufferOffset;

		switch (job.requestType) {
			case HEADER_REQUEST: {
				if (auto firstPoint = job.lasFile->initializeHeader(buffer, bufferSize); firstPoint.has_value()) {
					addChunkPoint(firstPoint.value());
				}
				workQueue->reportHeaderRead(job.lasFile);
				break;
			}
			case CHUNK_POINT_REQUEST: {
				auto point = job.lasFile->header().parsePoint(buffer);
				if (computeFileAverages) {
					job.lasFile->addChunkPointSample(point);
				}
				addChunkPoint(point);
				break;
			}
			case CHUNK_TABLE_REQUEST: {
				job.lasFile->initializeChunkTable(buffer, bufferSize);
				workQueue->reportChunkTableRead(job.lasFile->path, job.lasFile->numChunkPoints());
				bool hadNewRequest = true;
				while (hadNewRequest) {
					if (auto next =makePointLoaderJob(job.lasFile); next.has_value()) {
						localWorkQueue.emplace_back(std::move(next.value()));
					} else {
						hadNewRequest = false;
					}
				}
				break;
			}
			case CHUNK_TABLE_OFFSET_REQUEST:
				// todo: if chunk table offset not available, look for it at end of file (I think?)
			case VARIABLE_LENGTH_RECORDS_REQUEST:
				break;
		}
	}

	std::optional<std::shared_ptr<PointLoaderJob>> makePointLoaderJob(const std::shared_ptr<LasFile>& lasFile) {
		if (auto nextRequest = lasFile->nextRequest(); nextRequest.has_value()) {
			auto [type, offset] = nextRequest.value();

			auto job = std::make_shared<PointLoaderJob>();
			job->lasFile = lasFile;
			job->requestType = type;
			job->file = lasFile->file();

			size_t secSize = sectorSize();
			off_t fileOffset = offset - (offset % static_cast<off_t>(secSize));
			off_t bufferOffset = offset - fileOffset;
			bool usesTwoSectors = false;
			if (type == CHUNK_POINT_REQUEST) {
				usesTwoSectors = (secSize - bufferOffset) < lasFile->header().pointDataRecordLength;
			} else if (type == CHUNK_TABLE_REQUEST) {
				const size_t estimatedChunkTableSize = (lasFile->header().extendedVariableLengthRecordsOffset != 0 ? lasFile->header().extendedVariableLengthRecordsOffset : job->file->size) - fileOffset;
				usesTwoSectors = (secSize - job->bufferOffset) < estimatedChunkTableSize;
			}

			job->chunkOffset = offset;
			job->fileOffset = fileOffset;
			job->bufferOffset = bufferOffset;
			job->usesTwoSectors = usesTwoSectors;

			size_t dataSize = (job->usesTwoSectors ? 2 : 1) * secSize;
			job->dataSize = std::min(dataSize, job->file->size - job->fileOffset);

			return std::make_optional<std::shared_ptr<PointLoaderJob>>(std::move(job));
		}
		return std::nullopt;
	}

	std::optional<std::shared_ptr<PointLoaderJob>> tryGetWork() {
		if (!localWorkQueue.empty()) {
			auto job = localWorkQueue.front();
			localWorkQueue.pop_front();
			return std::make_optional(std::move(job));
		} else if (auto jobs = workQueue->tryGetJobs(); !jobs.empty()) {
			localWorkQueue.insert(localWorkQueue.end(), jobs.begin(), jobs.end());
			if (!localWorkQueue.empty()) {
				auto job = localWorkQueue.front();
				localWorkQueue.pop_front();
				return std::make_optional(std::move(job));
			}
		} else if (auto file = workQueue->getWork(); file.has_value()) {
			return std::move(makePointLoaderJob(file.value()));
		}
		return std::nullopt;
	}

	size_t sectorSize() const {
		return innerLoader.sectorSize();
	}

	std::jthread thread;
	std::atomic_bool isDone_ = false;
	std::shared_ptr<WorkQueue> workQueue;
	std::deque<std::shared_ptr<PointLoaderJob>> localWorkQueue;
	std::unordered_map<PointLoaderJob*, std::shared_ptr<PointLoaderJob>> jobsInProcess;
	std::mutex chunkPointsMutex;
	std::shared_ptr<std::vector<Point>> chunkPoints;
	AsyncPointLoader innerLoader;
	const bool computeFileAverages;
};

class ChunkPointLoader::ChunkPointLoaderImpl {
 public:
	ChunkPointLoaderImpl(
			const std::vector<std::string>& filePaths,
			std::function<void(const std::vector<LasFileInfo>&)> lasFileInfoCallback,
			std::function<void(const std::vector<ChunkTableInfo>&)> chunkTableInfoCallback,
			std::function<void(const std::vector<Point>&, bool)> chunkPointsCallback,
			const IoConfig& config):
				workManager(std::make_shared<WorkQueue>(filePaths, config.numThreads)),
				lasFileInfoCallback(std::move(lasFileInfoCallback)),
				chunkTableInfoCallback(std::move(chunkTableInfoCallback)),
				chunkPointsCallback(std::move(chunkPointsCallback)),
				config(config),
				closing(false),
				isDone_(false),
				loaderThreads()
	{
		for (size_t i = 0; i < config.numThreads; ++i) {
			loaderThreads.push_back(std::move(std::make_unique<LoaderThread>(
					closing,
					workManager,
					config.queueDepth,
					false)));
		}

		controllingThread = std::jthread([&]() {
			bool reportedInitialChunkBounds = false;
			while (!closing) {
				// todo: sleep interval && set sleep interval
				using namespace std::chrono_literals;
				std::this_thread::sleep_for(16ms);

				if (!closing && workManager->isDone()) {
					closing = true;
				}

				if (!reportedInitialChunkBounds && workManager->allHeadersRead()) {
					std::vector<LasFileInfo> bounds{};
					for (const auto& f : workManager->getFiles()) {
						if (auto b = f->asChunkBounds(); b.has_value()) {
							bounds.emplace_back(f->path, b.value(), f->header().numPoints);
						} else {
							spdlog::error("All headers read but file has no header: {}", f->path);
						}
					}
					this->lasFileInfoCallback(bounds);
					reportedInitialChunkBounds = true;
				}

				if (reportedInitialChunkBounds && workManager->hasNewChunkTableInfos()) {
					this->chunkTableInfoCallback(workManager->reapChunkTableInfos());
				}

				std::vector<Point> chunkPoints{};
				for (auto& loader : loaderThreads) {
					const auto loaderPoints = loader->reapChunkPoints();
					chunkPoints.insert(chunkPoints.end(), loaderPoints->begin(), loaderPoints->end());
				}
				if (!chunkPoints.empty() || closing) {
					this->chunkPointsCallback(chunkPoints, closing);
				}
			}
			// wait for loader threads to finish
			for (const auto& t : loaderThreads) {
				while (!t->isDone()) {}
			}
			isDone_ = true;
		});
	}

	void sortRemainingFiles(const std::function<bool(const std::optional<ChunkBounds>&, const std::optional<ChunkBounds>&)>& compareOp) {
		workManager->sortRemainingFiles(compareOp);
	}

	bool isDone() const {
		return isDone_.load();
	}

	bool terminate() {
		if (!closing) {
			closing = true;
		}
		return isDone();
	}

 private:
	std::shared_ptr<WorkQueue> workManager;
	std::function<void(const std::vector<LasFileInfo>&)> lasFileInfoCallback;
	std::function<void(const std::vector<ChunkTableInfo>&)> chunkTableInfoCallback;
	std::function<void(const std::vector<Point>&, bool)> chunkPointsCallback;
	icp::IoConfig config;

	std::jthread controllingThread;

	std::vector<std::unique_ptr<LoaderThread>> loaderThreads;

	std::atomic<bool> closing;
	std::atomic<bool> isDone_;
};
}

#endif	//ICP_CHUNKPOINTLOADERIMPL_H
