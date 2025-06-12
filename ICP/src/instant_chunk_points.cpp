//
// Created by lukas on 02.05.24.
//

#include <future>

#include "instant_chunk_points.h"
#include "SparsePointLoader.h"

namespace icp {
	std::vector<Point> loadChunkPoints(const std::vector<std::string> &files, IoConfig config) {
		auto globalWork = std::make_shared<SharedWorkManager>(std::vector<std::string>(files.begin(), files.end()), config.numThreads);

		std::vector<std::future<std::vector<Point>>> chunkPointFutures;
		for (size_t i = 0; i < config.numThreads; ++i) {
			chunkPointFutures.push_back(std::async(std::launch::async, [&]() {
				return SparsePointLoader(globalWork, config.queueDepth).loadChunkPoints();
			}));
		}

		size_t numFinished = 0;
		std::vector<Point> chunkPoints;
		while (numFinished != chunkPointFutures.size()) {
			for (auto& f : chunkPointFutures) {
				using namespace std::chrono_literals;
				if (f.valid() && f.wait_for(1ms) == std::future_status::ready) {
					auto resolved = f.get();
					chunkPoints.insert(chunkPoints.end(), resolved.begin(), resolved.end());
					++numFinished;
				}
			}
		}

		return std::move(chunkPoints);
	}

	std::pair<std::vector<Point>, std::vector<ChunkBounds>> loadChunkPointsAndBounds(const std::vector<std::string> &files, IoConfig config) {
		auto globalWork = std::make_shared<SharedWorkManager>(std::vector<std::string>(files.begin(), files.end()), config.numThreads);

		std::vector<std::future<std::pair<std::vector<Point>, std::unordered_map<std::string, ChunkBoundsSampleAccumulator>>>> chunkPointFutures;
		for (size_t i = 0; i < config.numThreads; ++i) {
			chunkPointFutures.push_back(std::async(std::launch::async, [&]() {
				return SparsePointLoader(globalWork, config.queueDepth).loadChunkPointsAndChunkBounds();
			}));
		}

		size_t numFinished = 0;
		std::vector<Point> chunkPoints;
		std::unordered_map<std::string, ChunkBoundsSampleAccumulator> chunkBoundsSamples;
		while (numFinished != chunkPointFutures.size()) {
			for (auto& f : chunkPointFutures) {
				using namespace std::chrono_literals;
				if (f.valid() && f.wait_for(1ms) == std::future_status::ready) {
					auto [points, bounds] = f.get();
					chunkPoints.insert(chunkPoints.end(), points.begin(), points.end());
					for (const auto& [file, sample] : bounds) {
						if (chunkBoundsSamples.contains(file)) {
							chunkBoundsSamples[file].color[0] += sample.color[0];
							chunkBoundsSamples[file].color[1] += sample.color[1];
							chunkBoundsSamples[file].color[2] += sample.color[2];
							chunkBoundsSamples[file].numPoints += sample.numPoints;
						} else {
							chunkBoundsSamples.insert({file, sample});
						}
					}
					++numFinished;
				}
			}
		}

		std::vector<ChunkBounds> chunkBounds{};
		for (const auto& [file, chunk] : chunkBoundsSamples) {
			chunkBounds.emplace_back();
			auto bounds = &chunkBounds[chunkBounds.size() - 1];
			bounds->min.x = chunk.min[0];
			bounds->min.y = chunk.min[1];
			bounds->min.z = chunk.min[2];
			bounds->max.x = chunk.max[0];
			bounds->max.y = chunk.max[1];
			bounds->max.z = chunk.max[2];
			bounds->rgba[0] = chunk.color[0] / chunk.numPoints;
			bounds->rgba[1] = chunk.color[1] / chunk.numPoints;
			bounds->rgba[2] = chunk.color[2] / chunk.numPoints;
			bounds->rgba[3] = 255;
		}

		return std::make_pair(std::move(chunkPoints), std::move(chunkBounds));
	}
}
