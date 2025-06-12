//
// Created by lukas on 02.05.24.
//

#pragma once

#ifndef INSTANT_CHUNK_POINTS_INSTANT_CHUNK_POINTS_H
#define INSTANT_CHUNK_POINTS_INSTANT_CHUNK_POINTS_H

#include <functional>
#include <future>
#include <optional>
#include <string>
#include <vector>

#ifndef ICP_POINT_FLOAT_TYPE
#define ICP_POINT_FLOAT_TYPE double
#endif

namespace icp {
	using PointFloatType = ICP_POINT_FLOAT_TYPE;

	struct Point {
		PointFloatType x, y, z;
		union {
			uint32_t color;
			uint8_t rgba[4];
		};
	};

	struct ChunkBounds {
		struct {
			double x,y,z;
		} min;
		struct {
			double x,y,z;
		} max;
		union {
			uint32_t color;
			uint8_t rgba[4];
		};
	};

	struct LasFileInfo {
		std::string path;
		ChunkBounds bounds;
		size_t numPoints;
	};

	struct ChunkTableInfo {
		std::string path;
		size_t numChunkPoints;
	};

	struct IoConfig {
		size_t numThreads = 16;
		size_t queueDepth = 32;
	};

	std::vector<Point> loadChunkPoints(const std::vector<std::string> &files, IoConfig config = {});

	std::pair<std::vector<Point>, std::vector<ChunkBounds>> loadChunkPointsAndBounds(const std::vector<std::string> &files, IoConfig config = {});

	inline std::future<std::vector<Point>> loadChunkPointsAsync(const std::vector<std::string> &files, IoConfig config = {}) {
		return std::move(std::async(std::launch::async, [&]() {
			return loadChunkPoints(files, config);
		}));
	}

	inline std::future<std::pair<std::vector<Point>, std::vector<ChunkBounds>>> loadChunkPointsAndBoundsAsync(const std::vector<std::string> &files, IoConfig config = {}) {
		return std::move(std::async(std::launch::async, [&]() {
			return loadChunkPointsAndBounds(files, config);
		}));
	}

	inline std::future<bool> loadChunkPointsWithCallback(const std::vector<std::string> &files, std::function<void(std::vector<Point>)> callback, IoConfig config = {}) {
		return std::move(std::async(std::launch::async, [&]() {
			auto chunkPoints = loadChunkPoints(files, config);
			callback(std::move(chunkPoints));
			return true;
		}));
	}

	inline std::future<bool> loadChunkPointsAndBoundsWithCallback(const std::vector<std::string> &files, std::function<void(std::pair<std::vector<Point>, std::vector<ChunkBounds>>)> callback, IoConfig config = {}) {
		return std::move(std::async(std::launch::async, [&]() {
			auto chunkData = loadChunkPointsAndBounds(files, config);
			callback(std::move(chunkData));
			return true;
		}));
	}

	/**
	 * An asynchronous chunk point loader.
	 * Loads all chunk points of a given sequence of LAS/LAZ files in the background.
	 */
	class ChunkPointLoader {
	 public:
		/**
		 * Constructs an instance of ChunkPointLoader.
		 * @param files paths of LAS/LAZ files for which chunk points should be loaded.
		 * @param lasFileInfoCallback A callback that is called once as soon as all headers have been read.
		 * @param chunkTableInfoCallback A callback that is called periodically with all chunk table infos that have been read since the last call of the callback.
		 * @param chunkPointsCallback A callback that is called periodically with all chunk points that have been loaded since the last call of the callback. Takes an additional boolean as input that is true if there are no remaining chunk points to be loaded.
		 * @param config I/O specific configuration (@link IoConfig).
		 */
		ChunkPointLoader(
				const std::vector<std::string>& files,
				std::function<void(const std::vector<LasFileInfo>&)> lasFileInfoCallback,
				std::function<void(const std::vector<ChunkTableInfo>&)> chunkTableInfoCallback,
				std::function<void(const std::vector<Point>&, bool)> chunkPointsCallback,
				IoConfig config = {}
		);

		virtual ~ChunkPointLoader();

		/**
		 * Sorts all LAS/LAZ files for which chunk points have not been loaded yet based on a given comparison operation.
		 *
		 * Example usage for sorting files based on distance to camera:
		 *	 chunkPointLoader.sortRemainingFiles([&camera](const auto& a, const auto& b) {
		 *			// if either a or b has no bounds, make sure we read its header first
		 *			if (!a.has_value()) {
		 *					return true;
		 *			} else if (!b.has_value()) {
		 *					return false;
		 *			}
		 *
		 *			if (camera.isInFrustum(a)) {
		 *					if (camera.isInFrustum(b)) {
		 *							return camera.distance(a) < camera.distance(b);
		 *					}
		 *					return true;
		 *			} else if (camera.isInFrustum(b)) {
		 *					return false;
		 *			} else {
		 *					return camera.distance(a) < camera.distance(b);
		 *			}
		 *	 });
		 *
		 * @param compareOp The comparison operation to sort the remaining LAS/LAZ files. Returns true if the first element is smaller than the second one. Takes optional {@link ChunkBounds} objects representing the bounding boxes of the two files. If a file's header has not been loaded yet, i.e., its bounds are not know yet, the comparison operation is given an empty optional in its place.
		 */
		void sortRemainingFiles(const std::function<bool(const std::optional<ChunkBounds>&, const std::optional<ChunkBounds>&)>& compareOp);

		/**
		 * Checks if all chunk points have been loaded yet.
		 * @return Returns true, if all chunk points have been loaded, false otherwise.
		 */
		bool isDone() const;

		/**
		 * Terminates all remaining I/O.
		 * @return Returns true if none of this loader's callbacks will be called after this call.
		 */
		bool terminate();

	 private:
		class ChunkPointLoaderImpl;
		std::unique_ptr<ChunkPointLoaderImpl> pImpl;
	};
}


#endif //INSTANT_CHUNK_POINTS_INSTANT_CHUNK_POINTS_H
