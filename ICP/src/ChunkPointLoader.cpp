//
// Created by lherzberger on 12.06.24.
//

#include "instant_chunk_points.h"
#include "ChunkPointLoaderImpl.h"

namespace icp {
ChunkPointLoader::ChunkPointLoader(
	const std::vector<std::string>& files,
	std::function<void(const std::vector<LasFileInfo>&)> lasFileInfoCallback,
	std::function<void(const std::vector<ChunkTableInfo>&)> chunkTableInfoCallback,
	std::function<void(const std::vector<Point>&, bool)> chunkPointsCallback,
	icp::IoConfig config):
 pImpl(std::make_unique<ChunkPointLoader::ChunkPointLoaderImpl>(files, std::move(lasFileInfoCallback), std::move(chunkTableInfoCallback), std::move(chunkPointsCallback), config)) {}

void ChunkPointLoader::sortRemainingFiles(const std::function<bool(const std::optional<ChunkBounds>&, const std::optional<ChunkBounds>&)>& compareOp) {
	pImpl->sortRemainingFiles(compareOp);
}

bool ChunkPointLoader::isDone() const {
	return pImpl->isDone();
}

bool ChunkPointLoader::terminate() {
	return pImpl->terminate();
}

// note: this needs to be defined in the cpp file where ChunkPointLoader::ChunkPointLoaderImpl is a complete type
ChunkPointLoader::~ChunkPointLoader() = default;
}