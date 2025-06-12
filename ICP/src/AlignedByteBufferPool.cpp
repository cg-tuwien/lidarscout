//
// Created by lherzberger on 14.05.24.
//

#include "AlignedByteBufferPool.h"

namespace icp {
std::shared_ptr<AlignedByteBuffer> makeReadBuffer(size_t bufferSize, size_t alignment) {
	static std::unordered_map<size_t, AlignedAllocator<std::byte>> alignedAllocators = {
			{alignment, AlignedAllocator<std::byte>(alignment)}};
	if (!alignedAllocators.contains(alignment)) {
		alignedAllocators.insert({alignment, AlignedAllocator<std::byte>(alignment)});
	}
	return std::make_shared<AlignedByteBuffer>(bufferSize, alignedAllocators[alignment]);
}

recycle::shared_pool<AlignedByteBuffer> makeReadBufferPool(size_t bufferSize, size_t alignment) {
	return {[size = bufferSize, align = alignment]() {
		return makeReadBuffer(size, align);
	}};
}
}