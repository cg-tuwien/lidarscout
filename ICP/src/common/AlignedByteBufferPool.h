//
// Created by lherzberger on 14.05.24.
//

#pragma once

#ifndef ICP_ALIGNEDBYTEBUFFERPOOL_H
#define ICP_ALIGNEDBYTEBUFFERPOOL_H

#include <limits>
#include <new>
#include <memory>

#include "recycle/shared_pool.hpp"

#include "utils.h"

namespace icp {
// https://stackoverflow.com/questions/60169819/modern-approach-to-making-stdvector-allocate-aligned-memory
template<typename ElementType>
class AlignedAllocator {
 public:
  using value_type = ElementType;
  const std::align_val_t alignment{4096};

 public:
  constexpr AlignedAllocator() noexcept = default;

  constexpr AlignedAllocator(const AlignedAllocator&) noexcept = default;

  template<typename T>
  constexpr explicit AlignedAllocator(AlignedAllocator<T> const&) noexcept {}

  explicit constexpr AlignedAllocator(size_t alignment): alignment(std::align_val_t{alignment}) {
    if (!isPowerOf2(alignment)) {
      throw std::runtime_error("alignment is not a power of two");
    }
  }

  ElementType* allocate(size_t nElementsToAllocate) {
    if (nElementsToAllocate > std::numeric_limits<size_t>::max() / sizeof(ElementType)) {
      throw std::bad_array_new_length();
    }
    auto const nBytesToAllocate = nElementsToAllocate * sizeof(ElementType);
    return reinterpret_cast<ElementType*>(::operator new[](nBytesToAllocate, alignment));
  }

  void deallocate(ElementType* allocatedPointer, [[maybe_unused]] std::size_t nBytesAllocated) {
    ::operator delete[](allocatedPointer, alignment);
  }
};

using AlignedByteBuffer = std::vector<std::byte, AlignedAllocator<std::byte>>;

std::shared_ptr<AlignedByteBuffer> makeReadBuffer(size_t bufferSize, size_t alignment);

recycle::shared_pool<AlignedByteBuffer> makeReadBufferPool(size_t bufferSize, size_t alignment);
}

#endif  //ICP_ALIGNEDBYTEBUFFERPOOL_H
