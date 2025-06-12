//
// Created by lherzberger on 14.05.24.
//

#pragma once

#ifndef ICP_ASYNCPOINTLOADER_H
#define ICP_ASYNCPOINTLOADER_H

#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <Windows.h>
// todo: concurrent_queue is not available on MinGW -> replace with something else? maybe a ring buffer with atomically incremented head & tail?
#include "concurrent_queue.h"

#include "spdlog/spdlog.h"
#include "recycle/shared_pool.hpp"

#include "AlignedByteBufferPool.h"
#include "PointLoaderJob.h"
#include "UnbufferedFile.h"
#include "utils.h"

namespace icp {
struct InternalOverlapped {
  OVERLAPPED o;
};

struct InternalJobData {
  std::shared_ptr<AlignedByteBuffer> buffer;
  std::atomic_bool complete = false;
  std::atomic_bool success = false;
  Concurrency::concurrent_queue<PointLoaderJob*>* completedJobs = nullptr;
  std::shared_ptr<InternalOverlapped> overlapped;
};

recycle::shared_pool<InternalOverlapped> makeOverlappedPool();

class AsyncPointLoader {
 public:
  const static size_t BLOCK_SIZE = 4096;

  AsyncPointLoader() = delete;

  explicit AsyncPointLoader(size_t queueDepth = 32) :
        queueDepth(queueDepth),
        completedJobs(std::make_unique<Concurrency::concurrent_queue<PointLoaderJob*>>()),
        numPendingReads_(0),
        overlappedPool(makeOverlappedPool()),
        bufferPoolSingle(makeReadBufferPool(sectorSize(), sectorSize())),
        bufferPoolDouble(makeReadBufferPool(sectorSize() * 2, sectorSize())) {
  }

  static size_t sectorSize() {
    return BLOCK_SIZE;
  }

  size_t submit(PointLoaderJob* job);

  size_t submitRemaining();

  std::vector<PointLoaderJob*> reap(int minEvents);

  bool isQueueFull() const {
    return numPendingReads() >= queueDepth;
  }

  bool hasPendingReads() const {
    return numPendingReads() != 0;
  }

  size_t numPendingReads() const;

 private:
  std::shared_ptr<AlignedByteBuffer> getOrCreateBuffer(PointLoaderJob* job) {
    if (job->usesTwoSectors) {
      return bufferPoolDouble.allocate();
    } else {
      return bufferPoolSingle.allocate();
    }
  }
 public:
  const size_t queueDepth;
 private:
  recycle::shared_pool<AlignedByteBuffer> bufferPoolSingle;
  recycle::shared_pool<AlignedByteBuffer> bufferPoolDouble;

  size_t numPendingReads_ = 0;
  std::unique_ptr<Concurrency::concurrent_queue<PointLoaderJob*>> completedJobs;
  recycle::shared_pool<InternalOverlapped> overlappedPool;
};
}
#endif  //ICP_ASYNCPOINTLOADER_H
