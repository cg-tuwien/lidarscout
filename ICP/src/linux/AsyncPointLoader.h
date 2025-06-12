//
// Created by lherzberger on 14.05.24.
//

#pragma once

#ifndef ICP_ASYNCPOINTLOADER_H
#define ICP_ASYNCPOINTLOADER_H

#include <algorithm>
#include <functional>
#include <cerrno>
#include <deque>
#include <fcntl.h>
#include <shared_mutex>
#include <string>
#include <utility>

#include "libaio.h"
#include "spdlog/spdlog.h"
#include "recycle/shared_pool.hpp"

#include "AlignedByteBufferPool.h"
#include "PointLoaderJob.h"
#include "UnbufferedFile.h"
#include "utils.h"

namespace icp {
struct ReadBatch {
  std::vector<iocb> ioBlocks;
  std::vector<iocb*> reads;
  size_t maxBatchSize;
  size_t numEnqueuedReads = 0;
  size_t numSubmitted = 0;
  explicit ReadBatch(size_t maxBatchSize) : ioBlocks(std::vector<iocb>(maxBatchSize)), reads(std::vector<iocb*>()), maxBatchSize(maxBatchSize) {
    reads.reserve(maxBatchSize);
    for (size_t i = 0; i < maxBatchSize; ++i) {
      reads.push_back(&ioBlocks[i]);
    }
  }
  iocb* allocate() {
    return &ioBlocks[numEnqueuedReads++];
  }
  size_t size() const {
    return numEnqueuedReads;
  }
  bool empty() const {
    return size() == 0;
  }
  bool full() const {
    return numEnqueuedReads == maxBatchSize;
  }
  void addSubmitted(size_t numSubmitted) {
    this->numSubmitted += numSubmitted;
  }
  size_t toSubmit() const {
    return numEnqueuedReads - numSubmitted;
  }
  bool done() const {
    return numSubmitted == numEnqueuedReads;
  }
  void reset() {
    numEnqueuedReads = 0;
    numSubmitted = 0;
  }
};

struct InternalJobData {
  std::shared_ptr<ReadBatch> readBatch;
  std::shared_ptr<AlignedByteBuffer> buffer;
};

recycle::shared_pool<ReadBatch> makeReadBatchPool(size_t maxBatchSize);

class AsyncPointLoader {
 public:
  const static size_t BLOCK_SIZE = 4096;

  AsyncPointLoader() = delete;

  explicit AsyncPointLoader(size_t queueDepth = 32) :
        ctx(nullptr),
        queueDepth(queueDepth),
        batchPool(makeReadBatchPool(queueDepth)),
        bufferPoolSingle(makeReadBufferPool(sectorSize(), sectorSize())),
        bufferPoolDouble(makeReadBufferPool(sectorSize() * 2, sectorSize())) {
    if (int err = io_queue_init(static_cast<int>(queueDepth), &ctx); err != 0) {
      spdlog::error("[AioLoaderThread]: io_queue_init error: {}", abs(err));
      throw std::runtime_error("io_queue_init");
    }
    aioEvents = std::vector<io_event>(queueDepth);
    inSubmission = {};
    numPendingReads_ = 0;
  }

  virtual ~AsyncPointLoader() {
    if (int err = io_destroy(ctx); err != 0) {
      spdlog::error("[AioLoaderThread]: could not destroy io ctx.");
    }
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
  size_t submitQueuedJobs();
  void submitCurrentBatch();
  void enqueue(PointLoaderJob* job);
  std::shared_ptr<ReadBatch> getCurrentBatch();

 public:
  const size_t queueDepth;
 private:
  recycle::shared_pool<AlignedByteBuffer> bufferPoolSingle;
  recycle::shared_pool<AlignedByteBuffer> bufferPoolDouble;

  io_context_t ctx;
  std::vector<io_event> aioEvents;
  std::deque<std::shared_ptr<ReadBatch>> inSubmission;
  recycle::shared_pool<ReadBatch> batchPool;
  std::shared_ptr<ReadBatch> currentBatch;
  size_t numPendingReads_ = 0;
};
}
#endif  //ICP_ASYNCPOINTLOADER_H
