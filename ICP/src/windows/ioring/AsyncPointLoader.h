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

#ifndef NOMINMAX
#define NOMINMAX
#endif
#include "Windows.h"
#include "ioringapi.h"

#include "spdlog/spdlog.h"
#include "recycle/shared_pool.hpp"

#include "AlignedByteBufferPool.h"
#include "LasFile.h"
#include "PointLoaderJob.h"
#include "utils.h"

namespace icp {
struct InternalJobData {
  std::shared_ptr<AlignedByteBuffer> buffer;
  std::atomic_bool complete = false;
  std::atomic_bool success = false;
};

class AsyncPointLoader {
 public:
  const static size_t BLOCK_SIZE = 4096;
  const static size_t COMPLETION_QUEUE_SIZE_FACTOR = 4;

  AsyncPointLoader() = delete;

  explicit AsyncPointLoader(size_t queueDepth = 32) :
        queueDepth(queueDepth),
        bufferPoolSingle(makeReadBufferPool(sectorSize(), sectorSize())),
        bufferPoolDouble(makeReadBufferPool(sectorSize() * 2, sectorSize())),
        registeredBufferIndices(),
        registeredFileIndices() {
    IORING_CAPABILITIES capabilities{};
    if (auto result = QueryIoRingCapabilities(&capabilities); result != S_OK) {
      throw std::runtime_error("could not query io ring capabilities");
    }
    IORING_CREATE_FLAGS ioringCreateFlags{};
    //spdlog::info("ioring submission queue size {}, completion queue size {}", capabilities.MaxSubmissionQueueSize, capabilities.MaxCompletionQueueSize);
    queueDepth = std::min(queueDepth, static_cast<size_t>(capabilities.MaxSubmissionQueueSize));
    auto result = CreateIoRing(
        capabilities.MaxVersion,
        ioringCreateFlags,
        queueDepth,
        std::min(static_cast<uint32_t>(queueDepth * COMPLETION_QUEUE_SIZE_FACTOR), capabilities.MaxCompletionQueueSize),
        &ctx);
    if (result != S_OK) {
      throw std::runtime_error("could not create io ring");
    }
    if (!IsIoRingOpSupported(ctx, IORING_OP_READ)) {
      throw std::runtime_error("io ring op read not supported");
    }
    if (!IsIoRingOpSupported(ctx, IORING_OP_REGISTER_FILES)) {
      throw std::runtime_error("io ring op register files not supported");
    }
    if (!IsIoRingOpSupported(ctx, IORING_OP_REGISTER_BUFFERS)) {
      throw std::runtime_error("io ring op register buffers not supported");
    }
    currentBatchSize = 0;

    completionEvent = CreateEvent(nullptr, false, false, nullptr);
    if (auto result = SetIoRingCompletionEvent(ctx, completionEvent); result != S_OK) {
      throw std::runtime_error("could not set io ring completion event");
    }
    closing = false;
    newReadsTimestamp = 0;
    lastTimestampDone = 0;
    pollingThread = std::thread([&](){
      while (!closing) {
        if (WaitForSingleObject(completionEvent, 250) == WAIT_OBJECT_0) {
          ResetEvent(completionEvent);
          ++newReadsTimestamp;
        }
      }
    });
  };

  virtual ~AsyncPointLoader() {
    CloseIoRing(ctx);
    closing = true;
    pollingThread.join();
  }

  void preRegisterResources(const std::deque<std::shared_ptr<PointLoaderJob>>& jobs) {
    std::vector<HANDLE> handles{};
    for (auto& job : jobs) {
      registeredFileIndices[job->file.get()] = handles.size();
      handles.push_back(job->file->handle);
    }

    std::vector<std::shared_ptr<AlignedByteBuffer>> alignedBuffers{};
    std::vector<IORING_BUFFER_INFO> bufferInfos{};
    for (size_t i = 0; i < queueDepth * COMPLETION_QUEUE_SIZE_FACTOR; ++i) {
      auto singleBuffer = bufferPoolSingle.allocate();
      auto doubleBuffer = bufferPoolDouble.allocate();

      registeredBufferIndices[singleBuffer.get()] = bufferInfos.size();
      bufferInfos.emplace_back(singleBuffer->data(), singleBuffer->size());

      registeredBufferIndices[doubleBuffer.get()] = bufferInfos.size();
      bufferInfos.emplace_back(doubleBuffer->data(), doubleBuffer->size());

      alignedBuffers.push_back(std::move(singleBuffer));
      alignedBuffers.push_back(std::move(doubleBuffer));
    }

    if (!handles.empty()) {
      if (BuildIoRingRegisterFileHandles(ctx, handles.size(), handles.data(), 0) != S_OK) {
        spdlog::warn("could not register file handles");
      } else {
        UINT32 numSubmitted = 0;
        if (SubmitIoRing(ctx, 0, 0, &numSubmitted) != S_OK || numSubmitted < 1) {
          spdlog::error("could not submit registrations");
        } else {
          IORING_CQE registrations;
          while (PopIoRingCompletion(ctx, &registrations) != S_OK) {
            Sleep(1);
          }
        }
      }
    }
    if (!bufferInfos.empty()) {
      if (BuildIoRingRegisterBuffers(ctx, bufferInfos.size(), bufferInfos.data(), 0) != S_OK) {
        spdlog::warn("could not register buffers");
      }  else {
        UINT32 numSubmitted = 0;
        if (SubmitIoRing(ctx, 0, 0, &numSubmitted) != S_OK || numSubmitted < 1) {
          spdlog::error("could not submit registrations");
        } else {
          IORING_CQE registrations;
          while (PopIoRingCompletion(ctx, &registrations) != S_OK) {
            Sleep(1);
          }
        }
      }
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

 public:
  const size_t queueDepth;
 private:
  recycle::shared_pool<AlignedByteBuffer> bufferPoolSingle;
  recycle::shared_pool<AlignedByteBuffer> bufferPoolDouble;
  size_t numPendingReads_ = 0;

  HIORING ctx;
  size_t currentBatchSize = 0;
  HANDLE completionEvent;
  std::atomic_bool closing = false;
  size_t lastTimestampDone = 0;
  std::atomic<size_t> newReadsTimestamp;
  std::thread pollingThread;

  std::unordered_map<AlignedByteBuffer*, UINT32> registeredBufferIndices;
  std::unordered_map<UnbufferedFile*, UINT32> registeredFileIndices;
};
}
#endif  //ICP_ASYNCPOINTLOADER_H
