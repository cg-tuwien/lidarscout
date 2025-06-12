//
// Created by lherzberger on 14.05.24.
//

#include "AsyncPointLoader.h"

namespace icp {
size_t AsyncPointLoader::submit(PointLoaderJob* job) {
  job->internal = std::make_unique<InternalJobData>();
  job->internal->buffer = getOrCreateBuffer(job);
  job->data = job->internal->buffer->data();
  job->internal->complete = false;
  job->internal->success = false;

  IORING_BUFFER_REF bufferRef = registeredBufferIndices.contains(job->internal->buffer.get()) ? IoRingBufferRefFromIndexAndOffset(registeredBufferIndices[job->internal->buffer.get()], 0) : IoRingBufferRefFromPointer(job->data);
  if (bufferRef.Kind != IORING_REF_REGISTERED) {
    spdlog::debug("buffer not registered");
  }

  IORING_HANDLE_REF handleRef = registeredFileIndices.contains(job->file.get()) ? IoRingHandleRefFromIndex(registeredFileIndices[job->file.get()]) : IoRingHandleRefFromHandle(job->file->handle);
  if (handleRef.Kind != IORING_REF_REGISTERED) {
    spdlog::debug("file not registered");
  }

  auto result = BuildIoRingReadFile(
      ctx,
      handleRef,
      bufferRef,
      static_cast<uint32_t>(job->internal->buffer->size()),
      static_cast<uint64_t>(job->fileOffset),
      (UINT_PTR)job,
      IOSQE_FLAGS_NONE);

  job->enqueued = result == S_OK;
  if (job->enqueued) {
    ++currentBatchSize;
    if (currentBatchSize >= queueDepth) {
      return submitRemaining();
    } else {
      return 0;
    }
  } else {
    job->internal = nullptr;
    return submitRemaining();
  }
}

size_t AsyncPointLoader::submitRemaining() {
  UINT32 numSubmitted = 0;
  if (currentBatchSize != 0) {
    if (SubmitIoRing(ctx, 0, 0, &numSubmitted) != S_OK) {
      spdlog::error("could not submit reads");
    }
    currentBatchSize -= numSubmitted;
    numPendingReads_ += numSubmitted;
  }
  return static_cast<size_t>(numSubmitted);
}

std::vector<PointLoaderJob*> AsyncPointLoader::reap(int minEvents) {
  std::vector<PointLoaderJob*> jobsDone{};
  size_t maxEvents = std::min(numPendingReads(), queueDepth);
  if (maxEvents == 0) {
    return jobsDone;
  }
  size_t newReadsTimestampSnapshot = newReadsTimestamp;
  if (newReadsTimestampSnapshot > lastTimestampDone) {
    IORING_CQE job;
    while (jobsDone.size() < maxEvents) {
      if (auto gotCompleted = PopIoRingCompletion(ctx, &job) == S_OK; gotCompleted) {
        jobsDone.push_back((PointLoaderJob*)job.UserData);
      } else {
        lastTimestampDone = newReadsTimestampSnapshot;
      }
    }
  }
  numPendingReads_ -= jobsDone.size();
  return jobsDone;
}

size_t AsyncPointLoader::numPendingReads() const {
  return numPendingReads_;
}
}
