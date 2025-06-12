//
// Created by lherzberger on 14.05.24.
//

#include "AsyncPointLoader.h"

namespace icp {
recycle::shared_pool<ReadBatch> makeReadBatchPool(size_t maxBatchSize) {
  return {
      [batchSize = maxBatchSize]() { return std::make_shared<ReadBatch>(batchSize);},
      [](const std::shared_ptr<ReadBatch>& b) { b->reset(); }
  };
}

size_t AsyncPointLoader::submit(PointLoaderJob* job) {
  enqueue(job);
  return submitQueuedJobs();
}

size_t AsyncPointLoader::submitRemaining() {
  if (!getCurrentBatch()->empty()) {
    submitCurrentBatch();
  }
  return submitQueuedJobs();
}

std::vector<PointLoaderJob*> AsyncPointLoader::reap(int minEvents) {
  std::vector<PointLoaderJob*> jobsDone;
  int result = 0;
  int numEventsRead = 0;
  int maxEvents = std::min(numPendingReads(), queueDepth);
  int actualMin = 0;
  if (maxEvents == 0) {
    return jobsDone;
  }
  jobsDone.reserve(maxEvents);
  do {
    result = io_getevents(
        ctx,
        actualMin,
        maxEvents - numEventsRead,
        aioEvents.data() + numEventsRead,
        nullptr);
    if (result > 0) {
      numEventsRead += result;
      actualMin -= std::min(numEventsRead, actualMin);
    }
    // todo: handle errors
  } while (numEventsRead < minEvents);
  for (size_t i = 0; i < numEventsRead; ++i) {
    if (aioEvents[i].data == nullptr) {
      spdlog::error("nullptr");
    }
    //spdlog::info("res {}, res2 {}", aioEvents[i].res, aioEvents[i].res2);
    //spdlog::info("deq file: {}", static_cast<PointLoaderJob*>(aioEvents[i].data)->file->path);
    jobsDone.push_back(static_cast<PointLoaderJob*>(aioEvents[i].data));
  }
  //spdlog::info("read {} events", numEventsRead);
  numPendingReads_ -= numEventsRead;
  return jobsDone;
}

size_t AsyncPointLoader::numPendingReads() const {
  return numPendingReads_;
}

size_t AsyncPointLoader::submitQueuedJobs() {
  if (inSubmission.empty()) {
    return 0;
  }

  auto batch = inSubmission.front();

  bool waitStart = false;
  size_t numSubmitted = 0;

  auto start = std::chrono::high_resolution_clock::now();

  do {
    long nr = static_cast<long>(batch->toSubmit());
    int ret = io_submit(ctx, nr, batch->reads.data() + batch->numSubmitted);
    if (ret > 0) {
      batch->addSubmitted(static_cast<size_t>(ret));
      numSubmitted += static_cast<size_t>(ret);
      numPendingReads_ += static_cast<size_t>(ret);
      waitStart = false;
      if (batch->done()) {
        inSubmission.pop_front();
        //spdlog::info("finished batch num remaining: {}", inSubmission.size());
        if (!inSubmission.empty()) {
          batch = inSubmission.front();
        }
      }
    } else if (ret == -EINTR || !ret) {
      waitStart = false;
      continue;
    } else if (ret == -EAGAIN) {
      // either queue is full or we need to wait for something else
      // if we have to wait for more than 30s, something's horribly wrong
      if (numPendingReads() > 0) {
        break;
      } else if (!waitStart) {
        waitStart = true;
      } else if (isTimeout(start, 30)) {
        spdlog::error("aio appears to be stalled, giving up");
        throw std::runtime_error("aio stalled");
      }
      usleep(1);
      continue;
    } else if (ret == -ENOMEM) {
      // need to reap some events
      break;
    }
  } while (!batch->done());

  return numSubmitted;
}

void AsyncPointLoader::submitCurrentBatch() {
  inSubmission.push_back(currentBatch);
  currentBatch = batchPool.allocate();
}

void AsyncPointLoader::enqueue(PointLoaderJob* job) {
  job->internal = std::make_unique<InternalJobData>(getCurrentBatch(), getOrCreateBuffer(job));
  auto block = job->internal->readBatch->allocate();
  io_prep_pread(
      block,
      job->file->handle,
      reinterpret_cast<void*>(job->internal->buffer->data()),
      job->internal->buffer->size(), job->fileOffset);
  block->data = static_cast<void*>(job);
  job->data = job->internal->buffer->data();
  job->enqueued = true;
}

std::shared_ptr<ReadBatch> AsyncPointLoader::getCurrentBatch() {
  if (!currentBatch) {
    currentBatch = batchPool.allocate();
  }
  if (currentBatch->full()) {
    submitCurrentBatch();
  }
  return currentBatch;
}
}